import { useState, useEffect, useRef, useCallback } from "react"
import { Card, CardContent, CardTitle, CardHeader } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Wifi, WifiOff, AlertTriangle, CheckCircle } from "lucide-react"

// WebSocket connection configuration
const WEBSOCKET_URL = "ws://localhost:9001"

// Message type constants from backend protocol.zig
const MESSAGE_TYPE_JOINT_STATE = 0
const MESSAGE_TYPE_SYSTEM_STATUS = 1
const MESSAGE_TYPE_COLLISION_DATA = 2
const MESSAGE_TYPE_COMMAND = 3
const MESSAGE_TYPE_CONNECTION_STATUS = 4

// Command type constants
const COMMAND_TYPE_POSITION = 0
const COMMAND_TYPE_VELOCITY = 1
const COMMAND_TYPE_TORQUE = 2
const COMMAND_TYPE_SAFETY = 4

// TypeScript interfaces matching FE.md
interface JointState {
  position: number[]      // Current joint positions (rad)
  velocity: number[]      // Current joint velocities (rad/s)
  torque: number[]       // Current joint torques (Nm)
  temperature: number[]   // Motor temperatures (°C)
  current: number[]      // Motor currents (A)
  timestamp: number      // Timestamp in microseconds
}

interface SystemStatus {
  state: 'READY' | 'BUSY' | 'ERROR' | 'WARNING'
  errorCode: string | null
  safetyStatus: {
    softLimitsActive: boolean
    emergencyStop: boolean
    collisionDetected: boolean
  }
  controlMode: 'POSITION' | 'VELOCITY' | 'TORQUE'
}

interface CollisionData {
  detected: boolean
  link1?: string
  link2?: string
  position?: { x: number, y: number, z: number }
  penetrationDepth?: number
  contactNormal?: { x: number, y: number, z: number }
}

interface ConnectionStatus {
  status: 'connected' | 'disconnected' | 'error'
  message: string
  timestamp_us: number
}

type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error'

interface ZigControllerProps {
  controlSignals: any
  setControlSignals: (signals: any) => void
}

export function ZigController({ controlSignals, setControlSignals }: ZigControllerProps) {
  const [connectionState, setConnectionState] = useState<ConnectionState>('disconnected')
  const [showDetails, setShowDetails] = useState(false)
  const [lastMessage, setLastMessage] = useState<string>('')
  const [jointsData, setJointsData] = useState<JointState | null>(null)
  const [systemStatus, setSystemStatus] = useState<SystemStatus | null>(null)
  const [collisionData, setCollisionData] = useState<CollisionData | null>(null)
  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus | null>(null)
  
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null)
  const reconnectAttempts = useRef(0)

  // Binary deserialization helper functions
  const deserializeJointState = useCallback((payload: ArrayBuffer): JointState => {
    const expectedSize = 8 + 7*4*5 // u64 + 5 arrays of 7 f32s = 148 bytes
    
    if (payload.byteLength !== expectedSize) {
      throw new Error(`Invalid JointStateMessage size: got ${payload.byteLength}, expected ${expectedSize}`)
    }
    
    const view = new DataView(payload)
    let offset = 0

    // Parse according to backend JointStateMessage struct
    const timestamp_us = view.getBigUint64(offset, true) // little-endian u64
    offset += 8

    const positions: number[] = []
    const velocities: number[] = []
    const torques: number[] = []
    const temperatures: number[] = []
    const currents: number[] = []

    // Read 7 joints worth of data (NUM_JOINTS = 7)
    // Backend struct layout: [timestamp][positions][velocities][torques][temperatures][currents]
    for (let i = 0; i < 7; i++) {
      positions.push(view.getFloat32(offset, true)) // little-endian f32
      offset += 4
    }
    for (let i = 0; i < 7; i++) {
      velocities.push(view.getFloat32(offset, true))
      offset += 4
    }
    for (let i = 0; i < 7; i++) {
      torques.push(view.getFloat32(offset, true))
      offset += 4
    }
    for (let i = 0; i < 7; i++) {
      temperatures.push(view.getFloat32(offset, true))
      offset += 4
    }
    for (let i = 0; i < 7; i++) {
      currents.push(view.getFloat32(offset, true))
      offset += 4
    }

    if (offset !== expectedSize) {
      throw new Error(`Parsing error: read ${offset} bytes, expected ${expectedSize}`)
    }

    return {
      position: positions,
      velocity: velocities,
      torque: torques,
      temperature: temperatures,
      current: currents,
      timestamp: Number(timestamp_us)
    }
  }, [])

  // Frame serialization for outgoing commands
  const serializeFrame = useCallback((messageType: number, payloadBytes: Uint8Array): ArrayBuffer => {
    const frameSize = 1 + 4 + payloadBytes.length // type + length + payload
    const buffer = new ArrayBuffer(frameSize)
    const view = new DataView(buffer)
    
    view.setUint8(0, messageType) // message_type: u8
    view.setUint32(1, payloadBytes.length, true) // payload_length: u32 (little-endian)
    
    // Copy payload
    const payloadView = new Uint8Array(buffer, 5)
    payloadView.set(payloadBytes)
    
    return buffer
  }, [])

  // Send joint position command
  const sendJointCommand = useCallback((positions: number[], maxVelocity?: number[]) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.warn('WebSocket not connected, cannot send command')
      return
    }

    const command = {
      type: COMMAND_TYPE_POSITION,
      values: positions,
      max_velocity: maxVelocity || null
    }

    const jsonString = JSON.stringify(command)
    wsRef.current.send(jsonString)
    console.log('Sent joint command:', command)
  }, [])

  // Handle incoming WebSocket messages
  const handleMessage = useCallback((event: MessageEvent) => {
    try {
      if (event.data instanceof ArrayBuffer) {
        // Binary frame - should be joint state data
        console.log('Received binary frame, size:', event.data.byteLength)
        
        try {
          const jointState = deserializeJointState(event.data)
          setJointsData(jointState)
          
          // Update control signals for robot arm visualization
          setControlSignals((prev) => ({
            ...prev,
            jointPositions: jointState.position,
            jointVelocities: jointState.velocity,
            jointTorques: jointState.torque,
            motorTemperatures: jointState.temperature,
            motorCurrents: jointState.current,
            timestamp: jointState.timestamp
          }))
          
          setLastMessage(`Joint State: ${jointState.position.map(p => p.toFixed(2)).join(', ')}`)
        } catch (parseError) {
          console.error('Failed to parse joint state binary data:', parseError)
          console.error('Binary data size:', event.data.byteLength, 'expected:', 8 + 7*4*5)
          // Log first few bytes for debugging
          const debugView = new DataView(event.data)
          const firstBytes = []
          for (let i = 0; i < Math.min(16, event.data.byteLength); i++) {
            firstBytes.push(debugView.getUint8(i).toString(16).padStart(2, '0'))
          }
          console.error('First 16 bytes (hex):', firstBytes.join(' '))
          // Don't close connection on parse error, just log it
          return
        }
      } else if (typeof event.data === 'string') {
        // Text frame - JSON data (system status, collision, connection status)
        const jsonData = JSON.parse(event.data)
        
        // Determine message type based on JSON structure
        if (jsonData.state !== undefined && jsonData.controlMode !== undefined) {
          // System status message
          const status: SystemStatus = jsonData
          setSystemStatus(status)
          setLastMessage(`System Status: ${status.state}`)
        } else if (jsonData.detected !== undefined) {
          // Collision data message
          const collision: CollisionData = jsonData
          setCollisionData(collision)
          setLastMessage(`Collision: ${collision.detected ? 'DETECTED' : 'NONE'}`)
        } else if (jsonData.status !== undefined && jsonData.timestamp_us !== undefined) {
          // Connection status message
          const connStatus: ConnectionStatus = jsonData
          setConnectionStatus(connStatus)
          setLastMessage(`Connection: ${connStatus.status}`)
        } else {
          console.warn('Unknown JSON message structure:', jsonData)
        }
      } else {
        console.error('Received unknown message type:', typeof event.data)
      }
    } catch (error) {
      console.error('Error parsing message:', error)
    }
  }, [deserializeJointState, setControlSignals])

  // WebSocket connection management
  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      return
    }

    setConnectionState('connecting')
    
    try {
      const ws = new WebSocket(WEBSOCKET_URL)
      ws.binaryType = 'arraybuffer'
      
      ws.onopen = () => {
        console.log('✅ WebSocket connected to backend successfully')
        console.log('WebSocket readyState:', ws.readyState)
        console.log('WebSocket protocol:', ws.protocol)
        console.log('WebSocket extensions:', ws.extensions)
        setConnectionState('connected')
        reconnectAttempts.current = 0
        setLastMessage('Connected to Zig backend')
      }
      
      ws.onmessage = (event) => {
        console.log('📨 WebSocket message received, type:', typeof event.data, 'size:', event.data.length || event.data.byteLength)
        handleMessage(event)
      }
      
      ws.onclose = (event) => {
        console.log('❌ WebSocket disconnected - Code:', event.code, 'Reason:', event.reason, 'Clean:', event.wasClean)
        console.log('WebSocket close codes reference:')
        console.log('  1000 = Normal closure')
        console.log('  1006 = Abnormal closure (no close frame)')
        console.log('  1011 = Server error')
        setConnectionState('disconnected')
        setLastMessage(`Disconnected: ${event.reason || 'Connection closed'}`)
        
        // Attempt reconnection
        if (reconnectAttempts.current < 5) {
          reconnectAttempts.current++
          reconnectTimeoutRef.current = setTimeout(connect, 2000 * reconnectAttempts.current)
        }
      }
      
      ws.onerror = (error) => {
        console.error('⚠️ WebSocket error occurred:', error)
        console.error('WebSocket readyState at error:', ws.readyState)
        setConnectionState('error')
        setLastMessage('Connection error')
      }
      
      wsRef.current = ws
    } catch (error) {
      console.error('Failed to create WebSocket:', error)
      setConnectionState('error')
    }
  }, [handleMessage])

  const disconnect = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current)
      reconnectTimeoutRef.current = null
    }
    
    if (wsRef.current) {
      wsRef.current.close()
      wsRef.current = null
    }
    
    setConnectionState('disconnected')
  }, [])

  // Initialize WebSocket connection on mount
  useEffect(() => {
    connect()
    
    return () => {
      disconnect()
    }
  }, [connect, disconnect])

  // Expose sendJointCommand for use by other components
  useEffect(() => {
    if (typeof setControlSignals === 'function') {
      setControlSignals((prev) => ({
        ...prev,
        sendJointCommand
      }))
    }
  }, [sendJointCommand, setControlSignals])

  const getStatusIcon = () => {
    switch (connectionState) {
      case 'connected': return <CheckCircle className="w-4 h-4 text-green-500" />
      case 'connecting': return <Wifi className="w-4 h-4 text-yellow-500 animate-pulse" />
      case 'error': return <AlertTriangle className="w-4 h-4 text-red-500" />
      default: return <WifiOff className="w-4 h-4 text-gray-500" />
    }
  }

  const getStatusColor = () => {
    switch (connectionState) {
      case 'connected': return 'bg-green-600'
      case 'connecting': return 'bg-yellow-600'
      case 'error': return 'bg-red-600'
      default: return 'bg-gray-600'
    }
  }

  return (
    <div
      className={`absolute ${showDetails ? "bottom-4 right-4 w-96 h-96" : "bottom-4 right-4 w-auto"} bg-black/80 backdrop-blur-md rounded-lg text-white overflow-hidden transition-all duration-300`}
    >
      {!showDetails ? (
        <Button
          variant="outline"
          size="sm"
          className={`m-2 bg-transparent border-orange-600 text-orange-400 hover:bg-orange-950/30 ${getStatusColor()}`}
          onClick={() => setShowDetails(true)}
        >
          {getStatusIcon()}
          <span className="ml-2">Zig Backend</span>
          <Badge variant="outline" className="ml-2 text-xs">
            {connectionState}
          </Badge>
        </Button>
      ) : (
        <Card className="border-0 bg-transparent text-white h-full">
          <CardHeader className="pb-2">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2">
                {getStatusIcon()}
                <CardTitle className="text-lg font-mono">Zig Backend</CardTitle>
                <Badge variant="outline" className="text-xs">
                  {connectionState}
                </Badge>
              </div>
              <div className="flex gap-2">
                <Button 
                  variant="ghost" 
                  size="sm" 
                  className="h-6 text-xs"
                  onClick={connectionState === 'connected' ? disconnect : connect}
                >
                  {connectionState === 'connected' ? 'Disconnect' : 'Connect'}
                </Button>
                <Button 
                  variant="ghost" 
                  size="sm" 
                  className="h-6 text-xs" 
                  onClick={() => setShowDetails(false)}
                >
                  Close
                </Button>
              </div>
            </div>
          </CardHeader>
          <CardContent className="overflow-auto h-[calc(100%-60px)] space-y-4">
            <div className="text-xs">
              <div className="text-green-400 mb-2">Connection: {WEBSOCKET_URL}</div>
              <div className="text-yellow-400 mb-2">Last Message:</div>
              <div className="bg-black/50 p-2 rounded text-xs">{lastMessage}</div>
            </div>
            
            {jointsData && (
              <div className="text-xs">
                <div className="text-blue-400 mb-1">Joint Positions (rad):</div>
                <div className="bg-black/50 p-2 rounded text-xs font-mono">
                  {jointsData.position.map((pos, i) => `J${i}: ${pos.toFixed(3)}`).join(' | ')}
                </div>
              </div>
            )}
            
            {systemStatus && (
              <div className="text-xs">
                <div className="text-purple-400 mb-1">System Status:</div>
                <div className="bg-black/50 p-2 rounded text-xs">
                  <div>State: {systemStatus.state}</div>
                  <div>Mode: {systemStatus.controlMode}</div>
                  <div>E-Stop: {systemStatus.safetyStatus.emergencyStop ? 'ACTIVE' : 'OFF'}</div>
                </div>
              </div>
            )}
          </CardContent>
        </Card>
      )}
    </div>
  )
}