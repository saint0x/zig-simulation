import { useState, useEffect, useCallback, Suspense } from "react"
import { Canvas } from "@react-three/fiber"
import { OrbitControls, Environment, PerspectiveCamera, ContactShadows, Text } from "@react-three/drei"
import { RobotArm } from "./robot-arm"
import { ControlPanel } from "./control-panel"
import { Physics, useBox, usePlane, useSphere } from "@react-three/cannon"
import { InfoPanel } from "./info-panel"
import { ZigController } from "./zig-controller"
import { HandTrackingPanel } from "./mediapipe/hand-tracking-panel"
import type { JointAngles, DiscreteCommand, InputSource } from "@/types/mediapipe"
import { jointAnglesToArray } from "@/lib/mediapipe/mapping"

export default function RobotArmSimulation() {
  const [armAngles, setArmAngles] = useState({
    baseRotation: 0,
    shoulderRotation: 0,
    elbowRotation: 0,
    wristBendRotation: 0,
    wristRotation: 0,
    toolRotation: 0,
    gripperRotation: 0,
  })

  const [controlSignals, setControlSignals] = useState({
    motorCurrents: [0, 0, 0, 0, 0, 0, 0],
    sensorReadings: [0, 0, 0, 0, 0, 0, 0],
    systemStatus: "READY",
    errorCode: null,
    sendJointCommand: null as ((positions: number[], maxVelocity?: number[]) => void) | null,
  })

  const [inputSource, setInputSource] = useState<InputSource>('manual')
  const [eStopActive, setEStopActive] = useState(false)

  // Predefined movements
  const movements = {
    rest: {
      baseRotation: 0,
      shoulderRotation: 0,
      elbowRotation: 0,
      wristBendRotation: 0,
      wristRotation: 0,
      toolRotation: 0,
      gripperRotation: 0,
    },
    pickup: {
      baseRotation: 45,
      shoulderRotation: -30,
      elbowRotation: -60,
      wristBendRotation: -30,
      wristRotation: 0,
      toolRotation: 0,
      gripperRotation: -45,
    },
    place: {
      baseRotation: -45,
      shoulderRotation: -45,
      elbowRotation: -30,
      wristBendRotation: 0,
      wristRotation: 90,
      toolRotation: 45,
      gripperRotation: 0,
    },
    inspect: {
      baseRotation: 0,
      shoulderRotation: -90,
      elbowRotation: 0,
      wristBendRotation: -90,
      wristRotation: 0,
      toolRotation: 180,
      gripperRotation: -20,
    },
    grasp: {
      baseRotation: 0,
      shoulderRotation: -45,
      elbowRotation: -45,
      wristBendRotation: -45,
      wristRotation: 0,
      toolRotation: 0,
      gripperRotation: -90,
    },
  }

  // Handle manual angle changes
  const handleAngleChange = useCallback((joint: string, value: number) => {
    if (eStopActive) return

    setArmAngles((prev) => ({ ...prev, [joint]: value }))
    setInputSource('manual')

    // Send to backend if available
    if (controlSignals.sendJointCommand) {
      const newAngles = { ...armAngles, [joint]: value }
      const positionArray = [
        newAngles.baseRotation,
        newAngles.shoulderRotation,
        newAngles.elbowRotation,
        newAngles.wristBendRotation,
        newAngles.wristRotation,
        newAngles.toolRotation,
        newAngles.gripperRotation,
      ]
      controlSignals.sendJointCommand(positionArray)
    }
  }, [armAngles, controlSignals.sendJointCommand, eStopActive])

  // Handle preset movement execution
  const executeMovement = useCallback((movementName: string) => {
    if (eStopActive) return

    const movement = movements[movementName as keyof typeof movements]
    if (movement) {
      setArmAngles(movement)
      setInputSource('preset')

      // Send to backend
      if (controlSignals.sendJointCommand) {
        const positionArray = jointAnglesToArray(movement as JointAngles)
        controlSignals.sendJointCommand(positionArray)
      }
    }
  }, [controlSignals.sendJointCommand, eStopActive])

  // Handle hand tracking joint angles
  const handleHandTrackingAngles = useCallback((angles: JointAngles) => {
    if (eStopActive) return

    setArmAngles(angles)
    setInputSource('mediapipe')

    // Send to backend
    if (controlSignals.sendJointCommand) {
      const positionArray = jointAnglesToArray(angles)
      controlSignals.sendJointCommand(positionArray)
    }
  }, [controlSignals.sendJointCommand, eStopActive])

  // Handle discrete commands from gestures
  const handleDiscreteCommand = useCallback((command: DiscreteCommand) => {
    console.log('Discrete command received:', command)

    switch (command.type) {
      case 'GRIPPER':
        const gripperValue = command.value === 'close' ? -90 : 0
        setArmAngles(prev => ({ ...prev, gripperRotation: gripperValue }))
        if (controlSignals.sendJointCommand) {
          const positionArray = jointAnglesToArray({ ...armAngles, gripperRotation: gripperValue })
          controlSignals.sendJointCommand(positionArray)
        }
        break

      case 'E_STOP':
        handleEStop()
        break

      case 'MODE':
        // Toggle between manual and mediapipe
        setInputSource(prev => prev === 'mediapipe' ? 'manual' : 'mediapipe')
        break
    }
  }, [armAngles, controlSignals.sendJointCommand])

  // Handle emergency stop
  const handleEStop = useCallback(() => {
    console.log('EMERGENCY STOP ACTIVATED')
    setEStopActive(true)
    setControlSignals(prev => ({ ...prev, systemStatus: 'E_STOP' }))

    // Reset E-stop after 3 seconds (in production this would require manual reset)
    setTimeout(() => {
      setEStopActive(false)
      setControlSignals(prev => ({ ...prev, systemStatus: 'READY' }))
      console.log('E-STOP reset')
    }, 3000)
  }, [])

  return (
    <div className="relative w-full h-screen bg-gradient-to-b from-gray-900 to-gray-800">
      <Canvas shadows dpr={[1, 2]}>
        <PerspectiveCamera makeDefault position={[8, 5, 12]} fov={45} />
        <color attach="background" args={["#111"]} />

        {/* Lighting setup */}
        <ambientLight intensity={0.2} />
        <spotLight
          position={[10, 15, 10]}
          angle={0.3}
          penumbra={0.7}
          intensity={1.5}
          castShadow
          shadow-mapSize-width={2048}
          shadow-mapSize-height={2048}
        />
        <directionalLight
          position={[-10, 10, -10]}
          intensity={0.7}
          castShadow
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />

        {/* Environment for reflections */}
        <Environment preset="warehouse" background={false} />

        <Physics
          gravity={[0, -9.8, 0]}
          defaultContactMaterial={{
            friction: 0.5,
            restitution: 0.2,
          }}
        >
          {/* Robot arm */}
          <RobotArm position={[0, 0, 0]} angles={armAngles} controlSignals={controlSignals} />

          {/* Floor with shadows */}
          <Floor />

          {/* Objects to interact with */}
          <InteractiveObjects />

          {/* KUKA branding */}
          <Text
            position={[0, 0.01, -4]}
            rotation={[-Math.PI / 2, 0, 0]}
            fontSize={0.5}
            color="#FF5F00"
            anchorX="center"
            anchorY="middle"
          >
            KUKA
          </Text>
        </Physics>

        <ContactShadows
          rotation-x={Math.PI / 2}
          position={[0, -0.49, 0]}
          opacity={0.5}
          width={30}
          height={30}
          blur={1}
          far={0.5}
        />

        <OrbitControls enableDamping dampingFactor={0.05} />
      </Canvas>

      {/* Control Panel - Manual controls */}
      <ControlPanel
        angles={armAngles}
        onAngleChange={handleAngleChange}
        onMovementSelect={executeMovement}
        controlSignals={controlSignals}
      />

      {/* Info Panel */}
      <InfoPanel />

      {/* Backend Connection */}
      <ZigController controlSignals={controlSignals} setControlSignals={setControlSignals} />

      {/* Hand Tracking Panel */}
      <HandTrackingPanel
        onJointAnglesChange={handleHandTrackingAngles}
        onDiscreteCommand={handleDiscreteCommand}
        onEStop={handleEStop}
        className="absolute top-4 right-4 w-[350px]"
      />

      {/* Input Source Indicator */}
      <div className="absolute top-4 left-1/2 -translate-x-1/2 bg-black/70 px-4 py-2 rounded-full">
        <span className="text-gray-400 text-sm">Input: </span>
        <span className={`text-sm font-medium ${
          inputSource === 'mediapipe' ? 'text-orange-400' :
          inputSource === 'preset' ? 'text-blue-400' :
          'text-green-400'
        }`}>
          {inputSource === 'mediapipe' ? 'Hand Tracking' :
           inputSource === 'preset' ? 'Preset' :
           'Manual'}
        </span>
        {eStopActive && (
          <span className="ml-2 text-red-500 font-bold animate-pulse">E-STOP</span>
        )}
      </div>
    </div>
  )
}

// Floor component
function Floor() {
  const [ref] = usePlane(() => ({
    rotation: [-Math.PI / 2, 0, 0],
    position: [0, -0.5, 0],
    type: "static",
  }))

  return (
    <mesh ref={ref} receiveShadow>
      <planeGeometry args={[30, 30]} />
      <meshStandardMaterial color="#333" roughness={0.8} metalness={0.2} />
    </mesh>
  )
}

// Interactive objects - much smaller relative to the large industrial arm
function InteractiveObjects() {
  return (
    <Suspense fallback={null}>
      {/* Cube objects that can be picked up */}
      <Box position={[2, 0, -2]} color="#3498db" size={[0.3, 0.3, 0.3]} />
      <Box position={[-2, 0, -2]} color="#2ecc71" size={[0.25, 0.25, 0.25]} />
      <Box position={[0, 0, -3]} color="#f1c40f" size={[0.35, 0.2, 0.2]} />

      {/* Sphere objects */}
      <Sphere position={[-1.5, 0, 0]} color="#e74c3c" radius={0.15} />
      <Sphere position={[1.5, 0, 0]} color="#9b59b6" radius={0.12} />

      {/* Cylinder object */}
      <Cylinder position={[0, 0, -1.5]} color="#1abc9c" />

      {/* Pallet with small parts */}
      <group position={[3, 0, 0]}>
        <mesh receiveShadow>
          <boxGeometry args={[1, 0.1, 1]} />
          <meshStandardMaterial color="#8B4513" roughness={0.9} metalness={0.1} />
        </mesh>

        {/* Small parts on pallet */}
        {Array.from({ length: 9 }).map((_, i) => {
          const x = ((i % 3) - 1) * 0.25
          const z = (Math.floor(i / 3) - 1) * 0.25
          return (
            <Box
              key={i}
              position={[3 + x, 0.15, z]}
              color={["#3498db", "#2ecc71", "#e74c3c", "#f1c40f", "#9b59b6"][i % 5]}
              size={[0.1, 0.1, 0.1]}
            />
          )
        })}
      </group>
    </Suspense>
  )
}

function Box({ position, color, size = [0.5, 0.5, 0.5] }: { position: [number, number, number], color: string, size?: [number, number, number] }) {
  const [ref] = useBox(() => ({
    mass: 1,
    position,
    args: size,
  }))

  return (
    <mesh ref={ref} castShadow>
      <boxGeometry args={size} />
      <meshStandardMaterial color={color} roughness={0.3} metalness={0.7} />
    </mesh>
  )
}

function Sphere({ position, color, radius = 0.5 }: { position: [number, number, number], color: string, radius?: number }) {
  const [ref] = useSphere(() => ({
    mass: 1,
    position,
    args: [radius],
  }))

  return (
    <mesh ref={ref} castShadow>
      <sphereGeometry args={[radius, 32, 32]} />
      <meshStandardMaterial color={color} roughness={0.2} metalness={0.6} />
    </mesh>
  )
}

function Cylinder({ position, color }: { position: [number, number, number], color: string }) {
  const [ref] = useBox(() => ({
    mass: 1,
    position,
    args: [0.2, 0.3, 0.2],
  }))

  return (
    <mesh ref={ref} castShadow>
      <cylinderGeometry args={[0.2, 0.2, 0.3, 32]} />
      <meshStandardMaterial color={color} roughness={0.4} metalness={0.6} />
    </mesh>
  )
}
