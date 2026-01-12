import { useState, useEffect, useCallback, Suspense } from "react"
import { Canvas } from "@react-three/fiber"
import { OrbitControls, Environment, PerspectiveCamera, ContactShadows, Text } from "@react-three/drei"
import { RobotArm } from "./robot-arm"
import { ControlPanel } from "./control-panel"
import { Physics, useBox, usePlane, useSphere } from "@react-three/cannon"
import { InfoPanel } from "./info-panel"
import { ZigController } from "./zig-controller"

// Joint angle interface matching the 7-DOF KUKA arm
interface ArmAngles {
  baseRotation: number
  shoulderRotation: number
  elbowRotation: number
  wristBendRotation: number
  wristRotation: number
  toolRotation: number
  gripperRotation: number
}

// Control signals received from backend
interface ControlSignals {
  // Joint state data from backend (degrees)
  jointPositions?: number[]
  jointVelocities?: number[]
  jointTorques?: number[]
  motorTemperatures?: number[]
  motorCurrents?: number[]
  timestamp?: number
  // System status
  systemStatus: string
  controlMode?: string
  emergencyStop?: boolean
  collisionDetected?: boolean
  errorCode: string | null
  // Legacy fields for compatibility
  sensorReadings: number[]
  // Function to send commands to backend
  sendJointCommand?: (positions: number[], maxVelocity?: number[]) => void
}

export default function RobotArmSimulation() {
  // Target angles set by user (degrees) - these are sent to backend
  const [targetAngles, setTargetAngles] = useState<ArmAngles>({
    baseRotation: 0,
    shoulderRotation: 0,
    elbowRotation: 0,
    wristBendRotation: 0,
    wristRotation: 0,
    toolRotation: 0,
    gripperRotation: 0,
  })

  // Actual angles from backend (degrees) - these drive the visualization
  const [actualAngles, setActualAngles] = useState<ArmAngles>({
    baseRotation: 0,
    shoulderRotation: 0,
    elbowRotation: 0,
    wristBendRotation: 0,
    wristRotation: 0,
    toolRotation: 0,
    gripperRotation: 0,
  })

  // Control signals from backend
  const [controlSignals, setControlSignals] = useState<ControlSignals>({
    motorCurrents: [0, 0, 0, 0, 0, 0, 0],
    sensorReadings: [0, 0, 0, 0, 0, 0, 0],
    systemStatus: "CONNECTING",
    errorCode: null,
  })

  // Track if we're connected to backend
  const [isBackendConnected, setIsBackendConnected] = useState(false)

  // Predefined movements (target positions in degrees)
  const movements: Record<string, ArmAngles> = {
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

  // Update actual angles when backend sends joint positions
  useEffect(() => {
    if (controlSignals.jointPositions && controlSignals.jointPositions.length >= 7) {
      setActualAngles({
        baseRotation: controlSignals.jointPositions[0],
        shoulderRotation: controlSignals.jointPositions[1],
        elbowRotation: controlSignals.jointPositions[2],
        wristBendRotation: controlSignals.jointPositions[3],
        wristRotation: controlSignals.jointPositions[4],
        toolRotation: controlSignals.jointPositions[5],
        gripperRotation: controlSignals.jointPositions[6],
      })
      setIsBackendConnected(true)
    }
  }, [controlSignals.jointPositions])

  // Track connection status
  useEffect(() => {
    if (controlSignals.systemStatus === "READY" || controlSignals.systemStatus === "BUSY") {
      setIsBackendConnected(true)
    } else if (controlSignals.systemStatus === "CONNECTING") {
      setIsBackendConnected(false)
    }
  }, [controlSignals.systemStatus])

  // Send command to backend when target angles change
  const sendCommandToBackend = useCallback((angles: ArmAngles) => {
    if (controlSignals.sendJointCommand) {
      const positions = [
        angles.baseRotation,
        angles.shoulderRotation,
        angles.elbowRotation,
        angles.wristBendRotation,
        angles.wristRotation,
        angles.toolRotation,
        angles.gripperRotation,
      ]
      // Send with default max velocity (backend will use limits)
      controlSignals.sendJointCommand(positions)
    }
  }, [controlSignals.sendJointCommand])

  // Handle user changing a joint angle via slider
  const handleAngleChange = useCallback((joint: keyof ArmAngles, value: number) => {
    setTargetAngles((prev) => {
      const newAngles = { ...prev, [joint]: value }
      // Send command to backend
      sendCommandToBackend(newAngles)
      return newAngles
    })
  }, [sendCommandToBackend])

  // Handle preset movement selection
  const executeMovement = useCallback((movementName: string) => {
    if (movements[movementName]) {
      const newAngles = movements[movementName]
      setTargetAngles(newAngles)
      // Send command to backend
      sendCommandToBackend(newAngles)
    }
  }, [movements, sendCommandToBackend])

  // Decide which angles to show: use actual backend angles if connected, otherwise target angles
  const displayAngles = isBackendConnected ? actualAngles : targetAngles

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
          {/* Robot arm - uses backend angles when connected */}
          <RobotArm position={[0, 0, 0]} angles={displayAngles} controlSignals={controlSignals} />

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

      {/* Control panel - shows target angles for user input */}
      <ControlPanel
        angles={targetAngles}
        onAngleChange={handleAngleChange}
        onMovementSelect={executeMovement}
        controlSignals={controlSignals}
      />

      <InfoPanel />

      {/* Zig backend controller - manages WebSocket connection */}
      <ZigController controlSignals={controlSignals} setControlSignals={setControlSignals} />
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
