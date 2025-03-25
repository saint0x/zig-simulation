"use client"

import { useState, useEffect, Suspense } from "react"
import { Canvas } from "@react-three/fiber"
import { OrbitControls, Environment, PerspectiveCamera, ContactShadows, Text } from "@react-three/drei"
import { RobotArm } from "./robot-arm"
import { ControlPanel } from "./control-panel"
import { Physics, useBox, usePlane, useSphere } from "@react-three/cannon"
import { InfoPanel } from "./info-panel"
import { ZigController } from "./zig-controller"

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
  })

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

  // Simulate Zig controller communication
  useEffect(() => {
    const interval = setInterval(() => {
      // Simulate motor current based on joint movement
      const newMotorCurrents = Object.values(armAngles).map((angle) => Math.abs(angle) * 0.1 + Math.random() * 0.5)

      // Simulate sensor readings
      const newSensorReadings = newMotorCurrents.map((current) => current + (Math.random() - 0.5) * 0.2)

      setControlSignals({
        motorCurrents: newMotorCurrents,
        sensorReadings: newSensorReadings,
        systemStatus: Math.random() > 0.98 ? "WARNING" : "READY",
        errorCode: Math.random() > 0.98 ? "TEMP_HIGH" : null,
      })
    }, 500)

    return () => clearInterval(interval)
  }, [armAngles])

  const handleAngleChange = (joint, value) => {
    setArmAngles((prev) => ({ ...prev, [joint]: value }))
  }

  const executeMovement = (movementName) => {
    if (movements[movementName]) {
      setArmAngles(movements[movementName])
    }
  }

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

      <ControlPanel
        angles={armAngles}
        onAngleChange={handleAngleChange}
        onMovementSelect={executeMovement}
        controlSignals={controlSignals}
      />

      <InfoPanel />

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

function Box({ position, color, size = [0.5, 0.5, 0.5] }) {
  const [ref, api] = useBox(() => ({
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

function Sphere({ position, color, radius = 0.5 }) {
  const [ref, api] = useSphere(() => ({
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

function Cylinder({ position, color }) {
  const [ref, api] = useBox(() => ({
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

