"use client"

import { useRef, useEffect } from "react"
import { useFrame } from "@react-three/fiber"
import { Vector3, Color } from "three"
import { useSpring } from "@react-spring/three"
import { MetalShaderMaterial } from "./metal-shader-material"
import * as THREE from "three"

// Industrial KUKA-style robot arm with complex multi-jointed design
export function RobotArm({ position, angles, controlSignals }) {
  // References to the different parts of the robot
  const groupRef = useRef()
  const baseRef = useRef()
  const shoulderRef = useRef()
  const upperArmRef = useRef()
  const elbowRef = useRef()
  const forearmRef = useRef()
  const wristRotationRef = useRef()
  const wristBendRef = useRef()
  const toolRotationRef = useRef()
  const gripperRef = useRef()
  const leftClawRef = useRef()
  const rightClawRef = useRef()

  // KUKA colors
  const kukaOrange = new Color("#FF5F00")
  const kukaBlack = new Color("#1A1A1A")
  const kukaGray = new Color("#333333")
  const kukaLightGray = new Color("#555555")
  const kukaDarkGray = new Color("#222222")

  // Default angles for a complex articulated position
  const defaultAngles = {
    baseRotation: 30,
    shoulderRotation: -45,
    elbowRotation: -60,
    wristBendRotation: -30,
    wristRotation: 45,
    toolRotation: 15,
    gripperRotation: -30,
  }

  // Animation springs for smooth movement with initial values set to complex articulated position
  const {
    baseRotation,
    shoulderRotation,
    elbowRotation,
    wristBendRotation,
    wristRotation,
    toolRotation,
    gripperRotation,
  } = useSpring({
    baseRotation: (angles.baseRotation !== 0 ? angles.baseRotation : defaultAngles.baseRotation) * (Math.PI / 180),
    shoulderRotation:
      (angles.shoulderRotation !== 0 ? angles.shoulderRotation : defaultAngles.shoulderRotation) * (Math.PI / 180),
    elbowRotation: (angles.elbowRotation !== 0 ? angles.elbowRotation : defaultAngles.elbowRotation) * (Math.PI / 180),
    wristBendRotation:
      (angles.wristBendRotation !== 0 ? angles.wristBendRotation : defaultAngles.wristBendRotation) * (Math.PI / 180),
    wristRotation: (angles.wristRotation !== 0 ? angles.wristRotation : defaultAngles.wristRotation) * (Math.PI / 180),
    toolRotation: (angles.toolRotation !== 0 ? angles.toolRotation : defaultAngles.toolRotation) * (Math.PI / 180),
    gripperRotation:
      (angles.gripperRotation !== 0 ? angles.gripperRotation : defaultAngles.gripperRotation) * (Math.PI / 180),
    config: { mass: 5, tension: 170, friction: 50 },
  })

  // Use the ref and spring to update the rotation on each frame
  useFrame(() => {
    if (baseRef.current) baseRef.current.rotation.y = baseRotation.get()
    if (shoulderRef.current) shoulderRef.current.rotation.x = shoulderRotation.get()
    if (elbowRef.current) elbowRef.current.rotation.x = elbowRotation.get()
    if (wristBendRef.current) wristBendRef.current.rotation.x = wristBendRotation.get()
    if (wristRotationRef.current) wristRotationRef.current.rotation.z = wristRotation.get()
    if (toolRotationRef.current) toolRotationRef.current.rotation.y = toolRotation.get()

    if (leftClawRef.current && rightClawRef.current) {
      // Gripper opens and closes both sides
      const gripValue = gripperRotation.get() * 0.5
      leftClawRef.current.rotation.z = -gripValue
      rightClawRef.current.rotation.z = gripValue
    }
  })

  return (
    <group ref={groupRef} position={position} scale={[1.5, 1.5, 1.5]}>
      {/* Base - Heavy industrial mounting platform */}
      <group ref={baseRef} position={[0, 0, 0]}>
        {/* Base plate - heavy industrial mounting */}
        <mesh position={[0, -0.3, 0]} receiveShadow>
          <cylinderGeometry args={[1.2, 1.4, 0.15, 32]} />
          <MetalShaderMaterial color={kukaBlack} roughness={0.5} metalness={0.7} scratches={0.4} />
        </mesh>

        {/* Base reinforcement ribs */}
        {[0, 1, 2, 3, 4, 5].map((i) => (
          <mesh key={`base-rib-${i}`} position={[0, -0.22, 0]} rotation={[0, (i * Math.PI) / 3, 0]} castShadow>
            <boxGeometry args={[1.0, 0.08, 0.15]} />
            <MetalShaderMaterial color={kukaBlack} roughness={0.5} metalness={0.7} scratches={0.3} />
          </mesh>
        ))}

        {/* Main base cylinder - heavy with cooling fins */}
        <mesh castShadow receiveShadow position={[0, 0, 0]}>
          <cylinderGeometry args={[0.9, 1.0, 0.5, 32]} />
          <MetalShaderMaterial color={kukaBlack} roughness={0.4} metalness={0.8} scratches={0.5} />
        </mesh>

        {/* Cooling fins around base */}
        {Array.from({ length: 16 }).map((_, i) => (
          <mesh key={`fin-${i}`} position={[0, 0, 0]} rotation={[0, (i * Math.PI) / 8, 0]} castShadow>
            <boxGeometry args={[1.1, 0.4, 0.05]} />
            <MetalShaderMaterial color={kukaBlack} roughness={0.6} metalness={0.7} scratches={0.3} />
          </mesh>
        ))}

        {/* Base top cover with details */}
        <mesh position={[0, 0.26, 0]} castShadow receiveShadow>
          <cylinderGeometry args={[0.8, 0.9, 0.05, 32]} />
          <MetalShaderMaterial color={kukaGray} roughness={0.5} metalness={0.7} scratches={0.4} />
        </mesh>

        {/* KUKA logo on base */}
        <mesh position={[0, 0.29, 0.7]} rotation={[0, 0, 0]}>
          <planeGeometry args={[0.4, 0.1]} />
          <meshStandardMaterial color="white" />
        </mesh>

        {/* Main rotation joint - thick industrial */}
        <mesh position={[0, 0.35, 0]} castShadow receiveShadow>
          <cylinderGeometry args={[0.7, 0.7, 0.15, 32]} />
          <MetalShaderMaterial color={kukaOrange} roughness={0.3} metalness={0.8} scratches={0.6} />
        </mesh>

        {/* Joint bolts - evenly spaced around rotation joint */}
        {Array.from({ length: 12 }).map((_, i) => {
          const angle = (i * Math.PI) / 6
          const radius = 0.6
          return (
            <mesh
              key={`joint-bolt-${i}`}
              position={[Math.sin(angle) * radius, 0.35, Math.cos(angle) * radius]}
              castShadow
            >
              <cylinderGeometry args={[0.05, 0.05, 0.2, 8]} />
              <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.9} scratches={0.2} />
            </mesh>
          )
        })}

        {/* Vertical column - main support structure */}
        <mesh position={[0, 1.2, 0]} castShadow receiveShadow>
          <cylinderGeometry args={[0.4, 0.5, 1.5, 32]} />
          <MetalShaderMaterial color={kukaOrange} roughness={0.4} metalness={0.85} scratches={0.7} />
        </mesh>

        {/* Column reinforcement panels */}
        {[0, 1, 2, 3].map((i) => (
          <mesh key={`column-panel-${i}`} position={[0, 1.0, 0]} rotation={[0, (i * Math.PI) / 2, 0]} castShadow>
            <boxGeometry args={[0.1, 1.3, 0.8]} />
            <MetalShaderMaterial color={kukaBlack} roughness={0.5} metalness={0.7} scratches={0.4} />
          </mesh>
        ))}

        {/* Column detail rings */}
        {[0.4, 0.9, 1.4].map((y, i) => (
          <mesh key={`column-ring-${i}`} position={[0, y, 0]} castShadow receiveShadow>
            <cylinderGeometry args={[0.52, 0.52, 0.05, 32]} />
            <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.9} scratches={0.4} />
          </mesh>
        ))}

        {/* Shoulder joint - massive industrial pivot at top of column */}
        <group position={[0, 2.0, 0]}>
          {/* Main shoulder housing */}
          <mesh castShadow receiveShadow>
            <cylinderGeometry args={[0.45, 0.45, 0.4, 32]} rotation={[Math.PI / 2, 0, 0]} />
            <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.9} scratches={0.6} />
          </mesh>

          {/* Shoulder joint orange accent rings */}
          {[-1, 1].map((side) => (
            <mesh key={`shoulder-ring-${side}`} castShadow receiveShadow position={[0, 0, side * 0.15]}>
              <cylinderGeometry args={[0.48, 0.48, 0.05, 32]} rotation={[Math.PI / 2, 0, 0]} />
              <MetalShaderMaterial color={kukaOrange} roughness={0.3} metalness={0.85} scratches={0.5} />
            </mesh>
          ))}

          {/* Shoulder joint bolts */}
          {Array.from({ length: 8 }).map((_, i) => {
            const angle = (i * Math.PI) / 4
            return (
              <mesh
                key={`shoulder-bolt-${i}`}
                position={[Math.cos(angle) * 0.35, Math.sin(angle) * 0.35, 0]}
                castShadow
              >
                <cylinderGeometry args={[0.04, 0.04, 0.45, 8]} rotation={[Math.PI / 2, 0, 0]} />
                <MetalShaderMaterial color={kukaGray} roughness={0.3} metalness={0.9} scratches={0.2} />
              </mesh>
            )
          })}

          {/* Shoulder to upperarm connection - the rotating part */}
          <group ref={shoulderRef} position={[0, 0, 0]}>
            {/* Upper arm mount - connects to the shoulder */}
            <mesh castShadow receiveShadow position={[0, 0, 0.3]}>
              <boxGeometry args={[0.6, 0.4, 0.3]} />
              <MetalShaderMaterial color={kukaOrange} roughness={0.4} metalness={0.85} scratches={0.6} />
            </mesh>

            {/* Hydraulic cylinders on shoulder */}
            {[-1, 1].map((side) => (
              <group key={`shoulder-hydraulic-${side}`} position={[side * 0.4, 0, 0.15]} rotation={[0, 0, Math.PI / 2]}>
                <mesh castShadow>
                  <cylinderGeometry args={[0.08, 0.08, 0.5, 16]} />
                  <MetalShaderMaterial color={kukaGray} roughness={0.3} metalness={0.9} scratches={0.5} />
                </mesh>
                <mesh castShadow position={[0, 0.3, 0]}>
                  <cylinderGeometry args={[0.05, 0.05, 0.2, 16]} />
                  <MetalShaderMaterial color={kukaLightGray} roughness={0.2} metalness={0.95} scratches={0.3} />
                </mesh>
              </group>
            ))}

            {/* Upper arm - KUKA style with thick industrial design */}
            <group ref={upperArmRef}>
              {/* Main upper arm body - thick and robust */}
              <mesh castShadow receiveShadow position={[0, 0, 0.9]}>
                <boxGeometry args={[0.5, 0.35, 1.0]} />
                <MetalShaderMaterial color={kukaOrange} roughness={0.4} metalness={0.85} scratches={0.7} />
              </mesh>

              {/* Upper arm reinforcement ribs */}
              {[-1, 1].map((side) => (
                <mesh key={`arm-rib-${side}`} castShadow receiveShadow position={[side * 0.26, 0, 0.9]}>
                  <boxGeometry args={[0.05, 0.4, 1.05]} />
                  <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.8} scratches={0.5} />
                </mesh>
              ))}

              {/* Upper arm detail panels */}
              <mesh castShadow receiveShadow position={[0, 0.18, 0.9]}>
                <boxGeometry args={[0.45, 0.05, 0.95]} />
                <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.8} scratches={0.4} />
              </mesh>

              {/* Cable routing along upper arm */}
              <mesh castShadow position={[0.25, 0.15, 0.9]}>
                <tubeGeometry
                  args={[
                    new THREE.CatmullRomCurve3([
                      new Vector3(0, -0.5, 0),
                      new Vector3(0.05, -0.3, 0.1),
                      new Vector3(0, 0, 0),
                      new Vector3(0.05, 0.3, 0.1),
                      new Vector3(0, 0.5, 0),
                    ]),
                    64,
                    0.03,
                    8,
                    false,
                  ]}
                />
                <MetalShaderMaterial color={kukaBlack} roughness={0.7} metalness={0.3} scratches={0.2} />
              </mesh>

              {/* Elbow joint - massive industrial pivot */}
              <group position={[0, 0, 1.4]}>
                {/* Main elbow housing */}
                <mesh castShadow receiveShadow>
                  <cylinderGeometry args={[0.35, 0.35, 0.4, 24]} rotation={[Math.PI / 2, 0, 0]} />
                  <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.9} scratches={0.6} />
                </mesh>

                {/* Elbow joint orange accent rings */}
                {[-1, 1].map((side) => (
                  <mesh key={`elbow-ring-${side}`} castShadow receiveShadow position={[0, 0, side * 0.15]}>
                    <cylinderGeometry args={[0.38, 0.38, 0.05, 24]} rotation={[Math.PI / 2, 0, 0]} />
                    <MetalShaderMaterial color={kukaOrange} roughness={0.3} metalness={0.85} scratches={0.5} />
                  </mesh>
                ))}

                {/* Elbow joint bolts */}
                {Array.from({ length: 8 }).map((_, i) => {
                  const angle = (i * Math.PI) / 4
                  return (
                    <mesh
                      key={`elbow-bolt-${i}`}
                      position={[Math.cos(angle) * 0.3, Math.sin(angle) * 0.3, 0]}
                      castShadow
                    >
                      <cylinderGeometry args={[0.04, 0.04, 0.45, 8]} rotation={[Math.PI / 2, 0, 0]} />
                      <MetalShaderMaterial color={kukaGray} roughness={0.3} metalness={0.9} scratches={0.2} />
                    </mesh>
                  )
                })}

                {/* Elbow to forearm connection */}
                <group ref={elbowRef} position={[0, 0, 0]}>
                  {/* Forearm - thick industrial design */}
                  <group ref={forearmRef}>
                    {/* Main forearm body - thick and robust */}
                    <mesh castShadow receiveShadow position={[0, 0, 0.7]}>
                      <boxGeometry args={[0.45, 0.35, 0.9]} />
                      <MetalShaderMaterial color={kukaOrange} roughness={0.4} metalness={0.85} scratches={0.7} />
                    </mesh>

                    {/* Forearm reinforcement structures */}
                    {[-1, 1].map((side) => (
                      <mesh key={`forearm-reinforce-${side}`} castShadow receiveShadow position={[side * 0.23, 0, 0.7]}>
                        <boxGeometry args={[0.04, 0.4, 0.95]} />
                        <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.8} scratches={0.5} />
                      </mesh>
                    ))}

                    {/* Forearm detail panels and vents */}
                    <mesh castShadow receiveShadow position={[0, 0.18, 0.7]}>
                      <boxGeometry args={[0.4, 0.04, 0.85]} />
                      <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.8} scratches={0.4} />
                    </mesh>

                    {/* Hydraulic lines along forearm */}
                    {[-0.15, 0, 0.15].map((offset, i) => (
                      <mesh key={`hydraulic-line-${i}`} castShadow position={[offset, 0.18, 0.7]}>
                        <tubeGeometry
                          args={[
                            new THREE.CatmullRomCurve3([
                              new Vector3(0, 0, -0.45),
                              new Vector3(0.02, 0, -0.2),
                              new Vector3(-0.02, 0, 0),
                              new Vector3(0.02, 0, 0.2),
                              new Vector3(0, 0, 0.45),
                            ]),
                            64,
                            0.02,
                            8,
                            false,
                          ]}
                        />
                        <MetalShaderMaterial
                          color={i === 1 ? kukaGray : kukaBlack}
                          roughness={0.6}
                          metalness={0.4}
                          scratches={0.3}
                        />
                      </mesh>
                    ))}

                    {/* Wrist complex - multiple joints for more DOF */}
                    <group position={[0, 0, 1.2]}>
                      {/* Wrist bend joint - massive industrial */}
                      <mesh castShadow receiveShadow>
                        <cylinderGeometry args={[0.28, 0.28, 0.35, 24]} rotation={[Math.PI / 2, 0, 0]} />
                        <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.9} scratches={0.6} />
                      </mesh>

                      {/* Wrist joint orange accent rings */}
                      {[-1, 1].map((side) => (
                        <mesh key={`wrist-ring-${side}`} castShadow receiveShadow position={[0, 0, side * 0.12]}>
                          <cylinderGeometry args={[0.3, 0.3, 0.05, 24]} rotation={[Math.PI / 2, 0, 0]} />
                          <MetalShaderMaterial color={kukaOrange} roughness={0.3} metalness={0.85} scratches={0.5} />
                        </mesh>
                      ))}

                      {/* Wrist bend mechanism */}
                      <group ref={wristBendRef} position={[0, 0, 0]}>
                        {/* Wrist rotation joint - industrial design */}
                        <mesh castShadow receiveShadow position={[0, 0, 0.25]}>
                          <cylinderGeometry args={[0.22, 0.22, 0.3, 24]} />
                          <MetalShaderMaterial color={kukaOrange} roughness={0.3} metalness={0.85} scratches={0.6} />
                        </mesh>

                        {/* Wrist rotation detail rings */}
                        {[0.1, 0.25].map((pos, i) => (
                          <mesh key={`wrist-detail-${i}`} castShadow receiveShadow position={[0, 0, pos]}>
                            <cylinderGeometry args={[0.24, 0.24, 0.03, 24]} />
                            <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.9} scratches={0.4} />
                          </mesh>
                        ))}

                        {/* Wrist rotation mechanism */}
                        <group ref={wristRotationRef} position={[0, 0, 0.4]}>
                          {/* Tool rotation joint - industrial design */}
                          <mesh castShadow receiveShadow>
                            <cylinderGeometry args={[0.18, 0.18, 0.25, 24]} rotation={[Math.PI / 2, 0, 0]} />
                            <MetalShaderMaterial color={kukaBlack} roughness={0.3} metalness={0.9} scratches={0.5} />
                          </mesh>

                          {/* Tool rotation mechanism */}
                          <group ref={toolRotationRef} position={[0, 0, 0]}>
                            {/* End effector mount - industrial flange */}
                            <mesh castShadow receiveShadow position={[0, 0, 0.15]}>
                              <cylinderGeometry args={[0.16, 0.16, 0.12, 24]} />
                              <MetalShaderMaterial
                                color={kukaOrange}
                                roughness={0.3}
                                metalness={0.85}
                                scratches={0.5}
                              />
                            </mesh>

                            {/* Mounting plate with bolt pattern */}
                            <mesh castShadow receiveShadow position={[0, 0, 0.22]}>
                              <cylinderGeometry args={[0.18, 0.18, 0.05, 24]} />
                              <MetalShaderMaterial color={kukaGray} roughness={0.4} metalness={0.8} scratches={0.4} />
                            </mesh>

                            {/* Mounting bolts */}
                            {Array.from({ length: 6 }).map((_, i) => {
                              const angle = (i * Math.PI) / 3
                              const radius = 0.13
                              return (
                                <mesh
                                  key={`mount-bolt-${i}`}
                                  position={[Math.sin(angle) * radius, Math.cos(angle) * radius, 0.24]}
                                  castShadow
                                >
                                  <cylinderGeometry args={[0.02, 0.02, 0.08, 8]} />
                                  <MetalShaderMaterial
                                    color={kukaBlack}
                                    roughness={0.3}
                                    metalness={0.9}
                                    scratches={0.2}
                                  />
                                </mesh>
                              )
                            })}

                            {/* Industrial gripper - heavy duty */}
                            <group ref={gripperRef} position={[0, 0, 0.3]}>
                              {/* Gripper base - industrial design */}
                              <mesh castShadow receiveShadow>
                                <boxGeometry args={[0.35, 0.22, 0.18]} />
                                <MetalShaderMaterial color={kukaGray} roughness={0.4} metalness={0.8} scratches={0.6} />
                              </mesh>

                              {/* Gripper mechanism details */}
                              <mesh castShadow receiveShadow position={[0, 0, 0.09]}>
                                <boxGeometry args={[0.3, 0.18, 0.05]} />
                                <MetalShaderMaterial
                                  color={kukaBlack}
                                  roughness={0.3}
                                  metalness={0.9}
                                  scratches={0.4}
                                />
                              </mesh>

                              {/* Pneumatic cylinders for gripper */}
                              {[-1, 1].map((side) => (
                                <mesh
                                  key={`gripper-cylinder-${side}`}
                                  castShadow
                                  position={[side * 0.13, 0, 0.05]}
                                  rotation={[Math.PI / 2, 0, 0]}
                                >
                                  <cylinderGeometry args={[0.04, 0.04, 0.18, 12]} />
                                  <MetalShaderMaterial
                                    color={kukaGray}
                                    roughness={0.3}
                                    metalness={0.9}
                                    scratches={0.3}
                                  />
                                </mesh>
                              ))}

                              {/* Left claw - industrial gripper */}
                              <group ref={leftClawRef} position={[-0.18, 0, 0.09]}>
                                {/* Main claw arm */}
                                <mesh castShadow receiveShadow position={[-0.12, 0, 0.12]}>
                                  <boxGeometry args={[0.12, 0.16, 0.35]} />
                                  <MetalShaderMaterial
                                    color={kukaGray}
                                    roughness={0.4}
                                    metalness={0.85}
                                    scratches={0.6}
                                  />
                                </mesh>

                                {/* Claw finger */}
                                <mesh castShadow receiveShadow position={[-0.12, 0, 0.35]}>
                                  <boxGeometry args={[0.1, 0.13, 0.12]} />
                                  <MetalShaderMaterial
                                    color={kukaBlack}
                                    roughness={0.3}
                                    metalness={0.9}
                                    scratches={0.5}
                                  />
                                </mesh>

                                {/* Grip surface */}
                                <mesh castShadow receiveShadow position={[-0.07, 0, 0.35]}>
                                  <boxGeometry args={[0.02, 0.12, 0.11]} />
                                  <MetalShaderMaterial
                                    color={kukaDarkGray}
                                    roughness={0.7}
                                    metalness={0.5}
                                    scratches={0.8}
                                  />
                                </mesh>

                                {/* Claw details - hydraulic lines and bolts */}
                                <mesh castShadow position={[-0.18, 0.08, 0.18]}>
                                  <cylinderGeometry args={[0.015, 0.015, 0.25, 8]} rotation={[0, 0, Math.PI / 2]} />
                                  <MetalShaderMaterial
                                    color={kukaBlack}
                                    roughness={0.6}
                                    metalness={0.4}
                                    scratches={0.3}
                                  />
                                </mesh>
                              </group>

                              {/* Right claw - industrial gripper */}
                              <group ref={rightClawRef} position={[0.18, 0, 0.09]}>
                                {/* Main claw arm */}
                                <mesh castShadow receiveShadow position={[0.12, 0, 0.12]}>
                                  <boxGeometry args={[0.12, 0.16, 0.35]} />
                                  <MetalShaderMaterial
                                    color={kukaGray}
                                    roughness={0.4}
                                    metalness={0.85}
                                    scratches={0.6}
                                  />
                                </mesh>

                                {/* Claw finger */}
                                <mesh castShadow receiveShadow position={[0.12, 0, 0.35]}>
                                  <boxGeometry args={[0.1, 0.13, 0.12]} />
                                  <MetalShaderMaterial
                                    color={kukaBlack}
                                    roughness={0.3}
                                    metalness={0.9}
                                    scratches={0.5}
                                  />
                                </mesh>

                                {/* Grip surface */}
                                <mesh castShadow receiveShadow position={[0.07, 0, 0.35]}>
                                  <boxGeometry args={[0.02, 0.12, 0.11]} />
                                  <MetalShaderMaterial
                                    color={kukaDarkGray}
                                    roughness={0.7}
                                    metalness={0.5}
                                    scratches={0.8}
                                  />
                                </mesh>

                                {/* Claw details - hydraulic lines and bolts */}
                                <mesh castShadow position={[0.18, 0.08, 0.18]}>
                                  <cylinderGeometry args={[0.015, 0.015, 0.25, 8]} rotation={[0, 0, Math.PI / 2]} />
                                  <MetalShaderMaterial
                                    color={kukaBlack}
                                    roughness={0.6}
                                    metalness={0.4}
                                    scratches={0.3}
                                  />
                                </mesh>
                              </group>
                            </group>
                          </group>
                        </group>
                      </group>
                    </group>
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>

      {/* Visual guide for joint limits and cables */}
      <CableAndMotors
        baseRef={baseRef}
        shoulderRef={shoulderRef}
        elbowRef={elbowRef}
        wristBendRef={wristBendRef}
        wristRotationRef={wristRotationRef}
      />
    </group>
  )
}

// Update the CableAndMotors function to match the industrial style
function CableAndMotors({ baseRef, shoulderRef, elbowRef, wristBendRef, wristRotationRef }) {
  return (
    <>
      {/* Motor details on the base */}
      <group position={[0, 0.25, 0]} rotation={[0, Math.PI / 3, 0]}>
        <mesh castShadow position={[0.6, 0, 0]}>
          <cylinderGeometry args={[0.15, 0.15, 0.3, 16]} />
          <meshStandardMaterial color="#333" roughness={0.7} metalness={0.3} />
        </mesh>
        <mesh castShadow position={[0.6, 0.2, 0]}>
          <boxGeometry args={[0.3, 0.1, 0.2]} />
          <meshStandardMaterial color="#222" roughness={0.6} metalness={0.4} />
        </mesh>
      </group>

      {/* Cables running along the arm - thicker and more industrial */}
      <CableSegment
        start={new Vector3(0.3, 0.2, 0)}
        end={new Vector3(0.3, 1.8, 0)}
        parentRef={baseRef}
        color="#222"
        thickness={0.04}
      />

      <CableSegment
        start={new Vector3(0.15, 0.2, 0)}
        end={new Vector3(0.15, 1.2, 0)}
        parentRef={shoulderRef}
        color="#222"
        thickness={0.04}
      />

      <CableSegment
        start={new Vector3(-0.15, 0.1, 0)}
        end={new Vector3(-0.15, 0.8, 0)}
        parentRef={elbowRef}
        color="#222"
        thickness={0.04}
      />

      {/* Hydraulic lines */}
      <CableSegment
        start={new Vector3(0.2, 0.2, 0.1)}
        end={new Vector3(0.2, 1.5, 0.1)}
        parentRef={baseRef}
        color="#444"
        thickness={0.03}
      />

      <CableSegment
        start={new Vector3(-0.2, 0.2, 0.1)}
        end={new Vector3(-0.2, 1.5, 0.1)}
        parentRef={baseRef}
        color="#444"
        thickness={0.03}
      />
    </>
  )
}

// Helper component for cables
function CableSegment({ start, end, parentRef, color, thickness = 0.02 }) {
  const cableRef = useRef()

  useEffect(() => {
    if (parentRef.current && cableRef.current) {
      parentRef.current.add(cableRef.current)
      cableRef.current.position.copy(start)
    }

    return () => {
      if (parentRef.current && cableRef.current) {
        parentRef.current.remove(cableRef.current)
      }
    }
  }, [parentRef, start])

  const direction = new Vector3().subVectors(end, start)
  const length = direction.length()

  return (
    <group ref={cableRef}>
      <mesh castShadow>
        <cylinderGeometry args={[thickness, thickness, length, 8]} />
        <meshStandardMaterial color={color} roughness={0.8} />
      </mesh>
    </group>
  )
}

// Make sure to export the RobotArm component

