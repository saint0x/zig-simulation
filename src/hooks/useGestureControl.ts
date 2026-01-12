// Gesture control hook - converts hand data to robot commands

import { useState, useRef, useCallback, useEffect } from 'react'
import type {
  ProcessedHandData,
  GestureResult,
  JointAngles,
  DiscreteCommand,
  ControlMappingConfig,
  UseGestureControlReturn,
} from '@/types/mediapipe'
import { GESTURES } from '@/types/mediapipe'
import { mapHandToJointAngles, mapGestureToCommand } from '@/lib/mediapipe/mapping'
import { DEFAULT_CONTROL_CONFIG } from '@/lib/mediapipe/config'
import { MultiLowPassFilter } from '@/lib/filters'

interface UseGestureControlOptions {
  config?: Partial<ControlMappingConfig>
  onJointAnglesChange?: (angles: JointAngles) => void
  onDiscreteCommand?: (command: DiscreteCommand) => void
  onEStop?: () => void
}

export function useGestureControl(
  handData: ProcessedHandData | null,
  gesture: GestureResult | null,
  options: UseGestureControlOptions = {}
): UseGestureControlReturn {
  const {
    config: initialConfig,
    onJointAnglesChange,
    onDiscreteCommand,
    onEStop,
  } = options

  const [jointAngles, setJointAngles] = useState<JointAngles | null>(null)
  const [discreteCommand, setDiscreteCommand] = useState<DiscreteCommand | null>(null)
  const [isControlling, setIsControlling] = useState(false)
  const [config, setConfig] = useState<ControlMappingConfig>({
    ...DEFAULT_CONTROL_CONFIG,
    ...initialConfig,
  })

  const filterRef = useRef(new MultiLowPassFilter(config.smoothingFactor))
  const lastCommandTimeRef = useRef(0)
  const lastGestureRef = useRef<string | null>(null)
  const gestureStartTimeRef = useRef(0)

  // Update filter when smoothing factor changes
  useEffect(() => {
    filterRef.current.setAlpha(config.smoothingFactor)
  }, [config.smoothingFactor])

  // Process hand data into joint angles
  useEffect(() => {
    if (!isControlling || !handData) {
      return
    }

    const now = Date.now()
    const minInterval = 1000 / config.maxCommandRate

    // Rate limiting
    if (now - lastCommandTimeRef.current < minInterval) {
      return
    }

    // Map hand to raw joint angles
    const rawAngles = mapHandToJointAngles(
      handData,
      config.sensitivity,
      config.positionDeadzone
    )

    // Apply smoothing if enabled
    let smoothedAngles: JointAngles
    if (config.smoothingEnabled) {
      const filtered = filterRef.current.updateAll({
        baseRotation: rawAngles.baseRotation,
        shoulderRotation: rawAngles.shoulderRotation,
        elbowRotation: rawAngles.elbowRotation,
        wristBendRotation: rawAngles.wristBendRotation,
        wristRotation: rawAngles.wristRotation,
        toolRotation: rawAngles.toolRotation,
        gripperRotation: rawAngles.gripperRotation,
      })

      smoothedAngles = {
        baseRotation: filtered.baseRotation,
        shoulderRotation: filtered.shoulderRotation,
        elbowRotation: filtered.elbowRotation,
        wristBendRotation: filtered.wristBendRotation,
        wristRotation: filtered.wristRotation,
        toolRotation: filtered.toolRotation,
        gripperRotation: filtered.gripperRotation,
      }
    } else {
      smoothedAngles = rawAngles
    }

    setJointAngles(smoothedAngles)
    onJointAnglesChange?.(smoothedAngles)
    lastCommandTimeRef.current = now
  }, [handData, isControlling, config, onJointAnglesChange])

  // Process gestures into discrete commands
  useEffect(() => {
    if (!isControlling || !gesture) {
      lastGestureRef.current = null
      setDiscreteCommand(null)
      return
    }

    // Check confidence threshold
    if (gesture.score < config.gestureConfidenceThreshold) {
      lastGestureRef.current = null
      return
    }

    // Skip "None" gestures
    if (gesture.categoryName === GESTURES.NONE) {
      lastGestureRef.current = null
      return
    }

    const now = Date.now()

    // Handle gesture hold time
    if (gesture.categoryName !== lastGestureRef.current) {
      lastGestureRef.current = gesture.categoryName
      gestureStartTimeRef.current = now
      return
    }

    // Check if gesture has been held long enough
    if (now - gestureStartTimeRef.current < config.gestureHoldTime) {
      return
    }

    // E-Stop has highest priority and is immediate
    if (gesture.categoryName === GESTURES.I_LOVE_YOU) {
      const command: DiscreteCommand = {
        type: 'E_STOP',
        source: 'mediapipe',
        gesture: gesture.categoryName,
        timestamp: now,
      }
      setDiscreteCommand(command)
      onDiscreteCommand?.(command)
      onEStop?.()
      // Reset to prevent repeated E-stops
      lastGestureRef.current = null
      return
    }

    // Map other gestures to commands
    const command = mapGestureToCommand(gesture)
    if (command) {
      setDiscreteCommand(command)
      onDiscreteCommand?.(command)
      // Reset gesture to require re-gesture for next command
      lastGestureRef.current = null
    }
  }, [gesture, isControlling, config, onDiscreteCommand, onEStop])

  const enable = useCallback(() => {
    setIsControlling(true)
    filterRef.current.reset()
    console.log('Gesture control enabled')
  }, [])

  const disable = useCallback(() => {
    setIsControlling(false)
    setJointAngles(null)
    setDiscreteCommand(null)
    console.log('Gesture control disabled')
  }, [])

  const setMode = useCallback((mode: ControlMappingConfig['mode']) => {
    setConfig(prev => ({ ...prev, mode }))
  }, [])

  const setSensitivity = useCallback((sensitivity: number) => {
    setConfig(prev => ({ ...prev, sensitivity: Math.max(0.1, Math.min(2.0, sensitivity)) }))
  }, [])

  return {
    jointAngles,
    discreteCommand,
    isControlling,
    enable,
    disable,
    setMode,
    setSensitivity,
  }
}
