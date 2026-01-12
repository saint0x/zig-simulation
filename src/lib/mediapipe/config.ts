// MediaPipe Configuration

import type { MediaPipeConfig, ControlMappingConfig } from '@/types/mediapipe'

export const DEFAULT_MEDIAPIPE_CONFIG: MediaPipeConfig = {
  numHands: 1,
  minDetectionConfidence: 0.5,
  minTrackingConfidence: 0.5,
  mirrorMode: true,
  preferredHand: 'right',
}

export const DEFAULT_CONTROL_CONFIG: ControlMappingConfig = {
  mode: 'direct',
  sensitivity: 1.0,
  positionDeadzone: 0.02,
  rotationDeadzone: 0.05,
  smoothingEnabled: true,
  smoothingFactor: 0.3,
  maxCommandRate: 60,
  gestureConfidenceThreshold: 0.7,
  gestureHoldTime: 150,
}

// Joint limits matching backend (from main.zig)
export const JOINT_LIMITS = {
  baseRotation: { min: -180, max: 180 },
  shoulderRotation: { min: -120, max: 30 },
  elbowRotation: { min: -120, max: 120 },
  wristBendRotation: { min: -100, max: 100 },
  wristRotation: { min: -180, max: 180 },
  toolRotation: { min: -180, max: 180 },
  gripperRotation: { min: 0, max: 100 },
} as const

// MediaPipe model paths
export const MODEL_PATHS = {
  gestureRecognizer: 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task',
  wasmPath: 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@latest/wasm',
}
