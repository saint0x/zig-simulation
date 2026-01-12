// MediaPipe Hand Tracking Types for KUKA Arm Control

// ===== MediaPipe Raw Data =====

export interface NormalizedLandmark {
  x: number  // 0.0 - 1.0 (normalized by image width)
  y: number  // 0.0 - 1.0 (normalized by image height)
  z: number  // Depth relative to wrist
}

export interface WorldLandmark {
  x: number  // meters
  y: number  // meters
  z: number  // meters
}

export interface HandResult {
  landmarks: NormalizedLandmark[]      // 21 points
  worldLandmarks: WorldLandmark[]      // 21 points in 3D
  handedness: 'Left' | 'Right'
  handednessScore: number              // 0.0 - 1.0
}

export interface GestureResult {
  categoryName: string                 // e.g., "Thumb_Up"
  score: number                        // 0.0 - 1.0
  index: number                        // Gesture index
}

// ===== Landmark Indices =====

export const LANDMARK = {
  WRIST: 0,
  THUMB_CMC: 1,
  THUMB_MCP: 2,
  THUMB_IP: 3,
  THUMB_TIP: 4,
  INDEX_MCP: 5,
  INDEX_PIP: 6,
  INDEX_DIP: 7,
  INDEX_TIP: 8,
  MIDDLE_MCP: 9,
  MIDDLE_PIP: 10,
  MIDDLE_DIP: 11,
  MIDDLE_TIP: 12,
  RING_MCP: 13,
  RING_PIP: 14,
  RING_DIP: 15,
  RING_TIP: 16,
  PINKY_MCP: 17,
  PINKY_PIP: 18,
  PINKY_DIP: 19,
  PINKY_TIP: 20,
} as const

// ===== Gesture Names =====

export const GESTURES = {
  NONE: 'None',
  CLOSED_FIST: 'Closed_Fist',
  OPEN_PALM: 'Open_Palm',
  POINTING_UP: 'Pointing_Up',
  THUMB_DOWN: 'Thumb_Down',
  THUMB_UP: 'Thumb_Up',
  VICTORY: 'Victory',
  I_LOVE_YOU: 'ILoveYou',
} as const

export type GestureName = typeof GESTURES[keyof typeof GESTURES]

// ===== Processed Hand Data =====

export interface ProcessedHandData {
  // Position (normalized 0-1)
  position: {
    x: number
    y: number
    z: number
  }

  // Orientation (radians)
  orientation: {
    roll: number   // Rotation around forward axis
    pitch: number  // Rotation around side axis
    yaw: number    // Rotation around up axis
  }

  // Finger states
  fingers: {
    thumb: FingerState
    index: FingerState
    middle: FingerState
    ring: FingerState
    pinky: FingerState
  }

  // Derived values
  pinchDistance: number    // 0.0 - 1.0 (thumb to index)
  grabStrength: number     // 0.0 - 1.0 (all fingers curl)

  // Metadata
  handedness: 'Left' | 'Right'
  confidence: number
  timestamp: number

  // Raw landmarks for visualization
  landmarks: NormalizedLandmark[]
}

export interface FingerState {
  extended: boolean
  curl: number       // 0.0 (straight) - 1.0 (fully curled)
  tipPosition: NormalizedLandmark
}

// ===== Joint Angles for Robot =====

export interface JointAngles {
  baseRotation: number       // -180 to +180 degrees
  shoulderRotation: number   // -120 to +30 degrees
  elbowRotation: number      // -120 to +120 degrees
  wristBendRotation: number  // -100 to +100 degrees
  wristRotation: number      // -180 to +180 degrees
  toolRotation: number       // -180 to +180 degrees
  gripperRotation: number    // 0 to 100 (percent open)
}

// ===== Control Commands =====

export interface JointCommand {
  type: 'POSITION'
  values: [number, number, number, number, number, number, number]
  maxVelocity?: number[]
  source: 'mediapipe' | 'manual' | 'preset'
  timestamp: number
}

export interface DiscreteCommand {
  type: 'GRIPPER' | 'MODE' | 'E_STOP' | 'CONFIRM' | 'CANCEL'
  value?: string | number
  source: 'mediapipe'
  gesture: string
  timestamp: number
}

// ===== Configuration =====

export interface MediaPipeConfig {
  numHands: 1 | 2
  minDetectionConfidence: number   // 0.0 - 1.0
  minTrackingConfidence: number    // 0.0 - 1.0

  // Custom settings
  mirrorMode: boolean
  preferredHand: 'left' | 'right' | 'any'
}

export interface ControlMappingConfig {
  mode: 'direct' | 'incremental' | 'target'

  // Sensitivity multiplier
  sensitivity: number  // 0.1 - 2.0

  // Dead zones
  positionDeadzone: number    // Normalized units (default 0.02)
  rotationDeadzone: number    // Radians (default 0.05)

  // Smoothing
  smoothingEnabled: boolean
  smoothingFactor: number     // 0.0 - 1.0

  // Rate limiting
  maxCommandRate: number      // Hz (default 60)

  // Gesture settings
  gestureConfidenceThreshold: number  // 0.0 - 1.0
  gestureHoldTime: number     // ms before action
}

// ===== Input Source =====

export type InputSource = 'manual' | 'mediapipe' | 'preset'

// ===== Hooks Return Types =====

export interface UseMediaPipeReturn {
  isInitialized: boolean
  isLoading: boolean
  error: Error | null
}

export interface UseWebcamReturn {
  videoRef: React.RefObject<HTMLVideoElement>
  isStreaming: boolean
  error: Error | null
  startStream: () => Promise<void>
  stopStream: () => void
}

export interface UseHandTrackingReturn {
  isTracking: boolean
  handData: ProcessedHandData | null
  gesture: GestureResult | null
  fps: number
  startTracking: () => void
  stopTracking: () => void
}

export interface UseGestureControlReturn {
  jointAngles: JointAngles | null
  discreteCommand: DiscreteCommand | null
  isControlling: boolean
  enable: () => void
  disable: () => void
  setMode: (mode: ControlMappingConfig['mode']) => void
  setSensitivity: (sensitivity: number) => void
}
