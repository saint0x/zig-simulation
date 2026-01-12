// Hand-to-Robot Mapping Functions

import type {
  NormalizedLandmark,
  ProcessedHandData,
  JointAngles,
  GestureResult,
  DiscreteCommand,
  FingerState,
} from '@/types/mediapipe'
import { LANDMARK, GESTURES } from '@/types/mediapipe'
import { JOINT_LIMITS } from './config'

// Map a value from one range to another
export function mapRange(
  value: number,
  inMin: number,
  inMax: number,
  outMin: number,
  outMax: number
): number {
  const mapped = ((value - inMin) / (inMax - inMin)) * (outMax - outMin) + outMin
  return Math.max(outMin, Math.min(outMax, mapped))
}

// Apply deadzone to a value
export function applyDeadzone(value: number, deadzone: number, center: number = 0.5): number {
  const delta = value - center
  if (Math.abs(delta) < deadzone) {
    return center
  }
  // Remap remaining range to remove deadzone gap
  const sign = delta > 0 ? 1 : -1
  const remapped = (Math.abs(delta) - deadzone) / (0.5 - deadzone)
  return center + sign * remapped * 0.5
}

// Calculate 3D distance between two landmarks
export function distance3D(a: NormalizedLandmark, b: NormalizedLandmark): number {
  const dx = a.x - b.x
  const dy = a.y - b.y
  const dz = a.z - b.z
  return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

// Calculate 2D distance (ignoring z)
export function distance2D(a: NormalizedLandmark, b: NormalizedLandmark): number {
  const dx = a.x - b.x
  const dy = a.y - b.y
  return Math.sqrt(dx * dx + dy * dy)
}

// Calculate angle between three points (returns radians)
export function angleBetweenPoints(
  a: NormalizedLandmark,
  b: NormalizedLandmark,
  c: NormalizedLandmark
): number {
  const ba = { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }
  const bc = { x: c.x - b.x, y: c.y - b.y, z: c.z - b.z }

  const dot = ba.x * bc.x + ba.y * bc.y + ba.z * bc.z
  const magBA = Math.sqrt(ba.x * ba.x + ba.y * ba.y + ba.z * ba.z)
  const magBC = Math.sqrt(bc.x * bc.x + bc.y * bc.y + bc.z * bc.z)

  if (magBA === 0 || magBC === 0) return 0

  const cosAngle = Math.max(-1, Math.min(1, dot / (magBA * magBC)))
  return Math.acos(cosAngle)
}

// Calculate finger curl (0 = straight, 1 = fully curled)
export function calculateFingerCurl(
  landmarks: NormalizedLandmark[],
  mcpIndex: number,
  pipIndex: number,
  dipIndex: number,
  tipIndex: number
): number {
  const angle1 = angleBetweenPoints(
    landmarks[mcpIndex - 1] || landmarks[LANDMARK.WRIST],
    landmarks[mcpIndex],
    landmarks[pipIndex]
  )
  const angle2 = angleBetweenPoints(
    landmarks[mcpIndex],
    landmarks[pipIndex],
    landmarks[dipIndex]
  )
  const angle3 = angleBetweenPoints(
    landmarks[pipIndex],
    landmarks[dipIndex],
    landmarks[tipIndex]
  )

  // Normalize to 0-1 (straight finger ~= 3.14 rad total, curled ~= 1.5 rad)
  const totalAngle = angle1 + angle2 + angle3
  const curl = 1 - mapRange(totalAngle, 1.5, Math.PI, 0, 1)
  return Math.max(0, Math.min(1, curl))
}

// Calculate finger state
export function getFingerState(
  landmarks: NormalizedLandmark[],
  mcpIndex: number,
  pipIndex: number,
  dipIndex: number,
  tipIndex: number
): FingerState {
  const curl = calculateFingerCurl(landmarks, mcpIndex, pipIndex, dipIndex, tipIndex)
  return {
    extended: curl < 0.4,
    curl,
    tipPosition: landmarks[tipIndex],
  }
}

// Calculate hand roll (rotation around forward axis)
export function calculateHandRoll(landmarks: NormalizedLandmark[]): number {
  const wrist = landmarks[LANDMARK.WRIST]
  const indexMcp = landmarks[LANDMARK.INDEX_MCP]
  const pinkyMcp = landmarks[LANDMARK.PINKY_MCP]

  // Vector from pinky to index MCP
  const dx = indexMcp.x - pinkyMcp.x
  const dy = indexMcp.y - pinkyMcp.y

  // Calculate angle from horizontal
  return Math.atan2(dy, dx)
}

// Calculate hand pitch (rotation around side axis - tilt forward/back)
export function calculateHandPitch(landmarks: NormalizedLandmark[]): number {
  const wrist = landmarks[LANDMARK.WRIST]
  const middleMcp = landmarks[LANDMARK.MIDDLE_MCP]

  // Vector from wrist to middle MCP
  const dz = middleMcp.z - wrist.z
  const dy = middleMcp.y - wrist.y

  return Math.atan2(dz, -dy) // Negative y because y increases downward
}

// Calculate hand yaw (rotation around up axis - twist left/right)
export function calculateHandYaw(landmarks: NormalizedLandmark[]): number {
  const wrist = landmarks[LANDMARK.WRIST]
  const middleMcp = landmarks[LANDMARK.MIDDLE_MCP]

  // Vector from wrist to middle MCP in x-z plane
  const dx = middleMcp.x - wrist.x
  const dz = middleMcp.z - wrist.z

  return Math.atan2(dz, dx)
}

// Calculate pinch distance (normalized 0-1)
export function calculatePinchDistance(landmarks: NormalizedLandmark[]): number {
  const thumbTip = landmarks[LANDMARK.THUMB_TIP]
  const indexTip = landmarks[LANDMARK.INDEX_TIP]

  const dist = distance3D(thumbTip, indexTip)
  // Normalize: ~0.02 = pinched, ~0.15 = fully open
  return mapRange(dist, 0.02, 0.15, 0, 1)
}

// Calculate grab strength (average finger curl)
export function calculateGrabStrength(landmarks: NormalizedLandmark[]): number {
  const indexCurl = calculateFingerCurl(landmarks, LANDMARK.INDEX_MCP, LANDMARK.INDEX_PIP, LANDMARK.INDEX_DIP, LANDMARK.INDEX_TIP)
  const middleCurl = calculateFingerCurl(landmarks, LANDMARK.MIDDLE_MCP, LANDMARK.MIDDLE_PIP, LANDMARK.MIDDLE_DIP, LANDMARK.MIDDLE_TIP)
  const ringCurl = calculateFingerCurl(landmarks, LANDMARK.RING_MCP, LANDMARK.RING_PIP, LANDMARK.RING_DIP, LANDMARK.RING_TIP)
  const pinkyCurl = calculateFingerCurl(landmarks, LANDMARK.PINKY_MCP, LANDMARK.PINKY_PIP, LANDMARK.PINKY_DIP, LANDMARK.PINKY_TIP)

  return (indexCurl + middleCurl + ringCurl + pinkyCurl) / 4
}

// Process raw landmarks into structured hand data
export function processLandmarks(
  landmarks: NormalizedLandmark[],
  handedness: 'Left' | 'Right',
  confidence: number,
  mirror: boolean = true
): ProcessedHandData {
  // Apply mirror transformation if needed
  const processed = mirror
    ? landmarks.map(l => ({ ...l, x: 1 - l.x }))
    : landmarks

  const wrist = processed[LANDMARK.WRIST]

  return {
    position: {
      x: wrist.x,
      y: wrist.y,
      z: wrist.z,
    },
    orientation: {
      roll: calculateHandRoll(processed),
      pitch: calculateHandPitch(processed),
      yaw: calculateHandYaw(processed),
    },
    fingers: {
      thumb: {
        extended: calculateFingerCurl(processed, LANDMARK.THUMB_MCP, LANDMARK.THUMB_MCP, LANDMARK.THUMB_IP, LANDMARK.THUMB_TIP) < 0.4,
        curl: calculateFingerCurl(processed, LANDMARK.THUMB_MCP, LANDMARK.THUMB_MCP, LANDMARK.THUMB_IP, LANDMARK.THUMB_TIP),
        tipPosition: processed[LANDMARK.THUMB_TIP],
      },
      index: getFingerState(processed, LANDMARK.INDEX_MCP, LANDMARK.INDEX_PIP, LANDMARK.INDEX_DIP, LANDMARK.INDEX_TIP),
      middle: getFingerState(processed, LANDMARK.MIDDLE_MCP, LANDMARK.MIDDLE_PIP, LANDMARK.MIDDLE_DIP, LANDMARK.MIDDLE_TIP),
      ring: getFingerState(processed, LANDMARK.RING_MCP, LANDMARK.RING_PIP, LANDMARK.RING_DIP, LANDMARK.RING_TIP),
      pinky: getFingerState(processed, LANDMARK.PINKY_MCP, LANDMARK.PINKY_PIP, LANDMARK.PINKY_DIP, LANDMARK.PINKY_TIP),
    },
    pinchDistance: calculatePinchDistance(processed),
    grabStrength: calculateGrabStrength(processed),
    handedness: mirror ? (handedness === 'Left' ? 'Right' : 'Left') : handedness,
    confidence,
    timestamp: Date.now(),
    landmarks: processed,
  }
}

// Map processed hand data to robot joint angles
export function mapHandToJointAngles(
  hand: ProcessedHandData,
  sensitivity: number = 1.0,
  positionDeadzone: number = 0.02
): JointAngles {
  // Apply deadzones
  const x = applyDeadzone(hand.position.x, positionDeadzone)
  const y = applyDeadzone(hand.position.y, positionDeadzone)
  const z = hand.position.z // Z is relative, no deadzone needed

  // Map hand position to base joints
  const baseRotation = mapRange(x, 0, 1, JOINT_LIMITS.baseRotation.min, JOINT_LIMITS.baseRotation.max) * sensitivity
  const shoulderRotation = mapRange(y, 0, 1, JOINT_LIMITS.shoulderRotation.max, JOINT_LIMITS.shoulderRotation.min) * sensitivity // Inverted
  const elbowRotation = mapRange(z, -0.3, 0.3, JOINT_LIMITS.elbowRotation.min, JOINT_LIMITS.elbowRotation.max) * sensitivity

  // Map hand orientation to wrist joints
  const wristBendRotation = mapRange(
    hand.orientation.pitch,
    -Math.PI / 3,
    Math.PI / 3,
    JOINT_LIMITS.wristBendRotation.min,
    JOINT_LIMITS.wristBendRotation.max
  ) * sensitivity

  const wristRotation = mapRange(
    hand.orientation.roll,
    -Math.PI / 2,
    Math.PI / 2,
    JOINT_LIMITS.wristRotation.min,
    JOINT_LIMITS.wristRotation.max
  ) * sensitivity

  const toolRotation = mapRange(
    hand.orientation.yaw,
    -Math.PI / 2,
    Math.PI / 2,
    JOINT_LIMITS.toolRotation.min,
    JOINT_LIMITS.toolRotation.max
  ) * sensitivity

  // Map pinch to gripper (inverted: pinched = closed)
  const gripperRotation = mapRange(
    hand.pinchDistance,
    0,
    1,
    0,
    100
  )

  return {
    baseRotation: clampToLimits(baseRotation, 'baseRotation'),
    shoulderRotation: clampToLimits(shoulderRotation, 'shoulderRotation'),
    elbowRotation: clampToLimits(elbowRotation, 'elbowRotation'),
    wristBendRotation: clampToLimits(wristBendRotation, 'wristBendRotation'),
    wristRotation: clampToLimits(wristRotation, 'wristRotation'),
    toolRotation: clampToLimits(toolRotation, 'toolRotation'),
    gripperRotation: clampToLimits(gripperRotation, 'gripperRotation'),
  }
}

// Clamp value to joint limits
function clampToLimits(value: number, joint: keyof typeof JOINT_LIMITS): number {
  const limits = JOINT_LIMITS[joint]
  return Math.max(limits.min, Math.min(limits.max, value))
}

// Map gesture to discrete command
export function mapGestureToCommand(gesture: GestureResult): DiscreteCommand | null {
  if (gesture.score < 0.7) return null

  const timestamp = Date.now()
  const base = { source: 'mediapipe' as const, gesture: gesture.categoryName, timestamp }

  switch (gesture.categoryName) {
    case GESTURES.CLOSED_FIST:
      return { ...base, type: 'GRIPPER', value: 'close' }

    case GESTURES.OPEN_PALM:
      return { ...base, type: 'GRIPPER', value: 'open' }

    case GESTURES.THUMB_UP:
      return { ...base, type: 'CONFIRM' }

    case GESTURES.THUMB_DOWN:
      return { ...base, type: 'CANCEL' }

    case GESTURES.VICTORY:
      return { ...base, type: 'MODE' }

    case GESTURES.I_LOVE_YOU:
      return { ...base, type: 'E_STOP' }

    case GESTURES.POINTING_UP:
    case GESTURES.NONE:
    default:
      return null
  }
}

// Convert JointAngles to array format for backend
export function jointAnglesToArray(angles: JointAngles): number[] {
  return [
    angles.baseRotation,
    angles.shoulderRotation,
    angles.elbowRotation,
    angles.wristBendRotation,
    angles.wristRotation,
    angles.toolRotation,
    angles.gripperRotation,
  ]
}
