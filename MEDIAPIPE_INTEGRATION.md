# MediaPipe Hand Tracking Integration Plan

## Executive Summary

This document outlines the production-ready integration of MediaPipe hand tracking to control the KUKA arm simulation via webcam gesture input. The implementation adds a new input modality while preserving the existing architecture and WebSocket communication protocol.

## 1. Current Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        CURRENT ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐    WebSocket (9001)    ┌─────────────────────────┐ │
│  │   Backend   │◄────────────────────────│      Frontend          │ │
│  │    (Zig)    │     Binary/JSON         │   (Next.js/React)      │ │
│  │             │────────────────────────►│                        │ │
│  │  • 1kHz     │                         │  • 3D Visualization    │ │
│  │    Control  │  Joint States (1kHz)    │  • Control Panel UI    │ │
│  │  • Physics  │  System Status (10Hz)   │  • ZigController       │ │
│  │  • Safety   │  Collision Data         │  • Spring Animation    │ │
│  └─────────────┘                         └─────────────────────────┘ │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## 2. Proposed Architecture with MediaPipe

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                     ENHANCED ARCHITECTURE WITH MEDIAPIPE                      │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌─────────────┐                         ┌─────────────────────────────────┐ │
│  │   Backend   │    WebSocket (9001)     │          Frontend               │ │
│  │    (Zig)    │◄────────────────────────│                                 │ │
│  │             │     Binary/JSON         │  ┌───────────────────────────┐  │ │
│  │  • 1kHz     │────────────────────────►│  │   Input Sources           │  │ │
│  │    Control  │                         │  │                           │  │ │
│  │  • Physics  │  Joint States (1kHz)    │  │  ┌─────────────────────┐  │  │ │
│  │  • Safety   │  System Status (10Hz)   │  │  │ MediaPipe Tracker   │  │  │ │
│  │             │  Collision Data         │  │  │ • Webcam Input      │  │  │ │
│  │             │                         │  │  │ • Hand Landmarks    │  │  │ │
│  │             │                         │  │  │ • Gesture Detect    │  │  │ │
│  │             │                         │  │  └──────────┬──────────┘  │  │ │
│  │             │                         │  │             │             │  │ │
│  │             │                         │  │  ┌──────────▼──────────┐  │  │ │
│  │             │                         │  │  │ Input Processor     │  │  │ │
│  │             │◄─ Joint Commands ───────│  │  │ • Gesture Mapping   │  │  │ │
│  │             │                         │  │  │ • Smoothing/Filter  │  │  │ │
│  │             │                         │  │  │ • Safety Checks     │  │  │ │
│  │             │                         │  │  └──────────┬──────────┘  │  │ │
│  │             │                         │  │             │             │  │ │
│  │             │                         │  │  ┌──────────▼──────────┐  │  │ │
│  │             │                         │  │  │ Control Panel UI    │  │  │ │
│  │             │                         │  │  │ (existing + new)    │  │  │ │
│  │             │                         │  │  └─────────────────────┘  │  │ │
│  │             │                         │  └───────────────────────────┘  │ │
│  │             │                         │                                 │ │
│  │             │                         │  ┌───────────────────────────┐  │ │
│  │             │                         │  │   ZigController           │  │ │
│  │             │                         │  │   (existing - unchanged)  │  │ │
│  │             │                         │  └───────────────────────────┘  │ │
│  │             │                         │                                 │ │
│  │             │                         │  ┌───────────────────────────┐  │ │
│  │             │                         │  │   3D Visualization        │  │ │
│  │             │                         │  │   (existing - unchanged)  │  │ │
│  │             │                         │  └───────────────────────────┘  │ │
│  └─────────────┘                         └─────────────────────────────────┘ │
│                                                                               │
└──────────────────────────────────────────────────────────────────────────────┘
```

## 3. MediaPipe Technical Specifications

### 3.1 Package & Installation

```bash
npm install @mediapipe/tasks-vision
```

### 3.2 Hand Landmark Model (21 Points)

```
                    FINGERTIP (4,8,12,16,20)
                         │
                    DIP  (3,7,11,15,19)
                         │
                    PIP  (2,6,10,14,18)
                         │
                    MCP  (1,5,9,13,17)
                         │
                    ┌────┴────┐
                    │  WRIST  │ (0)
                    └─────────┘

Landmark Index Map:
├── WRIST: 0
├── THUMB:  CMC(1), MCP(2), IP(3), TIP(4)
├── INDEX:  MCP(5), PIP(6), DIP(7), TIP(8)
├── MIDDLE: MCP(9), PIP(10), DIP(11), TIP(12)
├── RING:   MCP(13), PIP(14), DIP(15), TIP(16)
└── PINKY:  MCP(17), PIP(18), DIP(19), TIP(20)
```

### 3.3 Built-in Gestures

| Gesture | ID | Robot Action |
|---------|-----|--------------|
| `None` | 0 | No action |
| `Closed_Fist` | 1 | Close gripper |
| `Open_Palm` | 2 | Open gripper / Stop |
| `Pointing_Up` | 3 | Move up (Z+) |
| `Thumb_Down` | 4 | Move down (Z-) |
| `Thumb_Up` | 5 | Confirm / Execute |
| `Victory` | 6 | Toggle control mode |
| `ILoveYou` | 7 | Emergency stop |

### 3.4 Performance Specs

| Metric | Value |
|--------|-------|
| Frame Rate | 30+ FPS |
| Detection Time (GPU) | ~5-8ms |
| Detection Time (CPU) | ~15-30ms |
| Max Hands | 2 |
| Landmark Precision | Sub-pixel |

## 4. Hand-to-Robot Mapping Strategy

### 4.1 Coordinate Mapping

```typescript
// MediaPipe coordinates (normalized 0-1) to Robot joint angles

interface HandToRobotMapping {
  // WRIST POSITION → Base rotation + Shoulder
  wristX: number;        // 0-1 → Base rotation (-180° to +180°)
  wristY: number;        // 0-1 → Shoulder rotation (-120° to +30°)

  // HAND DEPTH → Elbow extension
  wristZ: number;        // Depth → Elbow rotation (-120° to +120°)

  // HAND ROTATION → Wrist joints
  handRoll: number;      // Calculated from landmarks → Wrist bend
  handPitch: number;     // Calculated from landmarks → Wrist rotation
  handYaw: number;       // Calculated from landmarks → Tool rotation

  // FINGER PINCH → Gripper
  pinchDistance: number; // Thumb-Index distance → Gripper (0-100%)
}
```

### 4.2 Mapping Functions

```typescript
// Primary mapping: Hand position to robot position
function mapHandToRobot(landmarks: NormalizedLandmark[]): JointAngles {
  const wrist = landmarks[0];
  const thumbTip = landmarks[4];
  const indexTip = landmarks[8];
  const middleMCP = landmarks[9];

  // Calculate hand orientation from landmark geometry
  const handVector = {
    x: middleMCP.x - wrist.x,
    y: middleMCP.y - wrist.y,
    z: middleMCP.z - wrist.z
  };

  return {
    baseRotation: mapRange(wrist.x, 0, 1, -180, 180),
    shoulderRotation: mapRange(wrist.y, 0, 1, -120, 30),
    elbowRotation: mapRange(wrist.z, -0.3, 0.3, -120, 120),
    wristBendRotation: calculateHandPitch(landmarks) * 100,
    wristRotation: calculateHandRoll(landmarks) * 180,
    toolRotation: calculateHandYaw(landmarks) * 180,
    gripperRotation: mapPinchToGripper(thumbTip, indexTip)
  };
}

// Gesture-based discrete commands
function mapGestureToCommand(gesture: string): RobotCommand | null {
  switch(gesture) {
    case 'Closed_Fist': return { type: 'GRIPPER', value: 'CLOSE' };
    case 'Open_Palm': return { type: 'GRIPPER', value: 'OPEN' };
    case 'Pointing_Up': return { type: 'MOVE', axis: 'Z', direction: 1 };
    case 'Thumb_Down': return { type: 'MOVE', axis: 'Z', direction: -1 };
    case 'Thumb_Up': return { type: 'CONFIRM' };
    case 'Victory': return { type: 'MODE_TOGGLE' };
    case 'ILoveYou': return { type: 'E_STOP' };
    default: return null;
  }
}
```

### 4.3 Control Modes

| Mode | Hand Input | Description |
|------|-----------|-------------|
| **Direct Position** | Hand position → Joint angles | Real-time 1:1 mapping |
| **Incremental** | Gestures → Delta movements | Discrete step control |
| **Target** | Point & confirm | Set waypoint then execute |
| **Playback** | Record gesture sequence | Teach pendant mode |

## 5. New Component Architecture

### 5.1 Component Hierarchy

```
src/
├── components/
│   ├── mediapipe/
│   │   ├── hand-tracker.tsx         # MediaPipe initialization & webcam
│   │   ├── gesture-processor.tsx    # Gesture recognition logic
│   │   ├── hand-visualizer.tsx      # Debug overlay for hand landmarks
│   │   └── control-mapper.tsx       # Hand → Robot command mapping
│   │
│   ├── input/
│   │   ├── input-source-manager.tsx # Switch between input modes
│   │   ├── input-smoothing.tsx      # Kalman filter / low-pass
│   │   └── safety-guard.tsx         # Pre-send safety validation
│   │
│   ├── webcam/
│   │   ├── webcam-provider.tsx      # Camera access & stream
│   │   └── webcam-overlay.tsx       # Camera preview in UI
│   │
│   └── (existing components...)
│
├── hooks/
│   ├── useMediaPipe.ts              # MediaPipe lifecycle hook
│   ├── useHandTracking.ts           # Real-time hand data
│   ├── useGestureControl.ts         # Gesture → command conversion
│   └── useInputSmoothing.ts         # Signal smoothing utilities
│
├── lib/
│   ├── mediapipe/
│   │   ├── config.ts                # MediaPipe configuration
│   │   ├── landmarks.ts             # Landmark utilities
│   │   ├── gestures.ts              # Custom gesture definitions
│   │   └── mapping.ts               # Hand-to-robot math
│   │
│   └── filters/
│       ├── kalman.ts                # Kalman filter implementation
│       ├── low-pass.ts              # Low-pass filter
│       └── velocity-limit.ts        # Rate limiting
│
└── types/
    └── mediapipe.ts                 # TypeScript interfaces
```

### 5.2 Core Components

#### HandTracker Component

```tsx
// src/components/mediapipe/hand-tracker.tsx

interface HandTrackerProps {
  enabled: boolean;
  onLandmarksUpdate: (landmarks: HandLandmarks) => void;
  onGestureDetected: (gesture: GestureResult) => void;
  onError: (error: Error) => void;
}

interface HandLandmarks {
  landmarks: NormalizedLandmark[];  // 21 points per hand
  worldLandmarks: Landmark[];       // 3D world coordinates
  handedness: 'Left' | 'Right';
  confidence: number;
}

interface GestureResult {
  gesture: string;                  // One of 8 built-in gestures
  confidence: number;
  handedness: 'Left' | 'Right';
}
```

#### ControlMapper Component

```tsx
// src/components/mediapipe/control-mapper.tsx

interface ControlMapperProps {
  landmarks: HandLandmarks | null;
  gesture: GestureResult | null;
  mode: 'direct' | 'incremental' | 'target';
  sensitivity: number;              // 0.1 - 2.0
  deadzone: number;                 // 0.01 - 0.1
  onJointCommand: (angles: JointAngles) => void;
  onDiscreteCommand: (command: DiscreteCommand) => void;
}
```

#### InputSourceManager Component

```tsx
// src/components/input/input-source-manager.tsx

type InputSource = 'manual' | 'mediapipe' | 'preset' | 'recording';

interface InputSourceManagerProps {
  activeSource: InputSource;
  onSourceChange: (source: InputSource) => void;
  children: React.ReactNode;
}
```

## 6. Implementation Phases

### Phase 1: Core MediaPipe Integration (Week 1-2)

**Goals:** Get hand tracking working with visual feedback

| Task | Priority | Complexity |
|------|----------|------------|
| Install @mediapipe/tasks-vision | High | Low |
| Create useMediaPipe hook | High | Medium |
| Implement HandTracker component | High | Medium |
| Add webcam access with getUserMedia | High | Low |
| Create hand landmark visualizer (debug) | Medium | Low |
| Add webcam preview overlay | Medium | Low |
| Unit tests for MediaPipe initialization | Medium | Medium |

**Deliverables:**
- Webcam feed displayed in UI
- Hand landmarks rendered as overlay
- Console logs showing landmark data

### Phase 2: Gesture Recognition & Mapping (Week 2-3)

**Goals:** Convert hand data to robot commands

| Task | Priority | Complexity |
|------|----------|------------|
| Implement gesture detection pipeline | High | Medium |
| Create hand-to-robot coordinate mapping | High | High |
| Add Kalman filter for smoothing | High | Medium |
| Implement deadzone logic | Medium | Low |
| Create gesture-to-command mapping | High | Medium |
| Add control mode switching | Medium | Medium |
| Integration tests for mapping accuracy | High | Medium |

**Deliverables:**
- Gestures detected and logged
- Hand position mapped to joint angles
- Smooth, filtered output signal

### Phase 3: Robot Control Integration (Week 3-4)

**Goals:** Send commands to backend, see robot move

| Task | Priority | Complexity |
|------|----------|------------|
| Connect ControlMapper to ZigController | High | Medium |
| Implement rate limiting (max 100Hz commands) | High | Low |
| Add input source manager (switch modes) | High | Medium |
| Safety validation layer | High | Medium |
| E-stop gesture handling | Critical | Low |
| Visual feedback for active gesture | Medium | Low |
| End-to-end integration tests | High | High |

**Deliverables:**
- Robot arm follows hand movements
- Gestures trigger discrete actions
- E-stop immediately halts robot

### Phase 4: UI/UX & Polish (Week 4-5)

**Goals:** Production-ready user experience

| Task | Priority | Complexity |
|------|----------|------------|
| Design hand tracking UI panel | Medium | Medium |
| Add calibration flow | High | Medium |
| Implement sensitivity controls | Medium | Low |
| Add visual guides (workspace boundaries) | Medium | Medium |
| Create onboarding tutorial overlay | Low | Medium |
| Performance optimization (Web Workers) | High | High |
| Accessibility considerations | Medium | Medium |
| Final QA and bug fixes | High | Variable |

**Deliverables:**
- Polished UI for hand tracking mode
- Calibration wizard
- Adjustable sensitivity/deadzone
- 60fps performance maintained

### Phase 5: Advanced Features (Future)

| Feature | Description |
|---------|-------------|
| Two-hand control | Dual arm or arm + gripper split |
| Custom gesture training | TensorFlow.js custom classifier |
| Gesture macros | Record → replay gesture sequences |
| Mobile companion | QR code → phone as input device |
| Voice + gesture | Multi-modal control |

## 7. Data Flow Diagrams

### 7.1 Real-Time Control Flow

```
┌──────────────────────────────────────────────────────────────────────┐
│                     REAL-TIME CONTROL FLOW                           │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   Webcam (30fps)                                                     │
│       │                                                              │
│       ▼                                                              │
│   ┌─────────────────┐                                                │
│   │ MediaPipe       │ ~5-8ms GPU / ~15-30ms CPU                      │
│   │ GestureRecognizer│                                               │
│   └────────┬────────┘                                                │
│            │                                                         │
│            ├──────────────┬──────────────┐                           │
│            ▼              ▼              ▼                           │
│   ┌──────────────┐ ┌────────────┐ ┌─────────────┐                    │
│   │  Landmarks   │ │  Gesture   │ │  Handedness │                    │
│   │  (21 points) │ │  (string)  │ │  (L/R)      │                    │
│   └──────┬───────┘ └─────┬──────┘ └──────┬──────┘                    │
│          │               │               │                           │
│          ▼               ▼               ▼                           │
│   ┌─────────────────────────────────────────┐                        │
│   │         Gesture Processor               │                        │
│   │  • Coordinate extraction                │                        │
│   │  • Hand orientation calculation         │                        │
│   │  • Pinch distance measurement           │                        │
│   └──────────────────┬──────────────────────┘                        │
│                      │                                               │
│                      ▼                                               │
│   ┌─────────────────────────────────────────┐                        │
│   │           Input Smoothing               │                        │
│   │  • Kalman filter (position)             │                        │
│   │  • Low-pass filter (rotation)           │                        │
│   │  • Deadzone application                 │                        │
│   │  • Rate limiting (100Hz max)            │                        │
│   └──────────────────┬──────────────────────┘                        │
│                      │                                               │
│                      ▼                                               │
│   ┌─────────────────────────────────────────┐                        │
│   │           Control Mapper                │                        │
│   │  • Hand position → Joint angles         │                        │
│   │  • Gesture → Discrete command           │                        │
│   │  • Safety boundary checks               │                        │
│   └──────────────────┬──────────────────────┘                        │
│                      │                                               │
│                      ▼                                               │
│   ┌─────────────────────────────────────────┐                        │
│   │           Safety Guard                  │                        │
│   │  • Velocity limits                      │                        │
│   │  • Joint limit pre-check                │                        │
│   │  • E-stop gesture detection             │                        │
│   └──────────────────┬──────────────────────┘                        │
│                      │                                               │
│                      ▼                                               │
│   ┌─────────────────────────────────────────┐                        │
│   │        ZigController.sendJointCommand() │ (existing)             │
│   │  • JSON serialization                   │                        │
│   │  • WebSocket send                       │                        │
│   └──────────────────┬──────────────────────┘                        │
│                      │                                               │
│                      ▼                                               │
│   ┌─────────────────────────────────────────┐                        │
│   │           Zig Backend                   │                        │
│   │  • Command processing                   │                        │
│   │  • Physics simulation                   │                        │
│   │  • Joint state update                   │                        │
│   └──────────────────┬──────────────────────┘                        │
│                      │                                               │
│                      ▼                                               │
│              WebSocket response                                      │
│              (Joint states @ 1kHz)                                   │
│                      │                                               │
│                      ▼                                               │
│   ┌─────────────────────────────────────────┐                        │
│   │        3D Visualization                 │ (existing)             │
│   │  • Spring animation                     │                        │
│   │  • 60fps rendering                      │                        │
│   └─────────────────────────────────────────┘                        │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

### 7.2 State Management

```
┌────────────────────────────────────────────────────────────────┐
│                    STATE MANAGEMENT                            │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│   ┌─────────────────────────────────────────────────────────┐  │
│   │                   Global App State                       │  │
│   │                                                          │  │
│   │  inputSource: 'manual' | 'mediapipe' | 'preset'          │  │
│   │  mediapikeEnabled: boolean                               │  │
│   │  calibrationComplete: boolean                            │  │
│   │                                                          │  │
│   │  ┌─────────────────┐  ┌─────────────────┐               │  │
│   │  │ MediaPipe State │  │ Robot State     │               │  │
│   │  │                 │  │ (from backend)  │               │  │
│   │  │ landmarks: []   │  │                 │               │  │
│   │  │ gesture: string │  │ jointAngles: {} │               │  │
│   │  │ handedness: L/R │  │ systemStatus:{} │               │  │
│   │  │ confidence: num │  │ collision: {}   │               │  │
│   │  │ isTracking: bool│  │ connected: bool │               │  │
│   │  └─────────────────┘  └─────────────────┘               │  │
│   │                                                          │  │
│   │  ┌─────────────────────────────────────────────────────┐ │  │
│   │  │              Settings                                │ │  │
│   │  │                                                      │ │  │
│   │  │  handTrackingEnabled: boolean                        │ │  │
│   │  │  controlMode: 'direct' | 'incremental' | 'target'    │ │  │
│   │  │  sensitivity: number (0.1 - 2.0)                     │ │  │
│   │  │  deadzone: number (0.01 - 0.1)                       │ │  │
│   │  │  smoothingFactor: number (0.0 - 1.0)                 │ │  │
│   │  │  commandRate: number (10 - 100 Hz)                   │ │  │
│   │  │  mirrorMode: boolean                                 │ │  │
│   │  │  dominantHand: 'left' | 'right' | 'both'             │ │  │
│   │  └─────────────────────────────────────────────────────┘ │  │
│   └─────────────────────────────────────────────────────────┘  │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

## 8. API Contracts

### 8.1 TypeScript Interfaces

```typescript
// src/types/mediapipe.ts

// ===== MediaPipe Raw Data =====

export interface NormalizedLandmark {
  x: number;  // 0.0 - 1.0 (normalized by image width)
  y: number;  // 0.0 - 1.0 (normalized by image height)
  z: number;  // Depth relative to wrist
}

export interface WorldLandmark {
  x: number;  // meters
  y: number;  // meters
  z: number;  // meters
}

export interface HandResult {
  landmarks: NormalizedLandmark[];      // 21 points
  worldLandmarks: WorldLandmark[];      // 21 points in 3D
  handedness: 'Left' | 'Right';
  handednessScore: number;              // 0.0 - 1.0
}

export interface GestureResult {
  categoryName: string;                 // e.g., "Thumb_Up"
  score: number;                        // 0.0 - 1.0
  index: number;                        // Gesture index
}

// ===== Processed Hand Data =====

export interface ProcessedHandData {
  // Position (normalized 0-1)
  position: {
    x: number;
    y: number;
    z: number;
  };

  // Orientation (radians)
  orientation: {
    roll: number;   // Rotation around forward axis
    pitch: number;  // Rotation around side axis
    yaw: number;    // Rotation around up axis
  };

  // Finger states
  fingers: {
    thumb: FingerState;
    index: FingerState;
    middle: FingerState;
    ring: FingerState;
    pinky: FingerState;
  };

  // Derived values
  pinchDistance: number;    // 0.0 - 1.0 (thumb to index)
  grabStrength: number;     // 0.0 - 1.0 (all fingers curl)

  // Metadata
  handedness: 'Left' | 'Right';
  confidence: number;
  timestamp: number;
}

export interface FingerState {
  extended: boolean;
  curl: number;       // 0.0 (straight) - 1.0 (fully curled)
  tipPosition: NormalizedLandmark;
}

// ===== Control Commands =====

export interface JointCommand {
  type: 'POSITION';
  values: [number, number, number, number, number, number, number];
  maxVelocity?: number[];
  source: 'mediapipe' | 'manual' | 'preset';
  timestamp: number;
}

export interface DiscreteCommand {
  type: 'GRIPPER' | 'MODE' | 'E_STOP' | 'CONFIRM' | 'CANCEL';
  value?: string | number;
  source: 'mediapipe';
  gesture: string;
  timestamp: number;
}

// ===== Configuration =====

export interface MediaPipeConfig {
  numHands: 1 | 2;
  minDetectionConfidence: number;   // 0.0 - 1.0
  minTrackingConfidence: number;    // 0.0 - 1.0
  modelComplexity: 0 | 1;           // 0 = lite, 1 = full

  // Custom settings
  mirrorMode: boolean;
  preferredHand: 'left' | 'right' | 'any';
}

export interface ControlMappingConfig {
  mode: 'direct' | 'incremental' | 'target';

  // Sensitivity per joint
  sensitivity: {
    base: number;
    shoulder: number;
    elbow: number;
    wrist: number;
    gripper: number;
  };

  // Dead zones
  positionDeadzone: number;    // Normalized units
  rotationDeadzone: number;    // Radians

  // Smoothing
  smoothingEnabled: boolean;
  smoothingFactor: number;     // 0.0 - 1.0

  // Rate limiting
  maxCommandRate: number;      // Hz

  // Gesture settings
  gestureConfidenceThreshold: number;  // 0.0 - 1.0
  gestureHoldTime: number;     // ms before action
}

// ===== Hooks Return Types =====

export interface UseMediaPipeReturn {
  isInitialized: boolean;
  isLoading: boolean;
  error: Error | null;
  gestureRecognizer: GestureRecognizer | null;
}

export interface UseHandTrackingReturn {
  isTracking: boolean;
  handData: ProcessedHandData | null;
  gesture: GestureResult | null;
  fps: number;
  startTracking: () => void;
  stopTracking: () => void;
}

export interface UseGestureControlReturn {
  currentCommand: JointCommand | null;
  discreteCommand: DiscreteCommand | null;
  isControlling: boolean;
  enable: () => void;
  disable: () => void;
  setMode: (mode: ControlMappingConfig['mode']) => void;
  setSensitivity: (sensitivity: number) => void;
}
```

### 8.2 Hook Signatures

```typescript
// src/hooks/useMediaPipe.ts
export function useMediaPipe(config?: Partial<MediaPipeConfig>): UseMediaPipeReturn;

// src/hooks/useHandTracking.ts
export function useHandTracking(
  videoRef: RefObject<HTMLVideoElement>,
  options?: {
    enabled?: boolean;
    onLandmarksUpdate?: (data: ProcessedHandData) => void;
    onGestureDetected?: (gesture: GestureResult) => void;
  }
): UseHandTrackingReturn;

// src/hooks/useGestureControl.ts
export function useGestureControl(
  handData: ProcessedHandData | null,
  gesture: GestureResult | null,
  options?: Partial<ControlMappingConfig>
): UseGestureControlReturn;

// src/hooks/useInputSmoothing.ts
export function useInputSmoothing<T extends Record<string, number>>(
  input: T,
  options?: {
    method: 'kalman' | 'lowpass' | 'exponential';
    factor?: number;
  }
): T;
```

## 9. Safety Considerations

### 9.1 E-Stop Priority

```
┌────────────────────────────────────────────────────────────────┐
│                    SAFETY PRIORITY CHAIN                       │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│   1. [HIGHEST] Hardware E-Stop (Backend)                       │
│        │                                                       │
│   2.   └─► "ILoveYou" Gesture (MediaPipe)                      │
│              │                                                 │
│   3.         └─► UI E-Stop Button                              │
│                    │                                           │
│   4.               └─► Keyboard Shortcut (Escape)              │
│                          │                                     │
│   5.                     └─► Loss of hand tracking             │
│                                (configurable timeout)          │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 9.2 Safety Validation Layer

```typescript
// Safety checks before sending any command

interface SafetyValidation {
  // Pre-command checks
  checkJointLimits(angles: JointAngles): boolean;
  checkVelocityLimits(current: JointAngles, target: JointAngles, dt: number): boolean;
  checkAccelerationLimits(velocities: number[], dt: number): boolean;

  // Rate limiting
  throttleCommands(command: JointCommand, lastCommandTime: number): boolean;

  // Gesture safety
  validateGestureConfidence(gesture: GestureResult, threshold: number): boolean;
  requireGestureHold(gesture: string, holdTime: number): boolean;

  // Tracking loss handling
  handleTrackingLoss(lastTrackingTime: number, timeout: number): void;
}
```

### 9.3 Graceful Degradation

| Scenario | Response |
|----------|----------|
| Hand tracking lost < 500ms | Hold last position |
| Hand tracking lost > 500ms | Gradual slow to stop |
| Hand tracking lost > 2s | Full stop, await recovery |
| Low confidence (< 0.5) | Reduce command rate |
| Very low confidence (< 0.3) | Pause commands |
| Gesture conflict | Prioritize safety gestures |
| WebSocket disconnect | Immediate local stop |

## 10. Performance Optimization

### 10.1 Web Worker Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                  WEB WORKER ARCHITECTURE                      │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│   Main Thread                     MediaPipe Worker           │
│   ┌─────────────────┐            ┌─────────────────┐        │
│   │                 │            │                 │        │
│   │  React App      │            │  GestureRecognizer│       │
│   │  60fps render   │◄──────────│  30fps process  │        │
│   │                 │  landmarks │                 │        │
│   │  3D Scene       │  gestures  │  Heavy ML       │        │
│   │  UI Updates     │            │  inference      │        │
│   │                 │──────────►│                 │        │
│   │                 │  video     │                 │        │
│   │                 │  frames    │                 │        │
│   └─────────────────┘            └─────────────────┘        │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### 10.2 Optimization Strategies

| Strategy | Implementation | Impact |
|----------|---------------|--------|
| Web Workers | Offload MediaPipe to worker thread | Prevent UI jank |
| RequestAnimationFrame | Sync with display refresh | Smooth updates |
| Memoization | React.memo on visualizers | Reduce re-renders |
| Debounce gestures | 100ms gesture debounce | Prevent spam |
| Throttle commands | Max 100Hz command rate | Network efficiency |
| Canvas offscreen | OffscreenCanvas for video | GPU optimization |
| Model caching | Cache MediaPipe model | Faster load |

### 10.3 Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| UI Frame Rate | 60 fps | requestAnimationFrame timing |
| Hand Tracking Latency | < 50ms | Video frame to landmark |
| Command Latency | < 20ms | Landmark to WebSocket send |
| Total Pipeline | < 100ms | Hand move to robot move |
| Memory Usage | < 200MB | Performance.memory |
| CPU Usage | < 30% | Performance observer |

## 11. Testing Strategy

### 11.1 Unit Tests

```typescript
// __tests__/mediapipe/mapping.test.ts

describe('Hand-to-Robot Mapping', () => {
  test('maps center hand position to zero angles', () => {
    const landmarks = createMockLandmarks({ x: 0.5, y: 0.5, z: 0 });
    const angles = mapHandToRobot(landmarks);
    expect(angles.baseRotation).toBeCloseTo(0);
    expect(angles.shoulderRotation).toBeCloseTo(-45); // midpoint
  });

  test('maps left edge to negative base rotation', () => {
    const landmarks = createMockLandmarks({ x: 0, y: 0.5, z: 0 });
    const angles = mapHandToRobot(landmarks);
    expect(angles.baseRotation).toBeCloseTo(-180);
  });

  test('maps pinch gesture to gripper close', () => {
    const landmarks = createMockPinchLandmarks(0.02); // 2% distance
    const angles = mapHandToRobot(landmarks);
    expect(angles.gripperRotation).toBeCloseTo(0); // closed
  });
});

describe('Safety Validation', () => {
  test('rejects out-of-range joint angles', () => {
    const angles = { baseRotation: 200 }; // > 180 limit
    expect(validateJointLimits(angles)).toBe(false);
  });

  test('E-stop gesture has immediate effect', () => {
    const gesture = { categoryName: 'ILoveYou', score: 0.9 };
    const command = mapGestureToCommand(gesture);
    expect(command.type).toBe('E_STOP');
  });
});
```

### 11.2 Integration Tests

```typescript
// __tests__/integration/mediapipe-to-backend.test.ts

describe('MediaPipe to Backend Integration', () => {
  test('hand movement results in joint command sent', async () => {
    const { sendJointCommand } = renderWithProviders(<App />);

    // Simulate hand tracking data
    act(() => {
      simulateHandTracking({ x: 0.7, y: 0.3, z: 0.1 });
    });

    await waitFor(() => {
      expect(sendJointCommand).toHaveBeenCalledWith(
        expect.objectContaining({
          type: 'POSITION',
          source: 'mediapipe'
        })
      );
    });
  });
});
```

### 11.3 E2E Tests

```typescript
// e2e/hand-tracking.spec.ts

test('full hand tracking control flow', async ({ page }) => {
  await page.goto('/');

  // Enable hand tracking
  await page.click('[data-testid="enable-hand-tracking"]');

  // Grant camera permission (mock)
  await page.evaluate(() => mockCameraStream());

  // Verify tracking started
  await expect(page.locator('[data-testid="tracking-status"]'))
    .toHaveText('Tracking');

  // Simulate gesture
  await page.evaluate(() => injectMockGesture('Thumb_Up'));

  // Verify command indicator
  await expect(page.locator('[data-testid="last-gesture"]'))
    .toHaveText('Thumb_Up');
});
```

## 12. File Structure Summary

```
src/
├── components/
│   ├── mediapipe/
│   │   ├── hand-tracker.tsx           # Core tracking component
│   │   ├── gesture-processor.tsx      # Gesture logic
│   │   ├── hand-visualizer.tsx        # Debug overlay
│   │   ├── control-mapper.tsx         # Hand → commands
│   │   └── index.ts                   # Exports
│   │
│   ├── input/
│   │   ├── input-source-manager.tsx   # Input mode switching
│   │   ├── input-smoothing.tsx        # Signal filtering
│   │   ├── safety-guard.tsx           # Pre-send validation
│   │   └── index.ts
│   │
│   ├── webcam/
│   │   ├── webcam-provider.tsx        # Camera access
│   │   ├── webcam-overlay.tsx         # Preview UI
│   │   └── index.ts
│   │
│   └── ui/
│       ├── hand-tracking-panel.tsx    # Settings UI
│       ├── calibration-wizard.tsx     # Setup flow
│       └── gesture-indicator.tsx      # Current gesture display
│
├── hooks/
│   ├── useMediaPipe.ts                # MediaPipe lifecycle
│   ├── useHandTracking.ts             # Real-time tracking
│   ├── useGestureControl.ts           # Command generation
│   ├── useInputSmoothing.ts           # Filtering utilities
│   └── useWebcam.ts                   # Camera management
│
├── lib/
│   ├── mediapipe/
│   │   ├── config.ts                  # Configuration
│   │   ├── landmarks.ts               # Landmark utilities
│   │   ├── gestures.ts                # Gesture definitions
│   │   ├── mapping.ts                 # Coordinate math
│   │   └── worker.ts                  # Web Worker code
│   │
│   └── filters/
│       ├── kalman.ts                  # Kalman filter
│       ├── low-pass.ts                # Low-pass filter
│       └── velocity-limit.ts          # Rate limiting
│
├── types/
│   ├── mediapipe.ts                   # MediaPipe types
│   └── input.ts                       # Input system types
│
└── workers/
    └── mediapipe.worker.ts            # Offloaded processing
```

## 13. Dependencies to Add

```json
{
  "dependencies": {
    "@mediapipe/tasks-vision": "^0.10.0"
  },
  "devDependencies": {
    "@types/dom-mediacapture-record": "^1.0.0"
  }
}
```

## 14. Conclusion

This integration plan provides a production-ready path to add MediaPipe hand tracking control to the KUKA arm simulation. Key principles:

1. **Preserve existing architecture** - MediaPipe is an additional input source, not a replacement
2. **Backend remains authority** - All physics and state managed by Zig backend
3. **Safety first** - E-stop gestures, tracking loss handling, velocity limits
4. **Performance optimized** - Web Workers, throttling, efficient React patterns
5. **Extensible** - Easy to add custom gestures, two-hand control, mobile companion

The phased approach allows incremental delivery with working features at each milestone.

## Sources

- [MediaPipe Gesture Recognizer for Web](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/web_js)
- [MediaPipe Hand Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker/web_js)
- [MediaPipe Hands Documentation](https://mediapipe.readthedocs.io/en/latest/solutions/hands.html)
