# Frontend Architecture and Integration Plan

## Tech Stack
- **Build Tool**: Vite
- **Framework**: React 18
- **3D Rendering**: React Three Fiber, @react-three/drei, @react-three/cannon
- **Styling**: Tailwind CSS
- **UI Components**: Radix UI

## Current Frontend Components

### Core Components (`src/components/`)
- `robot-arm.tsx`: Main 3D model with 7-DOF joint hierarchy and spring animations
- `robot-arm-simulation.tsx`: Simulation environment, backend integration, command dispatching
- `zig-controller.tsx`: WebSocket client, binary/JSON parsing, command serialization
- `control-panel.tsx`: User interface with manual control, presets, and diagnostics
- `info-panel.tsx`: Static system information display

## Backend-Frontend Interface

### Data Flow FROM Backend

1. **Joint State Updates** (1kHz, Binary)
   ```typescript
   interface JointState {
     position: number[];      // Joint positions (degrees)
     velocity: number[];      // Joint velocities (deg/s)
     torque: number[];        // Joint torques (Nm)
     temperature: number[];   // Motor temperatures (°C)
     current: number[];       // Motor currents (A)
     timestamp: number;       // Timestamp in microseconds
   }
   ```

2. **System Status** (100Hz, JSON)
   ```typescript
   // Backend sends numeric values
   interface BackendSystemStatus {
     state: number;           // 0=READY, 1=BUSY, 2=ERROR, 3=WARNING
     error_code: string | null;
     safety_status: {
       soft_limits_active: boolean;
       emergency_stop: boolean;
       collision_detected: boolean;
     };
     control_mode: number;    // 0=POSITION, 1=VELOCITY, 2=TORQUE
   }

   // Frontend converts to string enum
   interface SystemStatus {
     state: 'READY' | 'BUSY' | 'ERROR' | 'WARNING';
     errorCode: string | null;
     safetyStatus: {
       softLimitsActive: boolean;
       emergencyStop: boolean;
       collisionDetected: boolean;
     };
     controlMode: 'POSITION' | 'VELOCITY' | 'TORQUE';
   }
   ```

3. **Collision Data** (100Hz when detected, JSON)
   ```typescript
   interface CollisionData {
     detected: boolean;
     link1: string;
     link2: string;
     position: [number, number, number];
     penetration_depth: number;
     contact_normal: [number, number, number];
   }
   ```

### Data Flow TO Backend

1. **Position Commands** (JSON)
   ```typescript
   interface PositionCommand {
     type: 0;                 // COMMAND_TYPE_POSITION
     values: number[];        // Target positions (degrees)
     max_velocity?: number[]; // Optional max velocities
   }
   ```

2. **Control Mode Changes** (JSON)
   ```typescript
   interface ControlModeCommand {
     type: 3;                 // COMMAND_TYPE_CONTROL_MODE
     control_mode: number;    // 0=POSITION, 1=VELOCITY, 2=TORQUE
     parameters?: {
       stiffness?: number;
       damping?: number;
       feedforward?: boolean;
     };
   }
   ```

3. **Safety Commands** (JSON)
   ```typescript
   interface SafetyCommand {
     type: 4;                 // COMMAND_TYPE_SAFETY
     safety: {
       type: number;         // 0=ENABLE, 1=DISABLE, 2=RESET, 3=E_STOP
       zone_id?: string;
     };
   }
   ```

## Implementation Status

### Completed Features
- [x] WebSocket connection with auto-reconnect
- [x] Binary protocol parsing for joint states
- [x] JSON parsing for system status with numeric->string conversion
- [x] Joint position command sending
- [x] Real-time 3D visualization with spring animations
- [x] Manual control sliders (7 joints)
- [x] Preset movement buttons
- [x] Diagnostics panel with temperature and current display
- [x] Connection status indicator
- [x] Emergency stop and collision detection alerts

### Future Enhancements
- [ ] Add motor temperature visualization (color gradient on joints)
- [ ] Implement torque visualization (arrows/force indicators)
- [ ] Add safety zone visualization
- [ ] Add trajectory preview visualization
- [ ] Real-time plotting of joint data
- [ ] Advanced trajectory planning interface
- [ ] Unit tests for protocol handling
- [ ] Performance benchmarking

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    RobotArmSimulation                       │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ State:                                                  ││
│  │  - targetAngles (user input)                           ││
│  │  - actualAngles (from backend)                         ││
│  │  - controlSignals (backend data + sendJointCommand)    ││
│  │  - isBackendConnected                                  ││
│  └─────────────────────────────────────────────────────────┘│
│                            │                                │
│         ┌──────────────────┼──────────────────┐            │
│         ▼                  ▼                  ▼            │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────┐    │
│  │  RobotArm   │  │ControlPanel  │  │ ZigController  │    │
│  │  (3D viz)   │  │ (UI inputs)  │  │ (WebSocket)    │    │
│  │             │  │              │  │                │    │
│  │ angles prop │  │ angles prop  │  │ setControl-    │    │
│  │ from parent │  │ onAngle-     │  │ Signals        │    │
│  │             │  │ Change()     │  │ callback       │    │
│  └─────────────┘  └──────────────┘  └────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **User Input Flow**:
   - User moves slider → `handleAngleChange()` → updates `targetAngles`
   - → calls `sendCommandToBackend()` → `sendJointCommand()` → WebSocket

2. **Backend Data Flow**:
   - WebSocket receives binary frame → `deserializeJointState()`
   - → updates `controlSignals.jointPositions`
   - → `useEffect` updates `actualAngles`
   - → `displayAngles` = `actualAngles` when connected

3. **Fallback Flow**:
   - When backend disconnected, `displayAngles` = `targetAngles`
   - Robot visualization continues showing last target positions

## Performance Requirements

1. **Rendering**: Maintain 60fps during all operations
2. **Update Rates**:
   - Backend data: 1kHz (throttled to 60Hz for rendering)
   - Status display: 10Hz refresh
   - Control input: Immediate on slider change
3. **Latency**:
   - Command to backend: <10ms (WebSocket send)
   - Backend data to visualization: <16ms (next frame)

## Development Notes

- All 3D visualizations use React Three Fiber
- TypeScript for type safety
- Tailwind CSS for styling
- Spring animations provide smooth joint movement
- React.memo and useCallback for performance optimization
- Connection status and error handling throughout
