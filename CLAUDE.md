# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a KUKA robotic arm simulation project with a Zig backend for real-time control and physics simulation, and a Vite/React/Three.js frontend for 3D visualization and user interface.

## Commands

### Frontend Development
- **Development server**: `npm run dev`
- **Build**: `npm run build`
- **Preview production build**: `npm run preview`
- **Linting**: `npm run lint`

### Backend Development
- **Build**: `cd backend && zig build`
- **Run**: `cd backend && zig build run`
- **Run tests**: `cd backend && zig build test`
- **Clean**: `cd backend && rm -rf zig-cache zig-out`

## Architecture

### Backend (Zig)
The backend is structured as a modular system with clear dependency hierarchy:

**Core Modules**:
- `core/`: Fundamental types, initialization, and utilities
- `utils/`: Math, logging, and time utilities
- `timing/`: Real-time control loop at 1kHz with microsecond precision

**Physics & Kinematics**:
- `kinematics/`: Forward/inverse kinematics and collision detection
- `physics/`: Dynamics simulation with motor modeling, thermal, and electrical models
- `safety/`: Monitoring, limits, emergency stop, and safety enable/disable

**Control System**:
- `control/`: PID controllers, motion planning, and joint management
- `joints/`: Joint state management and coordination
- `hal/`: Hardware abstraction layer for motors and sensors

**Communication**:
- `communication/`: WebSocket server with binary protocol
- Protocol runs on port 9001 with mixed binary/JSON messaging
- 1kHz joint state updates (binary), 100Hz system status (JSON)

**Module Dependencies** (defined in build.zig:18-98):
```
utils (base) → timing → kinematics → safety → control → core → joints
                                          ↘     ↗
                                           hal ↗
                                       physics
                                   communication
```

### Frontend (Vite/React/Three.js)
The frontend provides 3D visualization and control interface:

**Key Components** (`src/components/`):
- `robot-arm.tsx`: Main 3D robot model using React Three Fiber with spring animations
- `robot-arm-simulation.tsx`: Simulation environment, backend integration, command dispatching
- `zig-controller.tsx`: WebSocket client, binary/JSON parsing, command serialization
- `control-panel.tsx`: User controls, sliders, presets, and diagnostics display
- `info-panel.tsx`: Static system information display

**Libraries**:
- Vite for build tooling
- React Three Fiber for 3D rendering
- @react-three/cannon for physics objects
- Tailwind CSS for styling
- Radix UI components for interface elements

### Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         BACKEND (Zig)                           │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ Physics Sim  │───▶│Joint Manager │───▶│ WebSocket Server │  │
│  │ (1kHz loop)  │    │              │    │   (port 9001)    │  │
│  └──────────────┘    └──────────────┘    └────────┬─────────┘  │
│                                                    │            │
└────────────────────────────────────────────────────┼────────────┘
                                                     │
                        WebSocket (binary + JSON)    │
                                                     ▼
┌────────────────────────────────────────────────────┼────────────┐
│                       FRONTEND (React)             │            │
│  ┌──────────────────┐    ┌─────────────────────────▼──────────┐│
│  │ Robot Arm (3D)   │◀───│       ZigController                ││
│  │ visualization    │    │  - Binary joint state parsing      ││
│  └──────────────────┘    │  - JSON status parsing             ││
│          ▲               │  - Command serialization           ││
│          │               └─────────────────────────────────────┘│
│  ┌───────┴──────────┐                                          │
│  │ Robot Simulation │◀──────────────────────────────────────────│
│  │  - Target angles │    ┌─────────────────────────────────────┐│
│  │  - Actual angles │    │         Control Panel               ││
│  │  - Backend sync  │◀───│  - Slider controls                  ││
│  └──────────────────┘    │  - Preset movements                 ││
│                          │  - Diagnostics display              ││
│                          └─────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

**Message Flow**:
- Backend → Frontend: Joint states (binary, 1kHz), System status (JSON, 100Hz), Collision data (JSON)
- Frontend → Backend: Position commands (JSON), Control mode changes (JSON), Safety commands (JSON)

## Integration Points

The system is designed for real-time integration where:
1. Backend is the authoritative source for all robot physics and state
2. Frontend displays actual joint positions received from backend
3. User inputs set target positions, which are sent as commands to backend
4. Communication happens via WebSocket on localhost:9001
5. Frontend uses spring animations for smooth interpolation between updates

## Important Files

- `backend/src/main.zig`: Main control loop with signal handling for graceful shutdown
- `backend/src/communication/main.zig`: WebSocket server and message dispatching
- `backend/src/joints/manager.zig`: Joint state management and physics integration
- `backend/src/safety/safety_monitor.zig`: Safety monitoring with enable/disable/e-stop
- `backend/build.zig`: Module dependency configuration
- `src/components/robot-arm-simulation.tsx`: Main frontend component with backend integration
- `src/components/zig-controller.tsx`: WebSocket client and protocol handling

## Development Notes

- Backend uses 1ms control loops for real-time performance
- Graceful shutdown: Ctrl+C or SIGTERM triggers clean shutdown sequence
- Safety system is enabled by default; can be disabled for testing
- Physics simulation includes motor dynamics, thermal modeling, and sensor noise
- Frontend automatically reconnects on WebSocket disconnection
- Target angles (user input) and actual angles (backend state) are tracked separately
- When backend is disconnected, frontend falls back to showing target angles
