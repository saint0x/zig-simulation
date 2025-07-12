# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a KUKA robotic arm simulation project with a Zig backend for real-time control and physics simulation, and a Next.js/React frontend for 3D visualization and user interface.

## Commands

### Frontend Development
- **Development server**: `npm run dev` or `bun dev`
- **Build**: `npm run build`
- **Linting**: `npm run lint`
- **Production start**: `npm run start`

### Backend Development
- **Build**: `cd backend && zig build`
- **Run**: `cd backend && zig build run`
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
- `physics/`: Dynamics simulation and motor modeling
- `safety/`: Monitoring, limits, and emergency stop systems

**Control System**:
- `control/`: PID controllers, motion planning, and joint management
- `joints/`: Joint state management and coordination
- `hal/`: Hardware abstraction layer for motors and sensors

**Communication**:
- `communication/`: WebSocket server with binary protocol
- Protocol runs on port 9001 with mixed binary/JSON messaging

**Module Dependencies** (defined in build.zig:18-98):
```
utils (base) → timing → kinematics → safety → control → core → joints
                                          ↘     ↗
                                           hal ↗
                                       physics
                                   communication
```

### Frontend (Next.js/React/Three.js)
The frontend provides 3D visualization and control interface:

**Key Components**:
- `robot-arm.tsx`: Main 3D robot model using React Three Fiber
- `robot-arm-simulation.tsx`: Simulation environment and physics integration
- `zig-controller.tsx`: Backend communication bridge (WebSocket)
- `control-panel.tsx`: User controls and command interface
- `info-panel.tsx`: Status display and system information

**Libraries**:
- React Three Fiber for 3D rendering
- Tailwind CSS for styling
- Radix UI components for interface elements
- React Hook Form for form handling

### Data Flow
- Backend sends joint states at 1kHz, system status at 100Hz
- Frontend receives data via WebSocket and updates 3D visualization
- Commands flow from frontend controls to backend via JSON messages
- Binary protocol for high-frequency joint state data, JSON for commands and status

## Integration Points

The system is designed for real-time integration where:
1. Backend is the authoritative source for all robot physics and state
2. Frontend acts as visualization and user interface layer
3. Communication happens via WebSocket on localhost:9001
4. Frontend interpolates between backend updates for smooth 60fps rendering

## Important Files

- `backend/src/main.zig:12-167`: Main control loop and system initialization
- `backend/build.zig:18-98`: Module dependency configuration
- `INTEGRATION.md`: Detailed integration plan and protocol specifications
- `FE.md`: Frontend architecture and interface definitions
- `backend/BE.md`: Backend implementation details and phases

## Development Notes

- Backend uses 1ms control loops for real-time performance
- All physics calculations are deterministic and use fixed-point math where appropriate
- Safety systems include collision detection, joint limits, and emergency stops
- Frontend removes local physics simulation in favor of backend authority
- Protocol handles mixed binary (joint states) and JSON (commands/status) messaging