# KUKA Arm Simulation: Backend-Frontend Integration Plan

## 1. Introduction & Goal

This document outlines the plan for integrating the Zig-based backend control system with the Next.js/React-based frontend simulation for the KUKA Arm project. The primary goal is to transition the frontend from a static visual simulation to a dynamic one, driven by the backend's real-time physics, kinematics, and control logic.

This integration aligns with **Phase 4: Simulation Integration** of the `ROADMAPPY.md`.

## 2. Current State

### 2.1. Backend (`backend/BE.md`, `backend/ROADMAPPY.md`)
-   **Completed:** Core control system, kinematics, dynamics, and a defined communication layer (Phase 1-3 of `ROADMAPPY.md`).
-   **Communication:** Utilizes a custom binary protocol over WebSockets.
    -   **Data from Backend:** Real-time streaming of joint states (1kHz), system status (100Hz), and event-based collision data.
    -   **Data to Backend:** Expects joint commands, control mode changes, and safety commands via the same binary protocol.
-   **Source of Truth:** The backend is the definitive source for all robot physics, state, and sensor simulation.

### 2.2. Frontend (`app/`, `components/`, `hooks/`, `FE.md`)
-   **Current Functionality:** Provides a 3D visualization of the KUKA arm and UI panels for control and information display.
-   **`zig-controller.tsx`:** Currently a mock/placeholder for backend communication. This will be the central component for integration.
-   **Data Structures:** TypeScript interfaces (`JointState`, `SystemStatus`, `CollisionData`, etc., defined in `FE.md`) are designed to mirror backend data.
-   **Key Requirement from `FE.md`:** Remove frontend-based physics simulation; all robot state and movement must be driven by backend data.

## 3. Core Integration Component: `components/zig-controller.tsx`

The `zig-controller.tsx` component will be refactored to become the live communication bridge between the frontend and the backend. Its responsibilities will include:

1.  **WebSocket Connection Management:**
    *   Establish and maintain a persistent WebSocket connection to the Zig backend.
    *   Manage connection states (connecting, connected, disconnected, error).
    *   Implement reconnection logic.
    *   Backend WebSocket server endpoint is **`ws://localhost:9001`** ([x] Confirmed and backend default updated).

2.  **Binary Protocol Handling:**
    *   **Overall Structure:** Communication uses a custom framing protocol (see `backend/src/communication/protocol.zig` - `Frame` struct):
        *   `message_type: u8`
        *   `payload_length: u32` (little-endian)
        *   `payload: []u8`
    *   **Payload Formats (Backend -> Frontend):**
        *   `MESSAGE_TYPE_JOINT_STATE (0)`: Payload is **binary** `JointStateMessage` (packed struct, little-endian).
        *   `MESSAGE_TYPE_SYSTEM_STATUS (1)`: Payload is **JSON** string of `SystemStatusMessage`.
        *   `MESSAGE_TYPE_COLLISION_DATA (2)`: Payload is **JSON** string of `CollisionMessage`.
        *   `MESSAGE_TYPE_CONNECTION_STATUS (4)`: Payload is **JSON** string of `ConnectionStatusMessage`.
    *   **Payload Format (Frontend -> Backend):**
        *   `MESSAGE_TYPE_COMMAND (3)`: Payload is **JSON** string of `CommandMessage`.
    *   **(Critical Action Item):** This mixed binary/JSON approach differs from the initial understanding in `BE.md`. Frontend (`zig-controller.tsx`) must handle both binary `ArrayBuffer` and JSON string payloads based on the `message_type`.

3.  **State Management Integration:**
    *   Receive high-frequency data (especially `JointState` at 1kHz) from the backend.
    *   Efficiently update the frontend's global state or specific component states.
    *   This state will drive visualizations in `robot-arm.tsx` and data displays in `info-panel.tsx` and other UI elements.
    *   Consider performance implications and React rendering optimization techniques (memoization, context optimization, or dedicated state management libraries like Zustand/Jotai if not already in use).

4.  **Command Dispatch:**
    *   Provide functions/methods for other frontend components (e.g., `control-panel.tsx`) to send commands to the backend.
    *   Implement command queuing and synchronization mechanisms as noted in `FE.md` to manage the flow of commands and their acknowledgments or resulting state changes.

## 4. Shift in Frontend Philosophy

-   **Backend as Authority:** The frontend will no longer perform its own physics calculations for the robot arm. It will act as a sophisticated rendering engine and user interface, displaying the state provided by the backend.
-   **Visualization Focus:** The primary role of `robot-arm.tsx` becomes rendering based on backend data.
    -   **Smooth Interpolation:** Implement smooth visual interpolation between the discrete state updates received from the backend (e.g., interpolate 1kHz joint updates to achieve a smooth 60fps visual) as per `FE.md`.

## 5. Data Mapping Summary

Refer to `FE.md` (Section: "Backend-Frontend Interface"), `backend/src/communication/main.zig`, and `backend/src/communication/protocol.zig` for detailed interface definitions. The backend sends a Frame (`type: u8`, `payload_len: u32`, `payload: []u8`). [x] Backend implementation verified and cleaned.

### 5.1. Backend to Frontend (Payload Format within Frame):
| Backend `message_type` (protocol.zig) | Message Name         | Payload Format | Frontend Interface (FE.md) | Frequency (approx) | Notes |
| :------------------------------------ | :------------------- | :------------- | :------------------------- | :----------------- | :---- |
| `MESSAGE_TYPE_JOINT_STATE (0)`        | `JointStateMessage`  | **Binary**     | `JointState`               | 1kHz               | [x] Uses NUM_JOINTS (7) |
| `MESSAGE_TYPE_SYSTEM_STATUS (1)`      | `SystemStatusMessage`| **JSON**       | `SystemStatus`             | 100Hz              |
| `MESSAGE_TYPE_COLLISION_DATA (2)`     | `CollisionMessage`   | **JSON**       | `CollisionData`            | 100Hz (on change)  |
| `MESSAGE_TYPE_CONNECTION_STATUS (4)`  | `ConnectionStatusMessage` | **JSON**   | (New - define in FE)     | On event           |


### 5.2. Frontend to Backend (Payload Format within Frame):
| Frontend Command (FE.md) | `message_type` (protocol.zig) | Payload Format | Backend Struct (protocol.zig) |
| :----------------------- | :---------------------------- | :------------- | :---------------------------- |
| `JointCommand`           | `MESSAGE_TYPE_COMMAND (3)`    | **JSON**       | `CommandMessage`              |
| `ControlModeRequest`     | `MESSAGE_TYPE_COMMAND (3)`    | **JSON**       | `CommandMessage`              |
| `SafetyCommand`          | `MESSAGE_TYPE_COMMAND (3)`    | **JSON**       | `CommandMessage`              |

## 6. Potential Challenges

-   **Binary Data in JavaScript/TypeScript:** Requires meticulous implementation for parsing and constructing binary messages.
-   **WebSocket Stability & Error Handling:** Robust connection management is crucial.
-   **Performance:** Handling high-frequency updates in React without UI lag. Efficient binary data processing.
-   **Synchronization:** Ensuring commands and state updates are handled coherently.
-   **Debugging:** Inspecting raw binary WebSocket frames can be complex. Comprehensive logging on both client and server will be essential.

## 7. Frontend Integration Phases (aligns with `FE.md` "Integration Timeline")

### Phase 1: Basic Communication & Visualization
-   [ ] Implement WebSocket connection in `zig-controller.tsx`.
-   [ ] Implement deserialization for `JointState` messages.
-   [ ] Pipe `JointState` data to `robot-arm.tsx` for basic visualization.
-   [ ] Remove mock physics from frontend for joint updates.
-   [ ] Implement serialization for a simple `JointCommand` (e.g., target position).
-   [ ] Allow `control-panel.tsx` to trigger this basic command.
-   [ ] **Goal:** Robot arm in frontend moves based on backend data.

### Phase 2: Enhanced Visualization & Full Data Flow
-   [ ] Implement deserialization for `SystemStatus` and `CollisionData`.
-   [ ] Integrate `SystemStatus` into `info-panel.tsx`.
-   [ ] Visualize collision information.
-   [ ] Implement visual feedback enhancements from `FE.md` (motor temp, torque, safety zones, trajectory preview).
-   [ ] Ensure smooth visual interpolation (e.g., 60fps from 1kHz data).

### Phase 3: Advanced Features & UI/UX Improvements
-   [ ] Implement full serialization for all defined `JointCommand`, `ControlModeRequest`, and `SafetyCommand` types.
-   [ ] Integrate these commands fully with `control-panel.tsx` and other relevant UI elements.
-   [ ] Develop UI/UX improvements from `FE.md` (detailed motor status, advanced trajectory planning UI, safety zone config, error/warning notifications, real-time plotting).
-   [ ] Implement robust error handling for communication issues and backend-reported errors.
-   [ ] Focus on performance optimization for rendering and state management.

### Phase 4: Testing, Validation & Polish
-   [ ] Unit tests for protocol handling (`zig-controller.tsx`).
-   [ ] Integration tests simulating various backend scenarios (connection loss, error states).
-   [ ] Visual regression tests.
-   [ ] Performance benchmarking to meet `FE.md` requirements.
-   [ ] UI polish and final documentation updates.

## 8. Conclusion

This integration is a pivotal step in realizing the KUKA arm simulation project. Success depends on clear communication between frontend and backend development, meticulous implementation of the binary protocol, and a strong focus on performance and user experience.

## 9. Detailed Phase 1 Checklist: Frontend (`components/zig-controller.tsx`)

This checklist details the initial steps for refactoring `components/zig-controller.tsx` to establish basic communication with the backend. Backend development for WebSocket and protocol handling must occur in parallel.

### 9.1. Configuration & Setup
- [ ] **Define Connection Parameters:**
    - [ ] In `zig-controller.tsx` (or a shared config file), define `const WEBSOCKET_URL = "ws://localhost:9001";`. (Port confirmed).
- [ ] **Remove Mock Zig Code Display:** Delete the existing JSX that renders the mock Zig code string. The component will likely become non-visual or have a minimal UI for status/debugging.
- [ ] **Props Review:** Confirm how `controlSignals` and `setControlSignals` (or their equivalents, e.g., a global state setter) will be used to integrate with `robot-arm.tsx`, `info-panel.tsx`, and `control-panel.tsx`.

### 9.2. WebSocket Connection Management
- [ ] **Establish WebSocket Connection:**
    - [ ] Implement a function to initialize and open the WebSocket connection.
    - [ ] Call this function on component mount (e.g., in a `useEffect` hook).
- [ ] **Connection Lifecycle Handlers:**
    - [ ] Implement `onopen`: Log successful connection.
    - [ ] Implement `onclose`: Log disconnection, potentially attempt reconnection.
    - [ ] Implement `onerror`: Log errors.
- [ ] **Cleanup:** Ensure WebSocket connection is properly closed on component unmount.

### 9.3. Binary Protocol - Deserialization (Backend -> Frontend)
- [ ] **Message Header Parsing (Frame Level):**
    - [ ] Read `message_type: u8`.
    - [ ] Read `payload_length: u32` (little-endian).
    - [ ] Extract the `payload: ArrayBuffer` of `payload_length`.
    - [ ] **Endianness for `payload_length` is little-endian (confirmed from `Frame.decode`). Endianness for `JointStateMessage` (binary payload) is also little-endian (Zig default for `std.mem.asBytes` on common platforms).**
- [ ] **Payload Deserialization (based on `message_type`):**
    - [ ] **If `MESSAGE_TYPE_JOINT_STATE (0)`:**
        - [ ] Implement `deserializeJointState(payload: ArrayBuffer): JointState`.
        - [ ] Parse binary fields according to `protocol.zig::JointStateMessage` (packed struct, little-endian: `timestamp_us: u64`, `positions: [6]f32`, etc.).
    - [ ] **If `MESSAGE_TYPE_SYSTEM_STATUS (1)`:**
        - [ ] Convert `payload` (ArrayBuffer) to a UTF-8 string.
        - [ ] Parse JSON string into `SystemStatus` (FE.md interface).
    - [ ] **If `MESSAGE_TYPE_COLLISION_DATA (2)`:**
        - [ ] Convert `payload` to UTF-8 string.
        - [ ] Parse JSON string into `CollisionData` (FE.md interface).
    - [ ] **If `MESSAGE_TYPE_CONNECTION_STATUS (4)`:**
        - [ ] Convert `payload` to UTF-8 string.
        - [ ] Parse JSON string into a new `ConnectionStatus` interface (to be defined in frontend, based on `protocol.zig::ConnectionStatusMessage`).
- [ ] **State Update:**
    - [ ] On successful deserialization, use `setControlSignals` (or equivalent) to update the frontend application state.

### 9.4. Frame and JSON Protocol - Serialization (Frontend -> Backend)
- [ ] **`CommandMessage` Preparation (JSON for Payload):**
    - [ ] Create a JavaScript object matching `protocol.zig::CommandMessage` structure for `JointCommand`, `ControlModeRequest`, or `SafetyCommand`.
    - [ ] Convert this JS object to a JSON string.
    - [ ] Convert the JSON string to `Uint8Array` (UTF-8 encoded). This is the `payload_bytes`.
- [ ] **Frame Serialization:**
    - [ ] Implement `serializeFrame(messageType: number, payloadBytes: Uint8Array): ArrayBuffer`.
    - [ ] Create an `ArrayBuffer` for the full frame (`1 byte for type + 4 bytes for length + payloadBytes.length`).
    - [ ] Use a `DataView` to write:
        - [ ] `messageType: u8`.
        - [ ] `payloadBytes.length: u32` (little-endian).
        - [ ] `payloadBytes: Uint8Array`.
- [ ] **Send Command Function:**
    - [ ] Modify `sendJointCommand` (and create similar for other commands) in `zig-controller.tsx`.
    - [ ] It will prepare the JSON `CommandMessage`, convert to `Uint8Array`, then call `serializeFrame(MESSAGE_TYPE_COMMAND, payloadBytes)` and then `websocket.send()`.

### 9.5. Initial Integration & Testing
- [ ] **Backend Prerequisite:** Confirm backend has implemented its WebSocket server and can send `JointState` messages and receive `JointCommand` messages.
- [ ] **Test Data Flow:**
    - [ ] Verify `zig-controller.tsx` connects to the backend WebSocket.
    - [ ] Verify `JointState` messages are received, deserialized, and update `robot-arm.tsx` (robot should move based on backend data).
    - [ ] Verify sending a `JointCommand` from the frontend is received by the backend.
- [ ] **Remove Mock Physics:** Ensure any frontend-only physics simulation in `robot-arm-simulation.tsx` or `robot-arm.tsx` related to joint movement is disabled or removed, as `FE.md` dictates. 