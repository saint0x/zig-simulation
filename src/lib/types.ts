// lib/types.ts

import type { Vector3 } from "three"; // Assuming Vector3 is needed for CollisionData

// Constants mirroring backend protocol.zig
export const MESSAGE_TYPE_JOINT_STATE = 0;
export const MESSAGE_TYPE_SYSTEM_STATUS = 1;
export const MESSAGE_TYPE_COLLISION_DATA = 2;
export const MESSAGE_TYPE_COMMAND = 3;
export const MESSAGE_TYPE_CONNECTION_STATUS = 4;
// export const MESSAGE_TYPE_ROBOT_STATE = 5; // Not currently used by FE

export const SYSTEM_STATE_READY = 0;
export const SYSTEM_STATE_BUSY = 1;
export const SYSTEM_STATE_ERROR = 2;
export const SYSTEM_STATE_WARNING = 3;

export const CONTROL_MODE_POSITION = 0;
export const CONTROL_MODE_VELOCITY = 1;
export const CONTROL_MODE_TORQUE = 2;

export const COMMAND_TYPE_POSITION = 0;
export const COMMAND_TYPE_VELOCITY = 1;
export const COMMAND_TYPE_TORQUE = 2;
export const COMMAND_TYPE_CONTROL_MODE = 3;
export const COMMAND_TYPE_SAFETY = 4;

export const SAFETY_CMD_ENABLE = 0;
export const SAFETY_CMD_DISABLE = 1;
export const SAFETY_CMD_RESET = 2;
export const SAFETY_CMD_E_STOP = 3;

export const CONNECTION_STATUS_CONNECTED = 0;
export const CONNECTION_STATUS_DISCONNECTED = 1;
export const CONNECTION_STATUS_ERROR = 2;

// --- Interfaces for Data FROM Backend ---

/**
 * Mirrors backend protocol.JointStateMessage
 * Received as Binary payload for MESSAGE_TYPE_JOINT_STATE
 */
export interface JointState {
    timestamp_us: bigint; // u64
    positions: number[]; // [7]f32
    velocities: number[]; // [7]f32
    torques: number[]; // [7]f32
    temperatures: number[]; // [7]f32
    currents: number[]; // [7]f32
}

/**
 * Mirrors backend protocol.SafetyStatus
 * Part of SystemStatusMessage
 */
export interface SafetyStatus {
    soft_limits_active: boolean;
    emergency_stop: boolean;
    collision_detected: boolean;
}

/**
 * Mirrors backend protocol.SystemStatusMessage
 * Received as JSON payload for MESSAGE_TYPE_SYSTEM_STATUS
 */
export interface SystemStatus {
    state: number; // Uses SYSTEM_STATE_* constants
    error_code: string | null;
    safety_status: SafetyStatus;
    control_mode: number; // Uses CONTROL_MODE_* constants
}

/**
 * Mirrors backend protocol.CollisionMessage
 * Received as JSON payload for MESSAGE_TYPE_COLLISION_DATA
 * NOTE: Assumes Vector3 type exists for position/normal
 */
export interface CollisionData {
    detected: boolean;
    link1: string;
    link2: string;
    position: [number, number, number]; // Matches backend [3]f32
    penetration_depth: number;
    contact_normal: [number, number, number]; // Matches backend [3]f32
}

/**
 * Mirrors backend protocol.ConnectionStatusMessage
 * Received as JSON payload for MESSAGE_TYPE_CONNECTION_STATUS
 */
export interface ConnectionStatus {
  status: number; // Uses CONNECTION_STATUS_* constants
  message: string | null;
  timestamp_us: bigint; // u64
}

// --- Interfaces for Data TO Backend ---

/**
 * Mirrors backend protocol.ControlParameters
 * Part of CommandMessage
 */
export interface ControlParameters {
    stiffness?: number | null;
    damping?: number | null;
    feedforward?: boolean;
}

/**
 * Mirrors backend protocol.CommandMessage.safety
 * Part of CommandMessage
 */
export interface SafetyParameters {
    type: number; // Uses SAFETY_CMD_* constants
    zone_id?: string | null;
}

/**
 * Mirrors backend protocol.CommandMessage
 * Sent as JSON payload for MESSAGE_TYPE_COMMAND
 */
export interface CommandMessage {
    type: number; // Uses COMMAND_TYPE_* constants
    control_mode?: number | null; // Uses CONTROL_MODE_* constants
    values?: number[] | null;
    max_velocity?: number[] | null;
    max_acceleration?: number[] | null;
    parameters?: ControlParameters | null;
    safety?: SafetyParameters | null;
}

// --- Aggregated State potentially used in Frontend ---
// Example: Combine different pieces of state for easier management
export interface AppControlState {
    jointState?: JointState;
    systemStatus?: SystemStatus;
    collisionData?: CollisionData;
    connectionStatus?: ConnectionStatus;
    // Add other frontend-specific states as needed
} 