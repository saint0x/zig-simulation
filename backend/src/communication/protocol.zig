const std = @import("std");
const core = @import("core");

// Message types as constants
pub const MESSAGE_TYPE_JOINT_STATE = 0;    // Binary format
pub const MESSAGE_TYPE_SYSTEM_STATUS = 1;  // JSON
pub const MESSAGE_TYPE_COLLISION_DATA = 2; // JSON
pub const MESSAGE_TYPE_COMMAND = 3;        // JSON from frontend

// System states
pub const SYSTEM_STATE_READY = 0;
pub const SYSTEM_STATE_BUSY = 1;
pub const SYSTEM_STATE_ERROR = 2;
pub const SYSTEM_STATE_WARNING = 3;

// Control modes
pub const CONTROL_MODE_POSITION = 0;
pub const CONTROL_MODE_VELOCITY = 1;
pub const CONTROL_MODE_TORQUE = 2;

// Command types
pub const COMMAND_TYPE_POSITION = 0;
pub const COMMAND_TYPE_VELOCITY = 1;
pub const COMMAND_TYPE_TORQUE = 2;
pub const COMMAND_TYPE_CONTROL_MODE = 3;
pub const COMMAND_TYPE_SAFETY = 4;

// Safety command types
pub const SAFETY_CMD_ENABLE = 0;
pub const SAFETY_CMD_DISABLE = 1;
pub const SAFETY_CMD_RESET = 2;
pub const SAFETY_CMD_E_STOP = 3;

// Binary format for high-frequency joint states
pub const JointStateMessage = packed struct {
    timestamp_us: u64,
    positions: [6]f32,
    velocities: [6]f32,
    torques: [6]f32,
    temperatures: [6]f32,
    currents: [6]f32,
};

pub const SafetyStatus = struct {
    soft_limits_active: bool,
    emergency_stop: bool,
    collision_detected: bool,
};

pub const SystemStatusMessage = struct {
    state: u8,  // Uses SYSTEM_STATE_* constants
    error_code: ?[]const u8,
    safety_status: SafetyStatus,
    control_mode: u8,  // Uses CONTROL_MODE_* constants
};

pub const CollisionMessage = struct {
    detected: bool,
    link1: []const u8,
    link2: []const u8,
    position: [3]f32,  // x, y, z
    penetration_depth: f32,
    contact_normal: [3]f32,  // nx, ny, nz
};

pub const CommandMessage = struct {
    type: u8,  // Uses COMMAND_TYPE_* constants
    values: ?[]f32 = null,
    max_velocity: ?[]f32 = null,
    max_acceleration: ?[]f32 = null,
    parameters: ?struct {
        stiffness: ?f32 = null,
        damping: ?f32 = null,
        feedforward: bool = false,
    } = null,
    safety: ?struct {
        type: u8,  // Uses SAFETY_CMD_* constants
        zone_id: ?[]const u8 = null,
    } = null,
};

// WebSocket frame handling
pub const Frame = struct {
    const Self = @This();
    
    type: u8,  // Uses MESSAGE_TYPE_* constants
    payload: []const u8,
    
    pub fn encode(self: Self, writer: anytype) !void {
        try writer.writeByte(self.type);
        try writer.writeIntLittle(u32, self.payload.len);
        try writer.writeAll(self.payload);
    }
    
    pub fn decode(reader: anytype, allocator: std.mem.Allocator) !Self {
        const msg_type = try reader.readByte();
        const payload_len = try reader.readIntLittle(u32);
        const payload = try allocator.alloc(u8, payload_len);
        try reader.readNoEof(payload);
        
        return Self{
            .type = msg_type,
            .payload = payload,
        };
    }
}; 