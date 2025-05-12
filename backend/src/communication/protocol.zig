const std = @import("std");
const core = @import("core");

// Message types as constants
pub const MESSAGE_TYPE_JOINT_STATE = 0;    // Binary format
pub const MESSAGE_TYPE_SYSTEM_STATUS = 1;  // JSON
pub const MESSAGE_TYPE_COLLISION_DATA = 2; // JSON
pub const MESSAGE_TYPE_COMMAND = 3;        // JSON from frontend
pub const MESSAGE_TYPE_CONNECTION_STATUS = 4; // JSON
pub const MESSAGE_TYPE_ROBOT_STATE = 5; // Binary format for RobotStateMessage

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

// Connection status constants
pub const CONNECTION_STATUS_CONNECTED = 0;
pub const CONNECTION_STATUS_DISCONNECTED = 1;
pub const CONNECTION_STATUS_ERROR = 2;

pub const MessageType = enum(u8) {
    JOINT_STATE = MESSAGE_TYPE_JOINT_STATE,
    SYSTEM_STATUS = MESSAGE_TYPE_SYSTEM_STATUS,
    COLLISION_DATA = MESSAGE_TYPE_COLLISION_DATA,
    COMMAND = MESSAGE_TYPE_COMMAND,
    CONNECTION_STATUS = MESSAGE_TYPE_CONNECTION_STATUS,
    ROBOT_STATE = MESSAGE_TYPE_ROBOT_STATE,
};

pub const GripperState = enum(u8) {
    OPEN,
    CLOSED,
    MOVING,
    ERROR,
};

// Binary format for high-frequency joint states
pub const JointStateMessage = extern struct {
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

// Named struct for control parameters
pub const ControlParameters = struct {
    stiffness: ?f32 = null,
    damping: ?f32 = null,
    feedforward: bool = false,
};

pub const CommandMessage = struct {
    type: u8,  // Uses COMMAND_TYPE_* constants
    control_mode: ?u8 = null,
    values: ?[]f32 = null,
    max_velocity: ?[]f32 = null,
    max_acceleration: ?[]f32 = null,
    parameters: ?ControlParameters = null, // Use named struct
    safety: ?struct {
        type: u8,  // Uses SAFETY_CMD_* constants
        zone_id: ?[]const u8 = null,
    } = null,
};

pub const ConnectionStatusMessage = struct {
    status: u8,  // Uses CONNECTION_STATUS_* constants
    message: ?[]const u8,
    timestamp_us: u64,

    pub fn jsonStringify(self: ConnectionStatusMessage, writer: anytype) !void {
        try writer.write("{\"status\":");
        const status_str = switch (self.status) {
            CONNECTION_STATUS_CONNECTED => "\"connected\"",
            CONNECTION_STATUS_DISCONNECTED => "\"disconnected\"",
            CONNECTION_STATUS_ERROR => "\"error\"",
            else => "\"unknown\"",
        };
        try writer.write(status_str);
        try writer.write(",\"message\":");
        if (self.message) |msg| {
            try writer.write("\"");
            try writer.write(msg); // Assuming msg doesn't need further JSON escaping for this context
            try writer.write("\"");
        } else {
            try writer.write("null");
        }
        try writer.write(",\"timestamp_us\":");
        try writer.print("{}", .{self.timestamp_us}); // Use print for basic number formatting
        try writer.write("}");
    }
};

// WebSocket frame handling
pub const Frame = struct {
    const Self = @This();
    
    type: u8,  // Uses MESSAGE_TYPE_* constants
    payload: []const u8,
    
    pub fn encode(self: Self, writer: anytype) !void {
        try writer.writeByte(self.type);
        var len_bytes: [4]u8 = undefined;
        const payload_len_u32: u32 = @truncate(self.payload.len);

        // Manual little-endian u32 to 4-byte array
        len_bytes[0] = @truncate(payload_len_u32);
        len_bytes[1] = @truncate(payload_len_u32 >> 8);
        len_bytes[2] = @truncate(payload_len_u32 >> 16);
        len_bytes[3] = @truncate(payload_len_u32 >> 24);

        try writer.writeAll(&len_bytes);
        try writer.writeAll(self.payload);
    }
    
    pub fn decode(reader: anytype, allocator: std.mem.Allocator) !Self {
        const msg_type = try reader.readByte();
        var len_bytes: [4]u8 = undefined;
        try reader.readNoEof(&len_bytes);

        // Manual little-endian 4-byte array to u32
        const byte0 = @as(u32, len_bytes[0]);
        const byte1 = @as(u32, len_bytes[1]) << 8;
        const byte2 = @as(u32, len_bytes[2]) << 16;
        const byte3 = @as(u32, len_bytes[3]) << 24;
        const payload_len = byte0 | byte1 | byte2 | byte3;

        const payload = try allocator.alloc(u8, payload_len);
        try reader.readNoEof(payload);
        
        return Self{
            .type = msg_type,
            .payload = payload,
        };
    }
};

pub const RobotStateMessage = extern struct {
    message_type: MessageType = .ROBOT_STATE,
    timestamp_us: u64,
    joint_positions: [6]u32,
    joint_velocities: [6]u32,
    joint_torques: [6]u32,
    object_position: [3]u32,
    object_orientation: [4]u32,
    object_linear_velocity: [3]u32,
    object_angular_velocity: [3]u32,
    gripper_state: GripperState,
    end_effector_pose: [7]u32,
    padding: [2]u8 = undefined,
}; 