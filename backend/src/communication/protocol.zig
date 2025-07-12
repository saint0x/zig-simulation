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
    positions: [core.types.NUM_JOINTS]f32,
    velocities: [core.types.NUM_JOINTS]f32,
    torques: [core.types.NUM_JOINTS]f32,
    temperatures: [core.types.NUM_JOINTS]f32,
    currents: [core.types.NUM_JOINTS]f32,
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
    status: []const u8,  // Use string directly instead of constants
    message: ?[]const u8,
    timestamp_us: u64,
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

// WebSocket frame implementation (RFC 6455)
pub const WebSocketFrame = struct {
    fin: bool,
    opcode: u4,
    masked: bool,
    payload: []const u8,
    
    pub const OPCODE_TEXT = 0x1;
    pub const OPCODE_BINARY = 0x2;
    pub const OPCODE_CLOSE = 0x8;
    pub const OPCODE_PING = 0x9;
    pub const OPCODE_PONG = 0xA;
    
    pub fn encodeText(data: []const u8, writer: anytype) !void {
        return encode(data, OPCODE_TEXT, writer);
    }
    
    pub fn encodeBinary(data: []const u8, writer: anytype) !void {
        return encode(data, OPCODE_BINARY, writer);
    }
    
    pub fn encodePong(data: []const u8, writer: anytype) !void {
        return encode(data, OPCODE_PONG, writer);
    }
    
    fn encode(data: []const u8, opcode: u4, writer: anytype) !void {
        // First byte: FIN=1, RSV=000, OPCODE
        const first_byte: u8 = 0x80 | @as(u8, opcode); // FIN=1, RSV=000
        try writer.writeByte(first_byte);
        
        const payload_len = data.len;
        
        if (payload_len < 126) {
            // Small payload: length fits in 7 bits
            try writer.writeByte(@truncate(payload_len));
        } else if (payload_len < 65536) {
            // Medium payload: use 16-bit length
            try writer.writeByte(126);
            try writer.writeInt(u16, @truncate(payload_len), .big);
        } else {
            // Large payload: use 64-bit length
            try writer.writeByte(127);
            try writer.writeInt(u64, payload_len, .big);
        }
        
        // Payload (server doesn't mask data)
        try writer.writeAll(data);
    }
    
    pub fn decode(reader: anytype, buffer: []u8) !WebSocketFrame {
        // Read first byte
        const first_byte = try reader.readByte();
        const fin = (first_byte & 0x80) != 0;
        const opcode: u4 = @truncate(first_byte & 0x0F);
        
        // Read second byte
        const second_byte = try reader.readByte();
        const masked = (second_byte & 0x80) != 0;
        var payload_len: u64 = second_byte & 0x7F;
        
        // Extended payload length
        if (payload_len == 126) {
            payload_len = try reader.readInt(u16, .big);
        } else if (payload_len == 127) {
            payload_len = try reader.readInt(u64, .big);
        }
        
        // Masking key (if present)
        var mask: [4]u8 = undefined;
        if (masked) {
            try reader.readNoEof(&mask);
        }
        
        // Read payload
        if (payload_len > buffer.len) {
            return error.PayloadTooLarge;
        }
        
        const payload = buffer[0..payload_len];
        try reader.readNoEof(payload);
        
        // Unmask payload if needed
        if (masked) {
            for (payload, 0..) |*byte, i| {
                byte.* ^= mask[i % 4];
            }
        }
        
        return WebSocketFrame{
            .fin = fin,
            .opcode = opcode,
            .masked = masked,
            .payload = payload,
        };
    }
};

// Simplified Message Encoder for Production WebSocket Protocol
// WebSocket is just transport - send direct message content
pub const MessageEncoder = struct {
    /// Send binary data (joint states) as WebSocket binary frame
    pub fn sendBinaryMessage(data: []const u8, writer: anytype) !void {
        try WebSocketFrame.encodeBinary(data, writer);
    }
    
    /// Send JSON data (status, collision, etc.) as WebSocket text frame
    pub fn sendJsonMessage(json_string: []const u8, writer: anytype) !void {
        try WebSocketFrame.encodeText(json_string, writer);
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