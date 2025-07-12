const std = @import("std");
const testing = std.testing;
const expect = testing.expect;
const expectEqual = testing.expectEqual;
const expectEqualSlices = testing.expectEqualSlices;

// Import our communication protocol via module system
const communication = @import("communication");
const protocol = communication.protocol;

test "Frame encoding and decoding" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Test data
    const test_payload = "Hello, WebSocket!";
    const test_type: u8 = protocol.MESSAGE_TYPE_SYSTEM_STATUS;
    
    // Create frame
    const frame = protocol.Frame{
        .type = test_type,
        .payload = test_payload,
    };
    
    // Encode frame
    var buffer = std.ArrayList(u8).init(allocator);
    defer buffer.deinit();
    
    try frame.encode(buffer.writer());
    
    // Verify encoded format
    try expectEqual(test_type, buffer.items[0]); // First byte is type
    
    // Next 4 bytes are length (little-endian)
    const encoded_length = std.mem.readInt(u32, buffer.items[1..5][0..4], .little);
    try expectEqual(@as(u32, test_payload.len), encoded_length);
    
    // Remaining bytes are payload
    try expectEqualSlices(u8, test_payload, buffer.items[5..]);
    
    // Decode frame
    var stream = std.io.fixedBufferStream(buffer.items);
    const decoded_frame = try protocol.Frame.decode(stream.reader(), allocator);
    
    try expectEqual(test_type, decoded_frame.type);
    try expectEqualSlices(u8, test_payload, decoded_frame.payload);
}

test "JointStateMessage binary serialization" {
    // Create test joint state
    const joint_state = protocol.JointStateMessage{
        .timestamp_us = 1234567890,
        .positions = [_]f32{ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 },
        .velocities = [_]f32{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 },
        .torques = [_]f32{ 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0 },
        .temperatures = [_]f32{ 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0 },
        .currents = [_]f32{ 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5 },
    };
    
    // Convert to bytes
    var bytes: [@sizeOf(protocol.JointStateMessage)]u8 = undefined;
    @memset(&bytes, 0);
    @memcpy(&bytes, std.mem.asBytes(&joint_state));
    
    // Verify size
    try expectEqual(@as(usize, @sizeOf(protocol.JointStateMessage)), bytes.len);
    
    // Convert back
    const reconstructed = std.mem.bytesToValue(protocol.JointStateMessage, &bytes);
    
    // Verify data integrity
    try expectEqual(joint_state.timestamp_us, reconstructed.timestamp_us);
    
    for (joint_state.positions, reconstructed.positions) |orig, recon| {
        try expectEqual(orig, recon);
    }
    
    for (joint_state.velocities, reconstructed.velocities) |orig, recon| {
        try expectEqual(orig, recon);
    }
}

test "JSON message serialization" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Test SystemStatusMessage
    const status = protocol.SystemStatusMessage{
        .state = protocol.SYSTEM_STATE_READY,
        .error_code = null,
        .safety_status = .{
            .soft_limits_active = true,
            .emergency_stop = false,
            .collision_detected = false,
        },
        .control_mode = protocol.CONTROL_MODE_POSITION,
    };
    
    // Serialize to JSON
    const json_string = try std.json.stringifyAlloc(allocator, status, .{});
    defer allocator.free(json_string);
    
    // Verify JSON contains expected fields
    try expect(std.mem.indexOf(u8, json_string, "state") != null);
    try expect(std.mem.indexOf(u8, json_string, "safety_status") != null);
    try expect(std.mem.indexOf(u8, json_string, "control_mode") != null);
    
    // Deserialize back
    const parsed = try std.json.parseFromSlice(
        protocol.SystemStatusMessage,
        allocator,
        json_string,
        .{}
    );
    defer parsed.deinit();
    
    try expectEqual(status.state, parsed.value.state);
    try expectEqual(status.control_mode, parsed.value.control_mode);
    try expectEqual(status.safety_status.soft_limits_active, parsed.value.safety_status.soft_limits_active);
}

test "CommandMessage parsing" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Test position command JSON
    const position_cmd_json = 
        \\{
        \\  "type": 0,
        \\  "values": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
        \\  "max_velocity": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        \\}
    ;
    
    const parsed = try std.json.parseFromSlice(
        protocol.CommandMessage,
        allocator,
        position_cmd_json,
        .{}
    );
    defer parsed.deinit();
    
    try expectEqual(@as(u8, protocol.COMMAND_TYPE_POSITION), parsed.value.type);
    try expect(parsed.value.values != null);
    try expectEqual(@as(usize, 7), parsed.value.values.?.len);
    try expectEqual(@as(f32, 1.0), parsed.value.values.?[0]);
    
    // Test safety command
    const safety_cmd_json = 
        \\{
        \\  "type": 4,
        \\  "safety": {
        \\    "type": 3
        \\  }
        \\}
    ;
    
    const safety_parsed = try std.json.parseFromSlice(
        protocol.CommandMessage,
        allocator,
        safety_cmd_json,
        .{}
    );
    defer safety_parsed.deinit();
    
    try expectEqual(@as(u8, protocol.COMMAND_TYPE_SAFETY), safety_parsed.value.type);
    try expect(safety_parsed.value.safety != null);
    try expectEqual(@as(u8, protocol.SAFETY_CMD_E_STOP), safety_parsed.value.safety.?.type);
}

test "Message type constants" {
    // Verify message type constants are unique
    const types = [_]u8{
        protocol.MESSAGE_TYPE_JOINT_STATE,
        protocol.MESSAGE_TYPE_SYSTEM_STATUS,
        protocol.MESSAGE_TYPE_COLLISION_DATA,
        protocol.MESSAGE_TYPE_COMMAND,
        protocol.MESSAGE_TYPE_CONNECTION_STATUS,
    };
    
    // Check for duplicates
    for (types, 0..) |type1, i| {
        for (types[i + 1..]) |type2| {
            try expect(type1 != type2);
        }
    }
    
    // Verify command type constants
    const cmd_types = [_]u8{
        protocol.COMMAND_TYPE_POSITION,
        protocol.COMMAND_TYPE_VELOCITY,
        protocol.COMMAND_TYPE_TORQUE,
        protocol.COMMAND_TYPE_CONTROL_MODE,
        protocol.COMMAND_TYPE_SAFETY,
    };
    
    for (cmd_types, 0..) |type1, i| {
        for (cmd_types[i + 1..]) |type2| {
            try expect(type1 != type2);
        }
    }
}

test "ConnectionStatusMessage custom JSON serialization" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    const status = protocol.ConnectionStatusMessage{
        .status = protocol.CONNECTION_STATUS_CONNECTED,
        .message = "Test connection",
        .timestamp_us = 1234567890,
    };
    
    // Test custom JSON serialization
    var buffer = std.ArrayList(u8).init(allocator);
    defer buffer.deinit();
    
    try status.jsonStringify(buffer.writer());
    const json_output = try buffer.toOwnedSlice();
    defer allocator.free(json_output);
    
    // Verify JSON structure
    try expect(std.mem.indexOf(u8, json_output, "\"status\":\"connected\"") != null);
    try expect(std.mem.indexOf(u8, json_output, "\"message\":\"Test connection\"") != null);
    try expect(std.mem.indexOf(u8, json_output, "\"timestamp_us\":1234567890") != null);
}

test "Protocol robustness with invalid data" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Test malformed JSON
    const invalid_json = "{ invalid json }";
    
    const result = std.json.parseFromSlice(
        protocol.CommandMessage,
        allocator,
        invalid_json,
        .{}
    );
    
    try expect(std.meta.isError(result));
    
    // Test frame with invalid length
    var invalid_frame_data = [_]u8{ 
        protocol.MESSAGE_TYPE_JOINT_STATE, // type
        0xFF, 0xFF, 0xFF, 0xFF, // invalid length (very large)
        // No payload data
    };
    
    var stream = std.io.fixedBufferStream(&invalid_frame_data);
    const frame_result = protocol.Frame.decode(stream.reader(), allocator);
    
    try expect(std.meta.isError(frame_result));
}