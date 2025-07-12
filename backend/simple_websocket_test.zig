const std = @import("std");

// Simplified WebSocket frame encoder for testing
fn encodeTextFrame(data: []const u8, writer: anytype) !void {
    // First byte: FIN=1, RSV=000, OPCODE=0x1 (text)
    const first_byte: u8 = 0x80 | 0x1; // 0x81
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

test "WebSocket text frame encoding" {
    const test_json = "{\"test\":\"message\"}";
    
    var buffer: [1024]u8 = undefined;
    var stream = std.io.fixedBufferStream(&buffer);
    
    try encodeTextFrame(test_json, stream.writer());
    
    const frame_data = buffer[0..stream.pos];
    
    std.log.info("=== WebSocket Frame Test ===", .{});
    std.log.info("Input JSON: {s}", .{test_json});
    std.log.info("Input length: {}", .{test_json.len});
    std.log.info("Frame size: {}", .{frame_data.len});
    std.log.info("Expected frame size: {}", .{2 + test_json.len}); // 2 header bytes + payload
    
    // Print all frame bytes
    std.log.info("Frame bytes (hex):", .{});
    for (frame_data, 0..) |byte, i| {
        if (i % 16 == 0) std.log.info("  {:02}: ", .{i});
        std.log.info("{X:0>2} ", .{byte});
        if (i % 16 == 15 or i == frame_data.len - 1) std.log.info("", .{});
    }
    
    // Verify frame structure
    try std.testing.expect(frame_data[0] == 0x81); // FIN + TEXT frame
    try std.testing.expect(frame_data[1] == test_json.len); // Payload length
    
    // Verify payload
    const payload = frame_data[2..2 + test_json.len];
    try std.testing.expectEqualStrings(test_json, payload);
    
    std.log.info("✅ Frame structure is correct", .{});
    
    // Test the exact same frame that backend sends
    const backend_json = "{\"status\":\"connected\",\"message\":\"New client connected\",\"timestamp_us\":1752343384019945}";
    std.log.info("=== Backend Frame Test ===", .{});
    std.log.info("Backend JSON: {s}", .{backend_json});
    std.log.info("Backend JSON length: {}", .{backend_json.len});
    
    var backend_buffer: [1024]u8 = undefined;
    var backend_stream = std.io.fixedBufferStream(&backend_buffer);
    
    try encodeTextFrame(backend_json, backend_stream.writer());
    
    const backend_frame = backend_buffer[0..backend_stream.pos];
    std.log.info("Backend frame size: {}", .{backend_frame.len});
    std.log.info("First 16 bytes: ", .{});
    for (backend_frame[0..@min(16, backend_frame.len)]) |byte| {
        std.log.info("{X:0>2} ", .{byte});
    }
    std.log.info("", .{});
    
    // Compare with what backend logged: 81577B22737461747573223A22636F6E
    const expected_start = [_]u8{ 0x81, 0x57, 0x7B, 0x22, 0x73, 0x74, 0x61, 0x74, 0x75, 0x73, 0x22, 0x3A, 0x22, 0x63, 0x6F, 0x6E };
    try std.testing.expectEqualSlices(u8, &expected_start, backend_frame[0..16]);
    
    std.log.info("✅ Backend frame matches expected format", .{});
    
    // Also print the result to stderr so we can see it
    const stderr = std.io.getStdErr().writer();
    try stderr.print("\n=== ANALYSIS ===\n", .{});
    try stderr.print("Backend sends frame: 81 57 7B 22...\n", .{});
    try stderr.print("  0x81 = FIN(1) + TEXT(1) = Valid WebSocket text frame\n", .{});
    try stderr.print("  0x57 = 87 bytes payload length\n", .{});
    try stderr.print("  Frame structure is CORRECT\n", .{});
    try stderr.print("  Issue must be in frontend handling!\n\n", .{});
}

test "WebSocket frame decoding" {
    // Test decoding the exact frame the backend sends
    const backend_frame_start = [_]u8{ 0x81, 0x57, 0x7B, 0x22, 0x73, 0x74, 0x61, 0x74, 0x75, 0x73, 0x22, 0x3A, 0x22, 0x63, 0x6F, 0x6E };
    
    std.log.info("=== WebSocket Frame Decoding Test ===", .{});
    std.log.info("Frame header: {X:0>2} {X:0>2}", .{ backend_frame_start[0], backend_frame_start[1] });
    
    const first_byte = backend_frame_start[0];
    const second_byte = backend_frame_start[1];
    
    const fin = (first_byte & 0x80) != 0;
    const opcode = first_byte & 0x0F;
    const masked = (second_byte & 0x80) != 0;
    const payload_len = second_byte & 0x7F;
    
    std.log.info("FIN: {}", .{fin});
    std.log.info("Opcode: 0x{X} (expected 0x1 for text)", .{opcode});
    std.log.info("Masked: {} (should be false for server->client)", .{masked});
    std.log.info("Payload length: {} bytes", .{payload_len});
    
    try std.testing.expect(fin == true);
    try std.testing.expect(opcode == 0x1); // TEXT frame
    try std.testing.expect(masked == false); // Server frames are not masked
    try std.testing.expect(payload_len == 87); // Length of the JSON
    
    std.log.info("✅ Frame header is valid WebSocket format", .{});
}