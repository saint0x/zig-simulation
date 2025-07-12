const std = @import("std");
const protocol = @import("src/communication/protocol.zig");

test "WebSocket frame encoding" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Test JSON text frame encoding
    const test_json = "{\"test\":\"message\"}";
    
    var buffer: [1024]u8 = undefined;
    var stream = std.io.fixedBufferStream(&buffer);
    
    try protocol.MessageEncoder.sendJsonMessage(test_json, stream.writer());
    
    const frame_data = buffer[0..stream.pos];
    std.log.info("Encoded frame size: {}", .{frame_data.len});
    std.log.info("Frame bytes: {any}", .{frame_data[0..@min(20, frame_data.len)]});
    
    // Verify frame structure
    // First byte should be 0x81 (FIN + TEXT frame)
    try std.testing.expect(frame_data[0] == 0x81);
    
    // Second byte should be payload length (test_json.len = 17, so 0x11)
    try std.testing.expect(frame_data[1] == test_json.len);
    
    // Payload should match our JSON
    const payload = frame_data[2..2 + test_json.len];
    try std.testing.expectEqualStrings(test_json, payload);
    
    std.log.info("✅ WebSocket text frame encoding test passed");
}

test "WebSocket frame decoding" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    
    // Create a test frame manually: 0x81 (FIN + TEXT) + 0x05 (length 5) + "hello"
    const test_frame = [_]u8{ 0x81, 0x05, 'h', 'e', 'l', 'l', 'o' };
    
    var stream = std.io.fixedBufferStream(&test_frame);
    var decode_buffer: [1024]u8 = undefined;
    
    const decoded_frame = try protocol.WebSocketFrame.decode(stream.reader(), &decode_buffer);
    
    try std.testing.expect(decoded_frame.fin == true);
    try std.testing.expect(decoded_frame.opcode == protocol.WebSocketFrame.OPCODE_TEXT);
    try std.testing.expectEqualStrings("hello", decoded_frame.payload);
    
    std.log.info("✅ WebSocket frame decoding test passed");
}

test "Full WebSocket message round-trip" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    
    const original_message = "{\"status\":\"connected\",\"timestamp\":123456}";
    
    // Encode
    var encode_buffer: [1024]u8 = undefined;
    var encode_stream = std.io.fixedBufferStream(&encode_buffer);
    try protocol.MessageEncoder.sendJsonMessage(original_message, encode_stream.writer());
    
    // Decode
    var decode_stream = std.io.fixedBufferStream(encode_buffer[0..encode_stream.pos]);
    var decode_buffer: [1024]u8 = undefined;
    const decoded_frame = try protocol.WebSocketFrame.decode(decode_stream.reader(), &decode_buffer);
    
    // Verify
    try std.testing.expectEqualStrings(original_message, decoded_frame.payload);
    try std.testing.expect(decoded_frame.opcode == protocol.WebSocketFrame.OPCODE_TEXT);
    
    std.log.info("✅ Full WebSocket round-trip test passed");
    std.log.info("Original: {s}", .{original_message});
    std.log.info("Decoded:  {s}", .{decoded_frame.payload});
}

test "Binary frame encoding" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    
    const test_binary = [_]u8{ 0x01, 0x02, 0x03, 0x04, 0xFF };
    
    var buffer: [1024]u8 = undefined;
    var stream = std.io.fixedBufferStream(&buffer);
    
    try protocol.MessageEncoder.sendBinaryMessage(&test_binary, stream.writer());
    
    const frame_data = buffer[0..stream.pos];
    
    // First byte should be 0x82 (FIN + BINARY frame)
    try std.testing.expect(frame_data[0] == 0x82);
    
    // Second byte should be payload length (5)
    try std.testing.expect(frame_data[1] == test_binary.len);
    
    // Payload should match our binary data
    const payload = frame_data[2..2 + test_binary.len];
    try std.testing.expectEqualSlices(u8, &test_binary, payload);
    
    std.log.info("✅ WebSocket binary frame encoding test passed");
}