const std = @import("std");
const Server = std.net.Server;
const Address = std.net.Address;
const core = @import("core");
const joints = @import("joints");
const protocol = @import("protocol.zig");
const Thread = std.Thread;
const time = std.time;
const crypto = std.crypto;

const UpdateThread = struct {
    thread: Thread,
    running: bool,
    clients: *std.ArrayList(*Client),
    allocator: std.mem.Allocator,
    mutex: Thread.Mutex,
    command_queue: std.ArrayList(protocol.CommandMessage),
    joint_manager: *joints.JointManager,

    pub fn init(allocator: std.mem.Allocator, clients: *std.ArrayList(*Client), joint_manager: *joints.JointManager) !*UpdateThread {
        const self = try allocator.create(UpdateThread);
        self.* = .{
            .thread = undefined,
            .running = false,
            .clients = clients,
            .allocator = allocator,
            .mutex = Thread.Mutex{},
            .command_queue = std.ArrayList(protocol.CommandMessage).init(allocator),
            .joint_manager = joint_manager,
        };
        return self;
    }

    pub fn start(self: *UpdateThread) !void {
        self.running = true;
        self.thread = try Thread.spawn(.{}, run, .{self});
    }

    pub fn stop(self: *UpdateThread) void {
        self.running = false;
        self.thread.join();
        self.command_queue.deinit();
    }
    
    pub fn deinit(self: *UpdateThread) void {
        self.stop();
        self.allocator.destroy(self);
    }

    fn run(self: *UpdateThread) !void {
        var timer = try time.Timer.start();
        var last_status_update: u64 = 0;
        // var last_collision_check: u64 = 0; // Temporarily unused for debugging
        const update_interval_ns = time.ns_per_ms; // 1kHz
        const status_interval_ns = 10 * time.ns_per_ms; // 100Hz
        // const collision_interval_ns = 10 * time.ns_per_ms; // 100Hz // Temporarily unused for debugging

        while (self.running) {
            const now = timer.read();

            // TEMPORARILY DISABLED: Joint state updates (1kHz) for debugging
            // try self.sendJointStates();

            // System status updates (100Hz) - reduced frequency for debugging
            if (now - last_status_update >= status_interval_ns * 10) { // 10Hz instead of 100Hz
                try self.sendSystemStatus();
                last_status_update = now;
            }

            // TEMPORARILY DISABLED: Collision detection updates for debugging
            // if (now - last_collision_check >= collision_interval_ns) {
            //     try self.checkAndSendCollisions();
            //     last_collision_check = now;
            // }

            // Process any pending commands
            try self.processCommands();

            // Sleep until next update
            const elapsed = timer.read() - now;
            if (elapsed < update_interval_ns) {
                time.sleep(update_interval_ns - elapsed);
            }
        }
    }

    fn sendJointStates(self: *UpdateThread) !void {
        // Get current joint states
        var positions: [core.types.NUM_JOINTS]f32 = undefined;
        var velocities: [core.types.NUM_JOINTS]f32 = undefined;
        var torques: [core.types.NUM_JOINTS]f32 = undefined;
        var temperatures: [core.types.NUM_JOINTS]f32 = undefined;
        var currents: [core.types.NUM_JOINTS]f32 = undefined;

        for (0..core.types.NUM_JOINTS) |i| {
            const state = self.joint_manager.getJointState(@enumFromInt(i));
            positions[i] = state.current_angle;
            velocities[i] = state.current_velocity;
            torques[i] = state.current_torque;
            temperatures[i] = state.temperature;
            currents[i] = state.current;
        }

        const state = protocol.JointStateMessage{
            .timestamp_us = @intCast(time.microTimestamp()),
            .positions = positions,
            .velocities = velocities,
            .torques = torques,
            .temperatures = temperatures,
            .currents = currents,
        };

        var bytes: [@sizeOf(protocol.JointStateMessage)]u8 = undefined;
        @memset(&bytes, 0);
        @memcpy(&bytes, std.mem.asBytes(&state));

        self.mutex.lock();
        defer self.mutex.unlock();

        for (self.clients.items) |client| {
            var stream = std.io.fixedBufferStream(&client.write_buffer);
            try protocol.MessageEncoder.sendBinaryMessage(&bytes, stream.writer());
            _ = client.connection.stream.write(client.write_buffer[0..stream.pos]) catch |err| {
                std.log.warn("Failed to send joint states to client: {}", .{err});
            };
        }
    }

    fn sendSystemStatus(self: *UpdateThread) !void {
        const robot_state = self.joint_manager.getRobotState();
        const collision_result = self.joint_manager.safety_monitor.collision_detector.getCurrentResult();
        
        const safety_status_placeholder = protocol.SafetyStatus {
            .soft_limits_active = true, // Placeholder
            .emergency_stop = false, // Placeholder
            .collision_detected = collision_result.collision_detected, 
        };
        
        const status = protocol.SystemStatusMessage{
            .state = switch (robot_state) {
                .ready => protocol.SYSTEM_STATE_READY,
                .moving => protocol.SYSTEM_STATE_BUSY,
                .fault => protocol.SYSTEM_STATE_ERROR,
                .emergency_stop => protocol.SYSTEM_STATE_ERROR,
                .powered_off => protocol.SYSTEM_STATE_WARNING,
            },
            .error_code = if (robot_state == .fault) "FAULT" else null,
            .safety_status = safety_status_placeholder, // Use placeholder
            .control_mode = switch (self.joint_manager.control_mode) { // Use actual control mode
                .position => protocol.CONTROL_MODE_POSITION,
                .velocity => protocol.CONTROL_MODE_VELOCITY,
                .torque => protocol.CONTROL_MODE_TORQUE,
            },
        };

        const json_string = try std.json.stringifyAlloc(self.allocator, status, .{});
        defer self.allocator.free(json_string);

        self.mutex.lock();
        defer self.mutex.unlock();

        for (self.clients.items) |client| {
            // Reset buffer and create fresh stream
            @memset(&client.write_buffer, 0);
            var stream = std.io.fixedBufferStream(&client.write_buffer);
            
            try protocol.MessageEncoder.sendJsonMessage(json_string, stream.writer());
            _ = client.connection.stream.write(client.write_buffer[0..stream.pos]) catch |err| {
                std.log.warn("Failed to send system status to client: {}", .{err});
            };
        }
    }

    fn processCommands(self: *UpdateThread) !void {
        self.mutex.lock();
        defer self.mutex.unlock();

        while (self.command_queue.items.len > 0) {
            const cmd = self.command_queue.orderedRemove(0);
            switch (cmd.type) {
                protocol.COMMAND_TYPE_POSITION => {
                    if (cmd.values) |positions| {
                        // Check length
                        if (positions.len != core.types.NUM_JOINTS) {
                            std.log.err("Position command length mismatch: expected {}, got {}", .{core.types.NUM_JOINTS, positions.len});
                            return error.InvalidCommandPayload;
                        }
                        var velocities: [core.types.NUM_JOINTS]f32 = [_]f32{0} ** core.types.NUM_JOINTS; // Use NUM_JOINTS
                        if (cmd.max_velocity) |max_vels| {
                            // Optional: Add stricter checking for max_vels length too
                            if (max_vels.len >= core.types.NUM_JOINTS) {
                                @memcpy(&velocities, max_vels[0..core.types.NUM_JOINTS]); 
                            }
                        }
                        // Length is now guaranteed to match
                        for (0..core.types.NUM_JOINTS) |i| {
                           self.joint_manager.joints[i].target_angle = positions[i];
                           self.joint_manager.joints[i].target_velocity = velocities[i];
                        }
                    } else {
                        std.log.err("Position command missing values field", .{});
                        return error.InvalidCommandPayload;
                    }
                },
                protocol.COMMAND_TYPE_VELOCITY => {
                    if (cmd.values) |velocities| {
                        // Check length
                        if (velocities.len != core.types.NUM_JOINTS) {
                            std.log.err("Velocity command length mismatch: expected {}, got {}", .{core.types.NUM_JOINTS, velocities.len});
                            return error.InvalidCommandPayload;
                        }
                        // Length is now guaranteed to match
                        for (0..core.types.NUM_JOINTS) |i| {
                            self.joint_manager.joints[i].target_velocity = velocities[i];
                        }
                    } else {
                        std.log.err("Velocity command missing values field", .{});
                        return error.InvalidCommandPayload;
                    }
                },
                protocol.COMMAND_TYPE_TORQUE => {
                    if (cmd.values) |torques| {
                        // Length check is handled within setTorques, which returns InvalidNumberOfTorques
                        try self.joint_manager.setTorques(torques);
                    } else {
                        std.log.err("Torque command missing values field", .{});
                        return error.InvalidCommandPayload;
                    }
                },
                protocol.COMMAND_TYPE_CONTROL_MODE => {
                    if (cmd.control_mode) |mode_value| {
                        const params = cmd.parameters orelse protocol.ControlParameters{};
                        try self.joint_manager.setControlMode(switch (mode_value) {
                            protocol.CONTROL_MODE_POSITION => .position,
                            protocol.CONTROL_MODE_VELOCITY => .velocity,
                            protocol.CONTROL_MODE_TORQUE => .torque,
                            else => return error.InvalidControlMode, // Use defined error
                        }, .{
                            .stiffness = params.stiffness orelse 1.0,
                            .damping = params.damping orelse 0.1,
                            .feedforward = params.feedforward,
                        });
                    } else {
                        std.log.err("Received CONTROL_MODE command without control_mode value", .{});
                        return error.MissingControlModeValue; // Use defined error
                    }
                },
                protocol.COMMAND_TYPE_SAFETY => {
                    if (cmd.safety) |safety_cmd| {
                        switch (safety_cmd.type) {
                            protocol.SAFETY_CMD_ENABLE => self.joint_manager.safety_monitor.enable(),
                            protocol.SAFETY_CMD_DISABLE => self.joint_manager.safety_monitor.disable(),
                            protocol.SAFETY_CMD_RESET => self.joint_manager.safety_monitor.reset(),
                            protocol.SAFETY_CMD_E_STOP => self.joint_manager.safety_monitor.emergencyStop(),
                            else => return error.InvalidSafetyCommand, // Use defined error
                        }
                    } else {
                         std.log.err("Safety command missing safety field", .{});
                        return error.InvalidCommandPayload;
                    }
                },
                else => {},
            }
        }
    }

    fn checkAndSendCollisions(self: *UpdateThread) !void {
        const collision_result = self.joint_manager.safety_monitor.collision_detector.getCurrentResult();
        if (collision_result.collision_detected) {
            const link1_name: []const u8 = if (collision_result.link1) |l1| @tagName(l1) else "unknown";
            const link2_name: []const u8 = if (collision_result.link2) |l2| @tagName(l2) else "unknown";

            const collision_msg = protocol.CollisionMessage{
                .detected = collision_result.collision_detected,
                .link1 = link1_name,
                .link2 = link2_name,
                .position = if (collision_result.collision_point) |p| [_]f32{p.x, p.y, p.z} else [_]f32{ 0, 0, 0 },
                .penetration_depth = collision_result.min_distance,
                .contact_normal = [_]f32{ 0, 0, 0 },
            };

            const json_string = try std.json.stringifyAlloc(self.allocator, collision_msg, .{});
            defer self.allocator.free(json_string);

            self.mutex.lock();
            defer self.mutex.unlock();

            for (self.clients.items) |client| {
                // Reset buffer and create fresh stream
                @memset(&client.write_buffer, 0);
                var stream = std.io.fixedBufferStream(&client.write_buffer);
                
                try protocol.MessageEncoder.sendJsonMessage(json_string, stream.writer());
                _ = client.connection.stream.write(client.write_buffer[0..stream.pos]) catch |err| {
                    std.log.warn("Failed to send collision data to client: {}", .{err});
                };
            }
        }
    }
};

pub const WebSocketServer = struct {
    allocator: std.mem.Allocator,
    server: Server,
    clients: std.ArrayList(*Client),
    update_thread: *UpdateThread,
    running: bool,
    thread: Thread,
    mutex: Thread.Mutex,

    pub fn init(allocator: std.mem.Allocator, port: u16, joint_manager: *joints.JointManager) !*WebSocketServer {
        const self = try allocator.create(WebSocketServer);
        const listen_address = try Address.parseIp4("127.0.0.1", port);
        const actual_server = try listen_address.listen(.{ .reuse_address = true });

        self.* = .{
            .allocator = allocator,
            .server = actual_server,
            .clients = std.ArrayList(*Client).init(allocator),
            .update_thread = try UpdateThread.init(allocator, &self.clients, joint_manager),
            .running = false,
            .thread = undefined,
            .mutex = Thread.Mutex{},
        };

        return self;
    }

    pub fn start(self: *WebSocketServer) !void {
        self.running = true;
        try self.update_thread.start();
        self.thread = try Thread.spawn(.{}, acceptLoop, .{self});
    }

    pub fn stop(self: *WebSocketServer) void {
        self.running = false;
        // Note: Server doesn't have close() method in this Zig version
        self.thread.join();
        self.update_thread.deinit(); // Properly cleanup UpdateThread
        
        // Clean up all client connections (arena cleanup automatic)
        for (self.clients.items) |client| {
            client.deinit(); // Arena cleanup happens here, including main_allocator.destroy(self)
        }
        self.clients.deinit();
        self.allocator.destroy(self);
    }

    fn acceptLoop(self: *WebSocketServer) !void {
        while (self.running) {
            std.log.info("Waiting for WebSocket connection...", .{});
            const connection = self.server.accept() catch |err| {
                if (err == error.ConnectionAborted) break;
                std.log.err("Failed to accept connection: {}", .{err});
                continue;
            };

            std.log.info("New connection accepted, creating client...", .{});
            const client = Client.init(self.allocator, connection) catch |err| {
                std.log.err("Failed to initialize client: {}", .{err});
                connection.stream.close();
                continue;
            };

            std.log.info("Performing WebSocket handshake...", .{});
            if (client.performHandshake()) |_| {
                std.log.info("Handshake successful, adding client to list...", .{});
                self.clients.append(client) catch |err| {
                    std.log.err("Failed to add client: {}", .{err});
                    client.deinit(); // Arena cleanup handles main_allocator.destroy(client)
                    continue;
                };
                std.log.info("Spawning client handler thread...", .{});
                _ = try Thread.spawn(.{}, handleClient, .{ self, client });
                std.log.info("Client connected successfully!", .{});
            } else |err| {
                std.log.err("WebSocket handshake failed: {}", .{err});
                client.deinit(); // Arena cleanup handles main_allocator.destroy(client)
            }
        }
    }

    fn handleClient(self: *WebSocketServer, client: *Client) !void {
        std.log.info("=== Starting client handler for new connection ===", .{});
        
        // Send connection status to all clients  
        std.log.info("Preparing connection status message...", .{});
        const current_timestamp_val = time.microTimestamp();
        const status = protocol.ConnectionStatusMessage{
            .status = "connected",
            .message = "New client connected",
            .timestamp_us = @intCast(current_timestamp_val),
        };
        const json_string = try std.json.stringifyAlloc(self.allocator, status, .{});
        defer self.allocator.free(json_string);
        std.log.info("Connection status JSON: {s}", .{json_string});

        self.mutex.lock();
        defer self.mutex.unlock();

        std.log.info("Sending connection status to {} clients...", .{self.clients.items.len});
        for (self.clients.items) |c| {
            // Reset buffer and create fresh stream
            @memset(&c.write_buffer, 0);
            var stream = std.io.fixedBufferStream(&c.write_buffer);
            
            std.log.info("Encoding JSON message: {s}", .{json_string});
            try protocol.MessageEncoder.sendJsonMessage(json_string, stream.writer());
            
            const frame_size = stream.pos;
            std.log.info("Encoded WebSocket frame size: {} bytes", .{frame_size});
            
            // Log first few bytes of frame for debugging
            const debug_bytes = c.write_buffer[0..@min(16, frame_size)];
            var hex_string: [32]u8 = undefined;
            for (debug_bytes, 0..) |byte, i| {
                _ = std.fmt.bufPrint(hex_string[i*2..i*2+2], "{X:0>2}", .{byte}) catch break;
            }
            std.log.info("Frame start (hex): {s}", .{hex_string[0..debug_bytes.len*2]});
            
            const bytes_written = c.connection.stream.write(c.write_buffer[0..frame_size]) catch |err| {
                std.log.warn("Failed to send connection status to client: {}", .{err});
                continue;
            };
            std.log.info("Successfully sent {} bytes to client", .{bytes_written});
        }
        
        std.log.info("=== Starting message reading loop for client ===", .{});

        // Ensure cleanup happens when function exits
        defer {
            // Remove client from list
            for (self.clients.items, 0..) |c, i| {
                if (c == client) {
                    _ = self.clients.orderedRemove(i);
                    break;
                }
            }
            client.deinit(); // Arena cleanup handles main_allocator.destroy(client)

            // Try to send disconnection status
            const current_timestamp_val_disconnect = time.microTimestamp();
            if (std.json.stringifyAlloc(self.allocator, protocol.ConnectionStatusMessage{
                .status = "disconnected",
                .message = "Client disconnected",
                .timestamp_us = @intCast(current_timestamp_val_disconnect),
            }, .{})) |disconnect_json| {
                defer self.allocator.free(disconnect_json);

                for (self.clients.items) |c| {
                    // Reset buffer and create fresh stream
                    @memset(&c.write_buffer, 0);
                    var stream = std.io.fixedBufferStream(&c.write_buffer);
                    
                    if (protocol.MessageEncoder.sendJsonMessage(disconnect_json, stream.writer())) |_| {
                        _ = c.connection.stream.write(c.write_buffer[0..stream.pos]) catch {};
                    } else |_| {}
                }
            } else |_| {}
        }

        var frame_buffer: [4096]u8 = undefined;
        while (true) {
            std.log.info("Waiting for WebSocket frame from client...", .{});
            const ws_frame = protocol.WebSocketFrame.decode(client.connection.stream.reader(), &frame_buffer) catch |err| {
                if (err != error.EndOfStream) {
                    std.log.err("Error reading WebSocket frame: {}", .{err});
                } else {
                    std.log.info("Client disconnected (EndOfStream)", .{});
                }
                break;
            };

            std.log.info("Received WebSocket frame - opcode: {}, payload size: {}", .{ws_frame.opcode, ws_frame.payload.len});
            
            // Handle different WebSocket frame types
            switch (ws_frame.opcode) {
                protocol.WebSocketFrame.OPCODE_TEXT => {
                    // Text frame should contain JSON command
                    const parsed_cmd = std.json.parseFromSlice(protocol.CommandMessage, client.client_allocator, ws_frame.payload, .{}) catch |err| {
                        std.log.err("Failed to parse JSON command: {}", .{err});
                        continue;
                    };
                    // No defer needed - arena cleanup handles it automatically on client disconnect
                    std.log.info("Received command of type: {}", .{parsed_cmd.value.type});
                    try self.update_thread.command_queue.append(parsed_cmd.value);
                },
                protocol.WebSocketFrame.OPCODE_CLOSE => {
                    std.log.info("Client sent close frame, disconnecting...", .{});
                    break;
                },
                protocol.WebSocketFrame.OPCODE_PING => {
                    // Respond with pong
                    var stream = std.io.fixedBufferStream(&client.write_buffer);
                    try protocol.WebSocketFrame.encodePong(ws_frame.payload, stream.writer());
                    _ = client.connection.stream.write(client.write_buffer[0..stream.pos]) catch {};
                },
                else => {
                    std.log.warn("Received unsupported WebSocket frame type: {}", .{ws_frame.opcode});
                },
            }
        }
    }
};

const Client = struct {
    main_allocator: std.mem.Allocator,  // For client list management
    arena: std.heap.ArenaAllocator,     // For all client data
    client_allocator: std.mem.Allocator, // Convenience wrapper for arena.allocator()
    connection: Server.Connection,
    write_buffer: [8192]u8,
    read_buffer: [8192]u8,

    pub fn init(main_allocator: std.mem.Allocator, connection: Server.Connection) !*Client {
        const self = try main_allocator.create(Client);
        errdefer main_allocator.destroy(self);
        
        var arena = std.heap.ArenaAllocator.init(main_allocator);
        errdefer arena.deinit();
        
        self.* = .{
            .main_allocator = main_allocator,
            .arena = arena,
            .client_allocator = arena.allocator(),
            .connection = connection,
            .write_buffer = undefined,
            .read_buffer = undefined,
        };
        return self;
    }

    pub fn deinit(self: *Client) void {
        self.connection.stream.close();
        self.arena.deinit(); // Cleans up ALL client memory at once
        self.main_allocator.destroy(self);
    }

    pub fn performHandshake(self: *Client) !void {
        std.log.info("Reading handshake request...", .{});
        var buffer: [1024]u8 = undefined;
        const bytes_read = try self.connection.stream.read(&buffer);
        const request = buffer[0..bytes_read];
        std.log.info("Read {} bytes from client", .{bytes_read});

        // Parse HTTP request and verify it's a WebSocket upgrade request
        std.log.info("Validating WebSocket upgrade request...", .{});
        if (!std.mem.startsWith(u8, request, "GET")) return error.InvalidRequest;
        if (!std.mem.containsAtLeast(u8, request, 1, "Upgrade: websocket")) return error.NotWebSocket;

        // Extract the Sec-WebSocket-Key header
        std.log.info("Extracting WebSocket key...", .{});
        var key_start = std.mem.indexOf(u8, request, "Sec-WebSocket-Key: ") orelse return error.NoKey;
        key_start += "Sec-WebSocket-Key: ".len;
        const key_end = std.mem.indexOfPos(u8, request, key_start, "\r\n") orelse return error.InvalidKey;
        const client_key = request[key_start..key_end];
        std.log.info("Found client key: {s}", .{client_key});

        // Generate the accept key
        std.log.info("Generating accept key with SHA1...", .{});
        const magic_string = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        
        // Use stack buffer for concatenation (WebSocket key is max 24 chars + magic 36 chars = 60 max)
        var combined_buffer: [128]u8 = undefined;
        const combined_key = std.fmt.bufPrint(&combined_buffer, "{s}{s}", .{ client_key, magic_string }) catch return error.KeyTooLong;
        std.log.info("Combined key length: {}", .{combined_key.len});
        
        // Use SHA1 hash
        var accept_key: [20]u8 = undefined;
        crypto.hash.Sha1.hash(combined_key, &accept_key, .{});
        std.log.info("SHA1 hash completed", .{});

        // Encode the accept key in base64
        var accept_key_base64: [28]u8 = undefined;
        _ = std.base64.standard.Encoder.encode(&accept_key_base64, &accept_key);

        // Send the WebSocket handshake response using stack buffer
        var response_buffer: [512]u8 = undefined;
        const response = try std.fmt.bufPrint(&response_buffer,
            \\HTTP/1.1 101 Switching Protocols\r\n
            \\Upgrade: websocket\r\n
            \\Connection: Upgrade\r\n
            \\Sec-WebSocket-Accept: {s}\r\n
            \\\r\n
        , .{accept_key_base64});
        std.log.info("Sending handshake response ({} bytes)", .{response.len});

        _ = try self.connection.stream.write(response);
        std.log.info("Handshake response sent successfully", .{});
    }

    pub fn readFrame(self: *Client, buffer: []u8) !protocol.Frame {
        // Read frame header
        var header: [2]u8 = undefined;
        _ = try self.connection.stream.read(&header);

        const opcode = header[0] & 0x0F;
        const masked = (header[1] & 0x80) != 0;
        const payload_len = header[1] & 0x7F;

        // Read extended payload length if needed
        var extended_len: u64 = 0;
        if (payload_len == 126) {
            var len_bytes: [2]u8 = undefined;
            _ = try self.connection.stream.read(&len_bytes);
            extended_len = @as(u64, len_bytes[0]) << 8 | len_bytes[1];
        } else if (payload_len == 127) {
            var len_bytes: [8]u8 = undefined;
            _ = try self.connection.stream.read(&len_bytes);
            extended_len = @as(u64, len_bytes[0]) << 56 |
                          @as(u64, len_bytes[1]) << 48 |
                          @as(u64, len_bytes[2]) << 40 |
                          @as(u64, len_bytes[3]) << 32 |
                          @as(u64, len_bytes[4]) << 24 |
                          @as(u64, len_bytes[5]) << 16 |
                          @as(u64, len_bytes[6]) << 8 |
                          len_bytes[7];
        }

        const final_len = if (payload_len < 126) payload_len else extended_len;

        // Read masking key if frame is masked
        var mask: [4]u8 = undefined;
        if (masked) {
            _ = try self.connection.stream.read(&mask);
        }

        // Read payload
        const payload = buffer[0..final_len];
        _ = try self.connection.stream.read(payload);

        // Unmask payload if needed
        if (masked) {
            for (payload, 0..) |*byte, i| {
                byte.* ^= mask[i % 4];
            }
        }

        return protocol.Frame{
            .type = opcode,
            .payload = payload,
        };
    }
};

pub const Communication = struct {
    allocator: std.mem.Allocator,
    server: *WebSocketServer,
    port: u16,

    pub fn init(allocator: std.mem.Allocator) !Communication {
        return Communication{
            .allocator = allocator,
            .server = undefined,
            .port = 9001,
        };
    }

    pub fn start(self: *Communication, joint_manager: *joints.JointManager) !void {
        self.server = try WebSocketServer.init(self.allocator, self.port, joint_manager);
        try self.server.start();
        std.log.info("WebSocket server started on port {d}", .{self.port});
    }

    pub fn stop(self: *Communication) void {
        if (self.server != undefined) {
            self.server.stop();
        }
    }

    pub fn deinit(self: *Communication) void {
        self.stop();
    }
};

pub fn init() core.Result {
    return core.Result{ .success = {} };
} 