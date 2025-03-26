const std = @import("std");
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

    fn run(self: *UpdateThread) !void {
        var timer = try time.Timer.start();
        var last_status_update: u64 = 0;
        var last_collision_check: u64 = 0;
        const update_interval_ns = time.ns_per_ms; // 1kHz
        const status_interval_ns = 10 * time.ns_per_ms; // 100Hz
        const collision_interval_ns = 10 * time.ns_per_ms; // 100Hz

        while (self.running) {
            const now = timer.read();

            // Joint state updates (1kHz)
            try self.sendJointStates();

            // System status updates (100Hz)
            if (now - last_status_update >= status_interval_ns) {
                try self.sendSystemStatus();
                last_status_update = now;
            }

            // Collision detection updates (100Hz)
            if (now - last_collision_check >= collision_interval_ns) {
                try self.checkAndSendCollisions();
                last_collision_check = now;
            }

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
        var positions: [6]f32 = undefined;
        var velocities: [6]f32 = undefined;
        var torques: [6]f32 = undefined;
        var temperatures: [6]f32 = undefined;
        var currents: [6]f32 = undefined;

        for (0..6) |i| {
            const state = self.joint_manager.getJointState(@enumFromInt(i));
            positions[i] = state.current_angle;
            velocities[i] = state.current_velocity;
            torques[i] = state.current_torque;
            temperatures[i] = state.temperature;
            currents[i] = state.current;
        }

        const state = protocol.JointStateMessage{
            .timestamp_us = @as(u64, time.microTimestamp()),
            .positions = positions,
            .velocities = velocities,
            .torques = torques,
            .temperatures = temperatures,
            .currents = currents,
        };

        var bytes: [@sizeOf(protocol.JointStateMessage)]u8 = undefined;
        @memset(&bytes, 0);
        @memcpy(&bytes, std.mem.asBytes(&state));

        const frame = protocol.Frame{
            .type = protocol.MESSAGE_TYPE_JOINT_STATE,
            .payload = &bytes,
        };

        self.mutex.lock();
        defer self.mutex.unlock();

        for (self.clients.items) |client| {
            var stream = std.io.fixedBufferStream(&client.write_buffer);
            try frame.encode(stream.writer());
            _ = try client.connection.stream.write(client.write_buffer[0..stream.pos]);
        }
    }

    fn sendSystemStatus(self: *UpdateThread) !void {
        const robot_state = self.joint_manager.getRobotState();
        const safety_status = self.joint_manager.safety.getStatus();
        
        const status = protocol.SystemStatusMessage{
            .state = switch (robot_state) {
                .ready => protocol.SYSTEM_STATE_READY,
                .moving => protocol.SYSTEM_STATE_BUSY,
                .fault => protocol.SYSTEM_STATE_ERROR,
                .emergency_stop => protocol.SYSTEM_STATE_ERROR,
                .powered_off => protocol.SYSTEM_STATE_WARNING,
            },
            .error_code = if (robot_state == .fault) "FAULT" else null,
            .safety_status = .{
                .soft_limits_active = safety_status.soft_limits_active,
                .emergency_stop = safety_status.emergency_stop,
                .collision_detected = safety_status.collision_detected,
            },
            .control_mode = protocol.CONTROL_MODE_POSITION, // TODO: Get from joint manager
        };

        const json_string = try std.json.stringifyAlloc(self.allocator, status, .{});
        defer self.allocator.free(json_string);

        const frame = protocol.Frame{
            .type = protocol.MESSAGE_TYPE_SYSTEM_STATUS,
            .payload = json_string,
        };

        self.mutex.lock();
        defer self.mutex.unlock();

        for (self.clients.items) |client| {
            var stream = std.io.fixedBufferStream(&client.write_buffer);
            try frame.encode(stream.writer());
            _ = try client.connection.stream.write(client.write_buffer[0..stream.pos]);
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
                        var velocities = [_]f32{0} ** 6;
                        if (cmd.max_velocity) |max_vels| {
                            @memcpy(&velocities, max_vels);
                        }
                        try self.joint_manager.setTargets(positions, velocities);
                    }
                },
                protocol.COMMAND_TYPE_VELOCITY => {
                    if (cmd.values) |velocities| {
                        try self.joint_manager.setVelocities(velocities);
                    }
                },
                protocol.COMMAND_TYPE_TORQUE => {
                    if (cmd.values) |torques| {
                        try self.joint_manager.setTorques(torques);
                    }
                },
                protocol.COMMAND_TYPE_CONTROL_MODE => {
                    if (cmd.parameters) |params| {
                        try self.joint_manager.setControlMode(switch (cmd.values.?[0]) {
                            protocol.CONTROL_MODE_POSITION => .position,
                            protocol.CONTROL_MODE_VELOCITY => .velocity,
                            protocol.CONTROL_MODE_TORQUE => .torque,
                            else => return error.InvalidControlMode,
                        }, .{
                            .stiffness = params.stiffness orelse 1.0,
                            .damping = params.damping orelse 0.1,
                            .feedforward = params.feedforward,
                        });
                    }
                },
                protocol.COMMAND_TYPE_SAFETY => {
                    if (cmd.safety) |safety_cmd| {
                        switch (safety_cmd.type) {
                            protocol.SAFETY_CMD_ENABLE => try self.joint_manager.safety.enable(),
                            protocol.SAFETY_CMD_DISABLE => try self.joint_manager.safety.disable(),
                            protocol.SAFETY_CMD_RESET => try self.joint_manager.safety.reset(),
                            protocol.SAFETY_CMD_E_STOP => try self.joint_manager.safety.emergencyStop(),
                            else => return error.InvalidSafetyCommand,
                        }
                    }
                },
                else => {},
            }
        }
    }

    fn checkAndSendCollisions(self: *UpdateThread) !void {
        const collision_result = self.joint_manager.collision_detector.getLastCollision();
        if (collision_result.detected) {
            const collision_msg = protocol.CollisionMessage{
                .detected = true,
                .link1 = collision_result.link1_name,
                .link2 = collision_result.link2_name,
                .position = collision_result.position,
                .penetration_depth = collision_result.penetration_depth,
                .contact_normal = collision_result.contact_normal,
            };

            const json_string = try std.json.stringifyAlloc(self.allocator, collision_msg, .{});
            defer self.allocator.free(json_string);

            const frame = protocol.Frame{
                .type = protocol.MESSAGE_TYPE_COLLISION_DATA,
                .payload = json_string,
            };

            self.mutex.lock();
            defer self.mutex.unlock();

            for (self.clients.items) |client| {
                var stream = std.io.fixedBufferStream(&client.write_buffer);
                try frame.encode(stream.writer());
                _ = try client.connection.stream.write(client.write_buffer[0..stream.pos]);
            }
        }
    }
};

const WebSocketServer = struct {
    allocator: std.mem.Allocator,
    server: std.net.StreamServer,
    clients: std.ArrayList(*Client),
    update_thread: *UpdateThread,
    running: bool,
    thread: Thread,
    mutex: Thread.Mutex,

    pub fn init(allocator: std.mem.Allocator, port: u16, joint_manager: *joints.JointManager) !*WebSocketServer {
        const self = try allocator.create(WebSocketServer);
        self.* = .{
            .allocator = allocator,
            .server = std.net.StreamServer.init(.{ .reuse_address = true }),
            .clients = std.ArrayList(*Client).init(allocator),
            .update_thread = try UpdateThread.init(allocator, &self.clients, joint_manager),
            .running = false,
            .thread = undefined,
            .mutex = Thread.Mutex{},
        };

        try self.server.listen(std.net.Address.parseIp4("127.0.0.1", port) catch return error.InvalidAddress);
        return self;
    }

    pub fn start(self: *WebSocketServer) !void {
        self.running = true;
        try self.update_thread.start();
        self.thread = try Thread.spawn(.{}, acceptLoop, .{self});
    }

    pub fn stop(self: *WebSocketServer) void {
        self.running = false;
        self.server.close();
        self.thread.join();
        self.update_thread.stop();
        
        for (self.clients.items) |client| {
            client.deinit();
            self.allocator.destroy(client);
        }
        self.clients.deinit();
        self.allocator.destroy(self);
    }

    fn acceptLoop(self: *WebSocketServer) !void {
        while (self.running) {
            const connection = self.server.accept() catch |err| {
                if (err == error.ConnectionAborted) break;
                std.log.err("Failed to accept connection: {}", .{err});
                continue;
            };

            const client = Client.init(self.allocator, connection) catch |err| {
                std.log.err("Failed to initialize client: {}", .{err});
                connection.stream.close();
                continue;
            };

            if (client.performHandshake()) |_| {
                self.clients.append(client) catch |err| {
                    std.log.err("Failed to add client: {}", .{err});
                    client.deinit();
                    self.allocator.destroy(client);
                    continue;
                };
                try Thread.spawn(.{}, handleClient, .{ self, client });
            } else |err| {
                std.log.err("WebSocket handshake failed: {}", .{err});
                client.deinit();
                self.allocator.destroy(client);
            }
        }
    }

    fn handleClient(self: *WebSocketServer, client: *Client) !void {
        // Send connection status to all clients
        const status = protocol.ConnectionStatusMessage{
            .status = protocol.CONNECTION_STATUS_CONNECTED,
            .message = "New client connected",
            .timestamp_us = @as(u64, time.microTimestamp()),
        };
        const json_string = try std.json.stringifyAlloc(self.allocator, status, .{});
        defer self.allocator.free(json_string);

        const connect_frame = protocol.Frame{
            .type = protocol.MESSAGE_TYPE_CONNECTION_STATUS,
            .payload = json_string,
        };

        self.mutex.lock();
        defer self.mutex.unlock();

        for (self.clients.items) |c| {
            var stream = std.io.fixedBufferStream(&c.write_buffer);
            try connect_frame.encode(stream.writer());
            _ = try c.connection.stream.write(c.write_buffer[0..stream.pos]);
        }

        // Ensure cleanup happens when function exits
        defer {
            // Remove client from list
            for (self.clients.items, 0..) |c, i| {
                if (c == client) {
                    _ = self.clients.orderedRemove(i);
                    break;
                }
            }
            client.deinit();
            self.allocator.destroy(client);

            // Try to send disconnection status
            if (std.json.stringifyAlloc(self.allocator, protocol.ConnectionStatusMessage{
                .status = protocol.CONNECTION_STATUS_DISCONNECTED,
                .message = "Client disconnected",
                .timestamp_us = @as(u64, time.microTimestamp()),
            }, .{})) |disconnect_json| {
                defer self.allocator.free(disconnect_json);

                const disconnect_frame = protocol.Frame{
                    .type = protocol.MESSAGE_TYPE_CONNECTION_STATUS,
                    .payload = disconnect_json,
                };

                for (self.clients.items) |c| {
                    var stream = std.io.fixedBufferStream(&c.write_buffer);
                    if (disconnect_frame.encode(stream.writer())) |_| {
                        _ = c.connection.stream.write(c.write_buffer[0..stream.pos]) catch {};
                    } else |_| {}
                }
            } else |_| {}
        }

        var frame_buffer: [4096]u8 = undefined;
        while (true) {
            const frame = client.readFrame(&frame_buffer) catch |err| {
                if (err != error.EndOfStream) {
                    std.log.err("Error reading frame: {}", .{err});
                }
                break;
            };

            if (frame.type == protocol.MESSAGE_TYPE_COMMAND) {
                const cmd = std.json.parse(protocol.CommandMessage, self.allocator, frame.payload, .{}) catch |err| {
                    std.log.err("Failed to parse command: {}", .{err});
                    continue;
                };
                try self.update_thread.command_queue.append(cmd);
            }
        }
    }
};

const Client = struct {
    allocator: std.mem.Allocator,
    connection: std.net.StreamServer.Connection,
    write_buffer: [8192]u8,
    read_buffer: [8192]u8,

    pub fn init(allocator: std.mem.Allocator, connection: std.net.StreamServer.Connection) !*Client {
        const self = try allocator.create(Client);
        self.* = .{
            .allocator = allocator,
            .connection = connection,
            .write_buffer = undefined,
            .read_buffer = undefined,
        };
        return self;
    }

    pub fn deinit(self: *Client) void {
        self.connection.stream.close();
    }

    pub fn performHandshake(self: *Client) !void {
        var buffer: [1024]u8 = undefined;
        const bytes_read = try self.connection.stream.read(&buffer);
        const request = buffer[0..bytes_read];

        // Parse HTTP request and verify it's a WebSocket upgrade request
        if (!std.mem.startsWith(u8, request, "GET")) return error.InvalidRequest;
        if (!std.mem.containsAtLeast(u8, request, 1, "Upgrade: websocket")) return error.NotWebSocket;

        // Extract the Sec-WebSocket-Key header
        var key_start = std.mem.indexOf(u8, request, "Sec-WebSocket-Key: ") orelse return error.NoKey;
        key_start += "Sec-WebSocket-Key: ".len;
        const key_end = std.mem.indexOfPos(u8, request, key_start, "\r\n") orelse return error.InvalidKey;
        const client_key = request[key_start..key_end];

        // Generate the accept key
        const magic_string = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        var hasher = crypto.hash.Sha1.init(.{});
        hasher.update(client_key);
        hasher.update(magic_string);
        var accept_key: [20]u8 = undefined;
        hasher.final(&accept_key);

        // Encode the accept key in base64
        var accept_key_base64: [28]u8 = undefined;
        _ = std.base64.standard.Encoder.encode(&accept_key_base64, &accept_key);

        // Send the WebSocket handshake response
        const response = try std.fmt.allocPrint(self.allocator,
            \\HTTP/1.1 101 Switching Protocols\r\n
            \\Upgrade: websocket\r\n
            \\Connection: Upgrade\r\n
            \\Sec-WebSocket-Accept: {s}\r\n
            \\\r\n
        , .{accept_key_base64});
        defer self.allocator.free(response);

        _ = try self.connection.stream.write(response);
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
            .port = 8080,
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