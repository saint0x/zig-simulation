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
            // TODO: Add real sensor data when hardware interface is ready
            torques[i] = 0;
            temperatures[i] = 25.0; // Room temperature for now
            currents[i] = 0;
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
    server: std.net.Server,
    allocator: std.mem.Allocator,
    port: u16,
    clients: std.ArrayList(*Client),
    running: bool,
    update_thread: *UpdateThread,
    joint_manager: *joints.JointManager,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, port: u16, joint_manager: *joints.JointManager) !*Self {
        const server = try std.net.Server.init(.{
            .address = std.net.Address.initIp4(.{ 0, 0, 0, 0 }, port),
            .reuse_address = true,
        });

        var clients = std.ArrayList(*Client).init(allocator);
        const update_thread = try UpdateThread.init(allocator, &clients, joint_manager);

        const self = try allocator.create(Self);
        self.* = .{
            .server = server,
            .allocator = allocator,
            .port = port,
            .clients = clients,
            .running = false,
            .update_thread = update_thread,
            .joint_manager = joint_manager,
        };
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.running = false;
        self.update_thread.stop();
        for (self.clients.items) |client| {
            client.deinit();
            self.allocator.destroy(client);
        }
        self.clients.deinit();
        self.server.deinit();
        self.allocator.destroy(self);
    }

    fn generateAcceptKey(key: []const u8) ![28]u8 {
        const magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        var hasher = crypto.hash.Sha1.init(.{});
        hasher.update(key);
        hasher.update(magic);
        var result: [20]u8 = undefined;
        hasher.final(&result);
        return std.base64.standard.Encoder.encode(result[0..]);
    }

    pub fn start(self: *Self) !void {
        std.log.info("WebSocket server starting on port {d}...", .{self.port});
        self.running = true;
        try self.update_thread.start();

        while (self.running) {
            const connection = try self.server.accept();
            
            // Handle HTTP upgrade to WebSocket
            var buf: [1024]u8 = undefined;
            const n = try connection.stream.read(&buf);
            
            // Parse headers to get Sec-WebSocket-Key
            var lines = std.mem.split(u8, buf[0..n], "\r\n");
            var key: ?[]const u8 = null;
            while (lines.next()) |line| {
                if (std.mem.startsWith(u8, line, "Sec-WebSocket-Key: ")) {
                    key = line["Sec-WebSocket-Key: ".len..];
                    break;
                }
            }

            if (key) |ws_key| {
                const accept_key = try generateAcceptKey(ws_key);
                
                // Very permissive CORS headers
                var response_buf: [512]u8 = undefined;
                const response = try std.fmt.bufPrint(&response_buf,
                    "HTTP/1.1 101 Switching Protocols\r\n" ++
                    "Upgrade: websocket\r\n" ++
                    "Connection: Upgrade\r\n" ++
                    "Access-Control-Allow-Origin: *\r\n" ++
                    "Access-Control-Allow-Methods: *\r\n" ++
                    "Access-Control-Allow-Headers: *\r\n" ++
                    "Sec-WebSocket-Accept: {s}\r\n" ++
                    "\r\n", .{accept_key});

                _ = try connection.stream.write(response);

                // Create and store client
                const client = try Client.init(self.allocator, connection);
                try self.clients.append(client);

                // Spawn client handler thread
                _ = try std.Thread.spawn(.{}, Client.handle, .{client});
            } else {
                // Invalid WebSocket request
                connection.stream.close();
            }
        }
    }
};

const Client = struct {
    allocator: std.mem.Allocator,
    connection: std.net.Server.Connection,
    running: bool,
    write_buffer: [4096]u8,
    read_buffer: [4096]u8,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, connection: std.net.Server.Connection) !*Self {
        const self = try allocator.create(Self);
        self.* = .{
            .allocator = allocator,
            .connection = connection,
            .running = true,
            .write_buffer = undefined,
            .read_buffer = undefined,
        };
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.running = false;
        self.connection.stream.close();
    }

    pub fn handle(self: *Self) !void {
        while (self.running) {
            if (self.connection.stream.read(&self.read_buffer)) |n| {
                if (n == 0) break; // Connection closed
                
                var stream = std.io.fixedBufferStream(self.read_buffer[0..n]);
                const frame = try protocol.Frame.decode(stream.reader(), self.allocator);
                defer self.allocator.free(frame.payload);

                switch (frame.type) {
                    protocol.MESSAGE_TYPE_COMMAND => {
                        if (std.json.parse(protocol.CommandMessage, self.allocator, frame.payload, .{})) |cmd| {
                            // TODO: Add to command queue
                            self.allocator.free(cmd);
                        } else |err| {
                            std.log.err("Failed to parse command: {}", .{err});
                        }
                    },
                    else => {}, // Ignore other message types from client
                }
            } else |err| {
                std.log.err("Error reading from client: {}", .{err});
                break;
            }
        }
    }
};

pub const Communication = struct {
    ws_server: *WebSocketServer,
    allocator: std.mem.Allocator,
    joint_manager: *joints.JointManager,

    pub fn init(allocator: std.mem.Allocator, joint_manager: *joints.JointManager) !Communication {
        const ws_server = try WebSocketServer.init(allocator, 8080, joint_manager);
        return Communication{
            .ws_server = ws_server,
            .allocator = allocator,
            .joint_manager = joint_manager,
        };
    }

    pub fn deinit(self: *Communication) void {
        self.ws_server.deinit();
    }

    pub fn start(self: *Communication) !void {
        try self.ws_server.start();
    }
};

pub fn init() core.Result {
    return core.Result{ .success = {} };
} 