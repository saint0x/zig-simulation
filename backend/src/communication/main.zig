const std = @import("std");
const core = @import("../core/main.zig");
const crypto = std.crypto;
const protocol = @import("protocol.zig");
const Thread = std.Thread;
const time = std.time;

const UpdateThread = struct {
    thread: Thread,
    running: bool,
    clients: *std.ArrayList(*Client),
    allocator: std.mem.Allocator,
    mutex: Thread.Mutex,
    command_queue: std.ArrayList(protocol.CommandMessage),

    pub fn init(allocator: std.mem.Allocator, clients: *std.ArrayList(*Client)) !*UpdateThread {
        const self = try allocator.create(UpdateThread);
        self.* = .{
            .thread = undefined,
            .running = false,
            .clients = clients,
            .allocator = allocator,
            .mutex = Thread.Mutex{},
            .command_queue = std.ArrayList(protocol.CommandMessage).init(allocator),
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
        const update_interval_ns = time.ns_per_ms; // 1kHz
        const status_interval_ns = 10 * time.ns_per_ms; // 100Hz

        while (self.running) {
            const now = timer.read();

            // Joint state updates (1kHz)
            try self.sendJointStates();

            // System status updates (100Hz)
            if (now - last_status_update >= status_interval_ns) {
                try self.sendSystemStatus();
                last_status_update = now;
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
        const state = protocol.JointStateMessage{
            .timestamp_us = @as(u64, time.microTimestamp()),
            .positions = [_]f32{0} ** 6, // TODO: Get real values
            .velocities = [_]f32{0} ** 6,
            .torques = [_]f32{0} ** 6,
            .temperatures = [_]f32{0} ** 6,
            .currents = [_]f32{0} ** 6,
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
                protocol.COMMAND_TYPE_POSITION => {}, // TODO: Implement
                protocol.COMMAND_TYPE_VELOCITY => {}, // TODO: Implement
                protocol.COMMAND_TYPE_TORQUE => {}, // TODO: Implement
                protocol.COMMAND_TYPE_CONTROL_MODE => {}, // TODO: Implement
                protocol.COMMAND_TYPE_SAFETY => {}, // TODO: Implement
                else => {},
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

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, port: u16) !*Self {
        const server = try std.net.Server.init(.{
            .address = std.net.Address.initIp4(.{ 0, 0, 0, 0 }, port),
            .reuse_address = true,
        });

        var clients = std.ArrayList(*Client).init(allocator);
        const update_thread = try UpdateThread.init(allocator, &clients);

        const self = try allocator.create(Self);
        self.* = .{
            .server = server,
            .allocator = allocator,
            .port = port,
            .clients = clients,
            .running = false,
            .update_thread = update_thread,
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

    pub fn init(allocator: std.mem.Allocator) !Communication {
        const ws_server = try WebSocketServer.init(allocator, 8080);
        return Communication{
            .ws_server = ws_server,
            .allocator = allocator,
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