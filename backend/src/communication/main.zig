const std = @import("std");
const core = @import("../core/main.zig");
const crypto = std.crypto;

const WebSocketServer = struct {
    server: std.net.Server,
    allocator: std.mem.Allocator,
    port: u16,
    clients: std.ArrayList(*Client),
    running: bool,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, port: u16) !*Self {
        const server = try std.net.Server.init(.{
            .address = std.net.Address.initIp4(.{ 0, 0, 0, 0 }, port),
            .reuse_address = true,
        });

        const self = try allocator.create(Self);
        self.* = .{
            .server = server,
            .allocator = allocator,
            .port = port,
            .clients = std.ArrayList(*Client).init(allocator),
            .running = false,
        };
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.running = false;
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

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, connection: std.net.Server.Connection) !*Self {
        const self = try allocator.create(Self);
        self.* = .{
            .allocator = allocator,
            .connection = connection,
            .running = true,
        };
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.running = false;
        self.connection.stream.close();
    }

    pub fn handle(self: *Self) !void {
        var buf: [1024]u8 = undefined;
        
        while (self.running) {
            if (self.connection.stream.read(&buf)) |n| {
                if (n == 0) break; // Connection closed
                
                // Echo the message back (for testing)
                _ = try self.connection.stream.write(buf[0..n]);
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