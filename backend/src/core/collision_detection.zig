const std = @import("std");
const types = @import("../core/types.zig");

const Self = @This();

/// Collision detection system state
config: types.CollisionConfig,
/// Array of link geometries
links: []types.LinkGeometry,
/// Last collision check timestamp
last_check_time: f64,
/// Current collision result
current_result: types.CollisionResult,

/// Initialize a new collision detection system
pub fn init(
    allocator: std.mem.Allocator,
    config: types.CollisionConfig,
    num_links: usize,
) !Self {
    const links = try allocator.alloc(types.LinkGeometry, num_links);
    errdefer allocator.free(links);

    // Initialize link geometries with default values
    for (links, 0..) |*link, i| {
        link.* = .{
            .id = @intCast(i),
            .bounds = .{
                .min = .{ .x = 0, .y = 0, .z = 0 },
                .max = .{ .x = 0, .y = 0, .z = 0 },
            },
            .center = .{ .x = 0, .y = 0, .z = 0 },
            .orientation = .{ .roll = 0, .pitch = 0, .yaw = 0 },
        };
    }

    return Self{
        .config = config,
        .links = links,
        .last_check_time = 0,
        .current_result = .{
            .collision_detected = false,
            .collision_type = .none,
            .link1 = null,
            .link2 = null,
            .collision_point = null,
            .min_distance = std.math.inf(f32),
        },
    };
}

/// Deinitialize the collision detection system
pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
    allocator.free(self.links);
}

/// Update link geometries with current robot state
pub fn updateLinkGeometries(self: *Self, new_links: []const types.LinkGeometry) void {
    std.mem.copy(types.LinkGeometry, self.links, new_links);
}

/// Check for collisions between links
fn checkSelfCollisions(self: *Self) types.CollisionResult {
    var result = types.CollisionResult{
        .collision_detected = false,
        .collision_type = .none,
        .link1 = null,
        .link2 = null,
        .collision_point = null,
        .min_distance = std.math.inf(f32),
    };

    // Check each pair of links for collisions
    for (self.links, 0..) |link1, i| {
        for (self.links[i + 1..]) |link2| {
            // Skip adjacent links as they're connected
            if (link2.id == link1.id + 1) continue;

            const distance = self.computeLinkDistance(link1, link2);
            if (distance < self.config.min_link_distance) {
                result.collision_detected = true;
                result.collision_type = .self_collision;
                result.link1 = link1.id;
                result.link2 = link2.id;
                result.min_distance = distance;
                return result;
            }
        }
    }

    return result;
}

/// Compute the minimum distance between two links
fn computeLinkDistance(self: *Self, link1: types.LinkGeometry, link2: types.LinkGeometry) f32 {
    // Simple distance between link centers for now
    // TODO: Implement more sophisticated distance computation using bounding boxes
    const dx = link1.center.x - link2.center.x;
    const dy = link1.center.y - link2.center.y;
    const dz = link1.center.z - link2.center.z;
    return @sqrt(dx * dx + dy * dy + dz * dz);
}

/// Perform collision detection check
pub fn checkCollisions(self: *Self, current_time: f64) types.CollisionResult {
    // Check if enough time has passed since last check
    const time_since_last_check = current_time - self.last_check_time;
    if (time_since_last_check < (1.0 / self.config.check_frequency)) {
        return self.current_result;
    }

    // Update last check time
    self.last_check_time = current_time;

    // Check for self collisions
    const self_collision_result = self.checkSelfCollisions();
    if (self_collision_result.collision_detected) {
        self.current_result = self_collision_result;
        return self.current_result;
    }

    // TODO: Implement environment collision detection
    // For now, return no collision
    self.current_result = .{
        .collision_detected = false,
        .collision_type = .none,
        .link1 = null,
        .link2 = null,
        .collision_point = null,
        .min_distance = std.math.inf(f32),
    };

    return self.current_result;
}

/// Get the current collision detection result
pub fn getCurrentResult(self: *const Self) types.CollisionResult {
    return self.current_result;
} 