const std = @import("std");
const types = @import("../core/types.zig");
const ForwardKinematics = @import("./forward_kinematics.zig").Self;

const Self = @This();

/// Collision detection system state
config: types.CollisionConfig,
/// Forward kinematics instance
fk: ForwardKinematics,
/// Last collision check timestamp
last_check_time: f64,
/// Current collision result
current_result: types.CollisionResult,

/// Initialize a new collision detection system
pub fn init(
    allocator: std.mem.Allocator,
    config: types.CollisionConfig,
) !Self {
    // Initialize forward kinematics
    const fk = ForwardKinematics.init(config.link_dimensions);

    return Self{
        .config = config,
        .fk = fk,
        .last_check_time = 0,
        .current_result = .{
            .collision_detected = false,
            .collision_type = .none,
            .link1 = null,
            .link2 = null,
            .collision_point = null,
            .min_distance = std.math.inf(f32),
            .joint_angles = fk.joint_angles,
        },
    };
}

/// Update joint angles
pub fn updateJointAngles(self: *Self, new_angles: [types.NUM_JOINTS]f32) void {
    self.fk.updateJointAngles(new_angles);
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
        .joint_angles = self.fk.joint_angles,
    };

    // Check each pair of non-adjacent links for collisions
    for (self.config.link_dimensions, 0..) |link1, i| {
        for (self.config.link_dimensions[i + 1..]) |link2| {
            // Skip adjacent links as they're connected
            if (@intFromEnum(link2.id) == @intFromEnum(link1.id) + 1) continue;

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
fn computeLinkDistance(self: *Self, link1: types.LinkDimensions, link2: types.LinkDimensions) f32 {
    // Get the current positions of both links using forward kinematics
    const pos1 = self.fk.computeLinkPosition(link1);
    const pos2 = self.fk.computeLinkPosition(link2);

    // Compute the minimum distance between the bounding boxes of the links
    const dx = pos1.x - pos2.x;
    const dy = pos1.y - pos2.y;
    const dz = pos1.z - pos2.z;

    // Add half the dimensions of both links to get the minimum distance between their surfaces
    const min_x = (link1.width + link2.width) / 2;
    const min_y = (link1.height + link2.height) / 2;
    const min_z = (link1.length + link2.length) / 2;

    // Compute the actual distance between the links
    const distance = @sqrt(dx * dx + dy * dy + dz * dz);
    
    // Return the distance minus the minimum required distance
    return distance - @sqrt(min_x * min_x + min_y * min_y + min_z * min_z);
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
        .joint_angles = self.fk.joint_angles,
    };

    return self.current_result;
}

/// Get the current collision detection result
pub fn getCurrentResult(self: *const Self) types.CollisionResult {
    return self.current_result;
} 