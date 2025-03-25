const std = @import("std");
const types = @import("../core/types.zig");

const Self = @This();

/// Forward kinematics state
joint_angles: [types.NUM_JOINTS]f32,
/// Link dimensions
link_dimensions: [9]types.LinkDimensions,

/// Initialize forward kinematics
pub fn init(link_dimensions: [9]types.LinkDimensions) Self {
    var joint_angles: [types.NUM_JOINTS]f32 = undefined;
    for (&joint_angles) |*angle| {
        angle.* = 0;
    }

    return Self{
        .joint_angles = joint_angles,
        .link_dimensions = link_dimensions,
    };
}

/// Update joint angles
pub fn updateJointAngles(self: *Self, new_angles: [types.NUM_JOINTS]f32) void {
    self.joint_angles = new_angles;
}

/// Compute the position of a link based on joint angles
pub fn computeLinkPosition(self: *Self, link: types.LinkDimensions) types.CartesianPosition {
    var position = types.CartesianPosition{
        .x = link.joint_offset.x,
        .y = link.joint_offset.y,
        .z = link.joint_offset.z,
    };

    // Apply transformations based on joint angles
    switch (link.id) {
        .base => {
            // Base rotation affects all subsequent links
            const base_angle = self.joint_angles[0] * std.math.pi / 180.0;
            const x = position.x * @cos(base_angle) - position.z * @sin(base_angle);
            const z = position.x * @sin(base_angle) + position.z * @cos(base_angle);
            position.x = x;
            position.z = z;
        },
        .shoulder => {
            // Shoulder rotation affects upper arm and subsequent links
            const shoulder_angle = self.joint_angles[1] * std.math.pi / 180.0;
            const y = position.y * @cos(shoulder_angle) - position.z * @sin(shoulder_angle);
            const z = position.y * @sin(shoulder_angle) + position.z * @cos(shoulder_angle);
            position.y = y;
            position.z = z;
        },
        .elbow => {
            // Elbow rotation affects forearm and subsequent links
            const elbow_angle = self.joint_angles[2] * std.math.pi / 180.0;
            const y = position.y * @cos(elbow_angle) - position.z * @sin(elbow_angle);
            const z = position.y * @sin(elbow_angle) + position.z * @cos(elbow_angle);
            position.y = y;
            position.z = z;
        },
        else => {
            // For other links, apply their respective joint rotations
            const joint_idx = @intFromEnum(link.id) - 1;
            if (joint_idx < self.joint_angles.len) {
                const angle = self.joint_angles[joint_idx] * std.math.pi / 180.0;
                const y = position.y * @cos(angle) - position.z * @sin(angle);
                const z = position.y * @sin(angle) + position.z * @cos(angle);
                position.y = y;
                position.z = z;
            }
        },
    }

    return position;
} 