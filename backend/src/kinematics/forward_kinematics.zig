const std = @import("std");
const core = @import("core");
const types = @import("types");

const NUM_LINKS = 9; // Number of links including base and end-effector
const LinkId = core.types.LinkId;
const LinkDimensions = core.types.LinkDimensions;
const CartesianPosition = core.types.CartesianPosition;
const Orientation = core.types.Orientation;

pub const TransformMatrix = [4][4]f32;

pub const ForwardKinematics = struct {
    link_dimensions: [NUM_LINKS]LinkDimensions,
    joint_angles: [core.types.NUM_JOINTS]f32,

    pub fn init(link_dimensions: [NUM_LINKS]LinkDimensions) ForwardKinematics {
        return ForwardKinematics{
            .link_dimensions = link_dimensions,
            .joint_angles = [_]f32{0} ** core.types.NUM_JOINTS,
        };
    }

    pub fn updateJointAngles(self: *ForwardKinematics, joint_angles: []const f32) void {
        std.debug.assert(joint_angles.len == core.types.NUM_JOINTS);
        std.mem.copy(f32, &self.joint_angles, joint_angles);
    }

    pub fn computeLinkPosition(self: *const ForwardKinematics, link_id: LinkId) struct { position: CartesianPosition, orientation: Orientation } {
        // Initialize position vector
        var position = CartesianPosition{ .x = 0, .y = 0, .z = 0 };

        // Apply transformations based on joint angles
        switch (link_id) {
            .base => {
                // Base link position is fixed at origin
                position = CartesianPosition{ .x = 0, .y = 0, .z = 0 };
            },
            .shoulder => {
                // Apply rotation around Z axis
                const theta = self.joint_angles[0];
                position = CartesianPosition{
                    .x = self.link_dimensions[0].length * @cos(theta),
                    .y = self.link_dimensions[0].length * @sin(theta),
                    .z = 0,
                };
            },
            .upper_arm => {
                // Apply rotations around Y and Z axes
                const theta1 = self.joint_angles[0];
                const theta2 = self.joint_angles[1];
                position = CartesianPosition{
                    .x = self.link_dimensions[1].length * @cos(theta1) * @cos(theta2),
                    .y = self.link_dimensions[1].length * @sin(theta1) * @cos(theta2),
                    .z = self.link_dimensions[1].length * @sin(theta2),
                };
            },
            .elbow => {
                // Apply rotations around Y and Z axes
                const theta1 = self.joint_angles[0];
                const theta2 = self.joint_angles[1];
                const theta3 = self.joint_angles[2];
                position = CartesianPosition{
                    .x = self.link_dimensions[2].length * @cos(theta1) * @cos(theta2 + theta3),
                    .y = self.link_dimensions[2].length * @sin(theta1) * @cos(theta2 + theta3),
                    .z = self.link_dimensions[2].length * @sin(theta2 + theta3),
                };
            },
            .forearm => {
                // Apply rotations around Y and Z axes
                const theta1 = self.joint_angles[0];
                const theta2 = self.joint_angles[1];
                const theta3 = self.joint_angles[2];
                const theta4 = self.joint_angles[3];
                position = CartesianPosition{
                    .x = self.link_dimensions[3].length * @cos(theta1) * @cos(theta2 + theta3 + theta4),
                    .y = self.link_dimensions[3].length * @sin(theta1) * @cos(theta2 + theta3 + theta4),
                    .z = self.link_dimensions[3].length * @sin(theta2 + theta3 + theta4),
                };
            },
            .wrist_rotation => {
                // Apply rotations around Y and Z axes
                const theta1 = self.joint_angles[0];
                const theta2 = self.joint_angles[1];
                const theta3 = self.joint_angles[2];
                const theta4 = self.joint_angles[3];
                const theta5 = self.joint_angles[4];
                position = CartesianPosition{
                    .x = self.link_dimensions[4].length * @cos(theta1) * @cos(theta2 + theta3 + theta4 + theta5),
                    .y = self.link_dimensions[4].length * @sin(theta1) * @cos(theta2 + theta3 + theta4 + theta5),
                    .z = self.link_dimensions[4].length * @sin(theta2 + theta3 + theta4 + theta5),
                };
            },
            .wrist_bend => {
                // Apply rotations around Y and Z axes
                const theta1 = self.joint_angles[0];
                const theta2 = self.joint_angles[1];
                const theta3 = self.joint_angles[2];
                const theta4 = self.joint_angles[3];
                const theta5 = self.joint_angles[4];
                const theta6 = self.joint_angles[5];
                position = CartesianPosition{
                    .x = self.link_dimensions[5].length * @cos(theta1) * @cos(theta2 + theta3 + theta4 + theta5 + theta6),
                    .y = self.link_dimensions[5].length * @sin(theta1) * @cos(theta2 + theta3 + theta4 + theta5 + theta6),
                    .z = self.link_dimensions[5].length * @sin(theta2 + theta3 + theta4 + theta5 + theta6),
                };
            },
            .tool_rotation, .gripper => {
                // Tool position is based on the last link's position
                const last_link = self.link_dimensions[NUM_LINKS - 1];
                const last_pos = self.computeLinkPosition(last_link.id);
                position = CartesianPosition{
                    .x = last_pos.position.x + last_link.length * @cos(self.joint_angles[0]),
                    .y = last_pos.position.y + last_link.length * @sin(self.joint_angles[0]),
                    .z = last_pos.position.z,
                };
            },
        }

        return .{
            .position = position,
            .orientation = Orientation{
                .roll = 0,  // TODO: Implement proper orientation calculations
                .pitch = 0,
                .yaw = 0,
            },
        };
    }
}; 