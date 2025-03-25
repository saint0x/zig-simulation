const std = @import("std");
const types = @import("types.zig");
const math = std.math;
const JointState = types.JointState;
const CartesianPosition = types.CartesianPosition;
const Orientation = types.Orientation;
const EndEffectorPose = types.EndEffectorPose;

/// 4x4 homogeneous transformation matrix
pub const TransformMatrix = struct {
    data: [4][4]f32,

    /// Initialize identity matrix
    pub fn identity() TransformMatrix {
        return .{
            .data = .{
                .{ 1.0, 0.0, 0.0, 0.0 },
                .{ 0.0, 1.0, 0.0, 0.0 },
                .{ 0.0, 0.0, 1.0, 0.0 },
                .{ 0.0, 0.0, 0.0, 1.0 },
            },
        };
    }

    /// Multiply two transformation matrices
    pub fn multiply(self: TransformMatrix, other: TransformMatrix) TransformMatrix {
        var result = TransformMatrix.identity();
        for (0..4) |i| {
            for (0..4) |j| {
                result.data[i][j] = 0.0;
                for (0..4) |k| {
                    result.data[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }
};

/// Denavit-Hartenberg parameters for KUKA arm
const DHParams = struct {
    alpha: f32, // Link twist
    a: f32,     // Link length
    d: f32,     // Link offset
    theta: f32, // Joint angle
};

/// Forward kinematics calculator
pub const ForwardKinematics = struct {
    /// DH parameters for each joint
    dh_params: [7]DHParams,

    /// Initialize forward kinematics with default DH parameters
    pub fn init() ForwardKinematics {
        return .{
            .dh_params = .{
                // Values based on KUKA LBR iiwa 7 R800
                .{ .alpha = -math.pi / 2.0, .a = 0.0,    .d = 0.34,  .theta = 0.0 },  // Joint 1
                .{ .alpha = math.pi / 2.0,  .a = 0.0,    .d = 0.0,   .theta = 0.0 },  // Joint 2
                .{ .alpha = math.pi / 2.0,  .a = 0.0,    .d = 0.4,   .theta = 0.0 },  // Joint 3
                .{ .alpha = -math.pi / 2.0, .a = 0.0,    .d = 0.0,   .theta = 0.0 },  // Joint 4
                .{ .alpha = -math.pi / 2.0, .a = 0.0,    .d = 0.4,   .theta = 0.0 },  // Joint 5
                .{ .alpha = math.pi / 2.0,  .a = 0.0,    .d = 0.0,   .theta = 0.0 },  // Joint 6
                .{ .alpha = 0.0,           .a = 0.0,    .d = 0.126, .theta = 0.0 },  // Joint 7
            },
        };
    }

    /// Calculate transformation matrix from DH parameters
    fn calcTransform(params: DHParams) TransformMatrix {
        const ct = @cos(params.theta);
        const st = @sin(params.theta);
        const ca = @cos(params.alpha);
        const sa = @sin(params.alpha);

        return .{
            .data = .{
                .{ ct, -st * ca, st * sa,  params.a * ct },
                .{ st, ct * ca,  -ct * sa, params.a * st },
                .{ 0,  sa,       ca,       params.d      },
                .{ 0,  0,        0,        1            },
            },
        };
    }

    /// Calculate end-effector pose from joint angles
    pub fn calcEndEffectorPose(self: *ForwardKinematics, joint_angles: []const f32) EndEffectorPose {
        // Update joint angles in DH parameters
        for (joint_angles, 0..) |angle, i| {
            self.dh_params[i].theta = angle;
        }

        // Calculate transformation matrices
        var transform = TransformMatrix.identity();
        for (self.dh_params) |params| {
            const joint_transform = calcTransform(params);
            transform = transform.multiply(joint_transform);
        }

        // Extract position and orientation
        const position = CartesianPosition{
            .x = transform.data[0][3],
            .y = transform.data[1][3],
            .z = transform.data[2][3],
        };

        // Convert rotation matrix to Euler angles (ZYX convention)
        const orientation = Orientation{
            .roll = math.atan2(transform.data[2][1], transform.data[2][2]),
            .pitch = math.asin(-transform.data[2][0]),
            .yaw = math.atan2(transform.data[1][0], transform.data[0][0]),
        };

        return EndEffectorPose{
            .position = position,
            .orientation = orientation,
        };
    }

    /// Get transformation matrix for a specific joint
    pub fn getJointTransform(self: *ForwardKinematics, joint_angles: []const f32, joint_index: usize) TransformMatrix {
        var transform = TransformMatrix.identity();
        var i: usize = 0;
        while (i <= joint_index) : (i += 1) {
            self.dh_params[i].theta = joint_angles[i];
            const joint_transform = calcTransform(self.dh_params[i]);
            transform = transform.multiply(joint_transform);
        }
        return transform;
    }
}; 