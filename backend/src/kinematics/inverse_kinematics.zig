const std = @import("std");
const math = std.math;
const Vec3 = @import("types.zig").Vec3;
const JointAngles = @import("types.zig").JointAngles;
const ForwardKinematics = @import("kinematics.zig").ForwardKinematics;

/// Inverse kinematics solver using numerical methods
pub const InverseKinematics = struct {
    fk: ForwardKinematics,
    max_iterations: u32 = 100,
    tolerance: f32 = 0.001,
    learning_rate: f32 = 0.1,

    pub fn init() InverseKinematics {
        return .{
            .fk = ForwardKinematics.init(),
        };
    }

    /// Calculate joint angles to reach target position and orientation
    pub fn solve(
        self: *InverseKinematics,
        target_pos: Vec3,
        target_rot: Vec3,
        initial_angles: ?JointAngles,
    ) !JointAngles {
        var current_angles = initial_angles orelse JointAngles.zero();
        var iteration: u32 = 0;

        while (iteration < self.max_iterations) {
            // Get current end-effector pose
            var current_pos: Vec3 = undefined;
            var current_rot: Vec3 = undefined;
            self.fk.calcEndEffectorPose(current_angles, &current_pos, &current_rot);

            // Calculate error
            const pos_error = self.calculatePositionError(target_pos, current_pos);
            const rot_error = self.calculateRotationError(target_rot, current_rot);

            // Check if we've reached the target
            if (pos_error < self.tolerance and rot_error < self.tolerance) {
                return current_angles;
            }

            // Calculate Jacobian and update angles
            const jacobian = self.calculateJacobian(current_angles);
            const delta_angles = self.calculateDeltaAngles(jacobian, pos_error, rot_error);
            current_angles = self.updateAngles(current_angles, delta_angles);

            iteration += 1;
        }

        return error.MaxIterationsReached;
    }

    /// Calculate position error between target and current position
    fn calculatePositionError(self: *InverseKinematics, target: Vec3, current: Vec3) f32 {
        _ = self;
        const dx = target.x - current.x;
        const dy = target.y - current.y;
        const dz = target.z - current.z;
        return @sqrt(dx * dx + dy * dy + dz * dz);
    }

    /// Calculate rotation error between target and current orientation
    fn calculateRotationError(self: *InverseKinematics, target: Vec3, current: Vec3) f32 {
        _ = self;
        const droll = target.x - current.x;
        const dpitch = target.y - current.y;
        const dyaw = target.z - current.z;
        return @sqrt(droll * droll + dpitch * dpitch + dyaw * dyaw);
    }

    /// Calculate Jacobian matrix for current joint angles
    fn calculateJacobian(self: *InverseKinematics, angles: JointAngles) [6][7]f32 {
        var jacobian: [6][7]f32 = undefined;
        const delta: f32 = 0.001; // Small angle change for numerical differentiation

        // Calculate Jacobian using finite differences
        for (0..7) |i| {
            var angles_plus = angles;
            angles_plus[i] += delta;
            var pos_plus: Vec3 = undefined;
            var rot_plus: Vec3 = undefined;
            self.fk.calcEndEffectorPose(angles_plus, &pos_plus, &rot_plus);

            var angles_minus = angles;
            angles_minus[i] -= delta;
            var pos_minus: Vec3 = undefined;
            var rot_minus: Vec3 = undefined;
            self.fk.calcEndEffectorPose(angles_minus, &pos_minus, &rot_minus);

            // Position derivatives
            jacobian[0][i] = (pos_plus.x - pos_minus.x) / (2 * delta);
            jacobian[1][i] = (pos_plus.y - pos_minus.y) / (2 * delta);
            jacobian[2][i] = (pos_plus.z - pos_minus.z) / (2 * delta);

            // Rotation derivatives
            jacobian[3][i] = (rot_plus.x - rot_minus.x) / (2 * delta);
            jacobian[4][i] = (rot_plus.y - rot_minus.y) / (2 * delta);
            jacobian[5][i] = (rot_plus.z - rot_minus.z) / (2 * delta);
        }

        return jacobian;
    }

    /// Calculate delta angles using Jacobian
    fn calculateDeltaAngles(
        self: *InverseKinematics,
        jacobian: [6][7]f32,
        pos_error: f32,
        rot_error: f32,
    ) JointAngles {
        // Weight position and rotation errors
        const total_error = pos_error + 0.5 * rot_error;
        
        // Simple gradient descent
        var delta_angles: JointAngles = undefined;
        for (0..7) |i| {
            var sum: f32 = 0;
            for (0..6) |j| {
                sum += jacobian[j][i] * self.learning_rate * total_error;
            }
            delta_angles[i] = sum;
        }
        return delta_angles;
    }

    /// Update joint angles with delta angles
    fn updateAngles(
        self: *InverseKinematics,
        current: JointAngles,
        delta: JointAngles,
    ) JointAngles {
        _ = self;
        var new_angles = current;
        for (0..7) |i| {
            new_angles[i] += delta[i];
        }
        return new_angles;
    }
}; 