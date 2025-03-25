const std = @import("std");
const core = @import("core");
const types = @import("types.zig");
const timing = @import("timing");
const joint_manager = @import("joint_manager/root.zig");

const JointConfig = types.JointConfig;
const JointState = types.JointState;
const RobotState = types.RobotState;
const TimingSystem = timing.TimingSystem;
const JointManager = joint_manager.JointManager;

/// Motion planning configuration
pub const MotionConfig = struct {
    /// Maximum velocity for each joint (rad/s)
    max_velocities: [7]f32,
    /// Maximum acceleration for each joint (rad/s²)
    max_accelerations: [7]f32,
    /// Time step for trajectory generation (s)
    dt: f32,
    /// Minimum time between waypoints (s)
    min_waypoint_time: f32,
};

/// Trajectory point containing joint positions and timing
pub const TrajectoryPoint = struct {
    /// Joint positions (rad)
    positions: [7]f32,
    /// Time from start (s)
    time: f32,
};

/// Motion planner for generating smooth trajectories
pub const MotionPlanner = struct {
    /// Motion planning configuration
    config: MotionConfig,
    /// Joint manager for validation
    joint_mgr: *JointManager,
    /// Current trajectory
    trajectory: std.ArrayList(TrajectoryPoint),
    /// Current trajectory index
    current_index: usize,
    /// Whether trajectory is valid
    is_valid: bool,

    /// Initialize motion planner
    pub fn init(
        config: MotionConfig,
        joint_mgr: *JointManager,
        allocator: std.mem.Allocator,
    ) !MotionPlanner {
        return MotionPlanner{
            .config = config,
            .joint_mgr = joint_mgr,
            .trajectory = std.ArrayList(TrajectoryPoint).init(allocator),
            .current_index = 0,
            .is_valid = false,
        };
    }

    /// Deinitialize motion planner
    pub fn deinit(self: *MotionPlanner) void {
        self.trajectory.deinit();
    }

    /// Plan trajectory from current state to target state
    pub fn planTrajectory(
        self: *MotionPlanner,
        current_state: *const JointState,
        target_state: *const JointState,
    ) !void {
        // Clear existing trajectory
        self.trajectory.clearRetainingCapacity();
        self.current_index = 0;
        self.is_valid = false;

        // Calculate time needed for each joint
        var max_time: f32 = 0;
        for (0..7) |i| {
            const distance = @abs(target_state.positions[i] - current_state.positions[i]);
            const time = calculateJointTime(
                distance,
                self.config.max_velocities[i],
                self.config.max_accelerations[i],
            );
            max_time = @max(max_time, time);
        }

        // Generate trajectory points
        const num_points = @as(usize, @intFromFloat(max_time / self.config.dt));
        try self.trajectory.ensureTotalCapacity(num_points + 1);

        // Add initial point
        try self.trajectory.append(.{
            .positions = current_state.positions,
            .time = 0,
        });

        // Generate intermediate points
        var t: f32 = self.config.dt;
        while (t <= max_time) : (t += self.config.dt) {
            var point = TrajectoryPoint{
                .positions = undefined,
                .time = t,
            };

            // Calculate position for each joint
            for (0..7) |i| {
                const distance = target_state.positions[i] - current_state.positions[i];
                point.positions[i] = calculatePosition(
                    distance,
                    t,
                    self.config.max_velocities[i],
                    self.config.max_accelerations[i],
                ) + current_state.positions[i];
            }

            try self.trajectory.append(point);
        }

        // Add final point
        try self.trajectory.append(.{
            .positions = target_state.positions,
            .time = max_time,
        });

        // Validate trajectory
        self.is_valid = self.validateTrajectory();
    }

    /// Get next trajectory point
    pub fn getNextPoint(self: *MotionPlanner) ?TrajectoryPoint {
        if (self.current_index >= self.trajectory.items.len) {
            return null;
        }

        const point = self.trajectory.items[self.current_index];
        self.current_index += 1;
        return point;
    }

    /// Reset trajectory to beginning
    pub fn resetTrajectory(self: *MotionPlanner) void {
        self.current_index = 0;
    }

    /// Check if trajectory is complete
    pub fn isComplete(self: *const MotionPlanner) bool {
        return self.current_index >= self.trajectory.items.len;
    }

    /// Calculate time needed for a joint to move a given distance
    fn calculateJointTime(distance: f32, max_velocity: f32, max_acceleration: f32) f32 {
        // Time to reach max velocity
        const t1 = max_velocity / max_acceleration;
        // Distance covered during acceleration
        const d1 = 0.5 * max_acceleration * t1 * t1;
        // Distance covered at constant velocity
        const d2 = distance - 2 * d1;

        if (d2 <= 0) {
            // Distance too short to reach max velocity
            return 2 * @sqrt(distance / max_acceleration);
        }

        // Time at constant velocity
        const t2 = d2 / max_velocity;
        return 2 * t1 + t2;
    }

    /// Calculate position at given time
    fn calculatePosition(distance: f32, time: f32, max_velocity: f32, max_acceleration: f32) f32 {
        // Time to reach max velocity
        const t1 = max_velocity / max_acceleration;
        // Distance covered during acceleration
        const d1 = 0.5 * max_acceleration * t1 * t1;
        // Distance covered at constant velocity
        const d2 = distance - 2 * d1;

        if (d2 <= 0) {
            // Distance too short to reach max velocity
            const t = time / 2;
            if (t < t1) {
                return 0.5 * max_acceleration * t * t;
            } else {
                const t_remaining = time - t1;
                return d1 + max_velocity * t_remaining;
            }
        }

        // Time at constant velocity
        const t2 = d2 / max_velocity;
        const t_total = 2 * t1 + t2;

        if (time <= t1) {
            // Acceleration phase
            return 0.5 * max_acceleration * time * time;
        } else if (time <= t1 + t2) {
            // Constant velocity phase
            return d1 + max_velocity * (time - t1);
        } else {
            // Deceleration phase
            const t_remaining = t_total - time;
            return distance - 0.5 * max_acceleration * t_remaining * t_remaining;
        }
    }

    /// Validate trajectory against joint limits and constraints
    fn validateTrajectory(self: *MotionPlanner) bool {
        for (self.trajectory.items) |point| {
            // Check joint limits
            for (0..7) |i| {
                if (!self.joint_mgr.isValidPosition(i, point.positions[i])) {
                    return false;
                }
            }

            // Check velocity limits
            if (self.current_index > 0) {
                const prev_point = self.trajectory.items[self.current_index - 1];
                const dt = point.time - prev_point.time;
                for (0..7) |i| {
                    const velocity = @abs(point.positions[i] - prev_point.positions[i]) / dt;
                    if (velocity > self.config.max_velocities[i]) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
}; 