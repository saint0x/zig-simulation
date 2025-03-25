const std = @import("std");

// Core types and error handling
pub const types = @import("types.zig");
pub const error = @import("error.zig");

// Control system
pub const pid = @import("../control/pid.zig");
pub const joint_manager = @import("../control/joint_manager.zig");
pub const motion_planning = @import("../control/motion_planning.zig");

// Kinematics
pub const forward_kinematics = @import("../kinematics/forward_kinematics.zig");
pub const inverse_kinematics = @import("../kinematics/inverse_kinematics.zig");
pub const collision_detection = @import("../kinematics/collision_detection.zig");

// Safety
pub const safety = @import("../safety/safety.zig");

// Timing
pub const timing = @import("../timing/timing.zig");

// Re-export commonly used types for convenience
pub const TimingConfig = timing.TimingConfig;
pub const TimingSystem = timing.TimingSystem;
pub const JointConfig = types.JointConfig;
pub const JointState = types.JointState;
pub const RobotState = types.RobotState;
pub const RobotStatus = types.RobotStatus;
pub const CartesianPosition = types.CartesianPosition;
pub const Orientation = types.Orientation;
pub const EndEffectorPose = types.EndEffectorPose;
pub const PIDController = pid.PIDController;
pub const JointManager = joint_manager.JointManager;
pub const MotionPlanner = motion_planning.MotionPlanner;
pub const MotionConfig = motion_planning.MotionConfig;
pub const TrajectoryPoint = motion_planning.TrajectoryPoint;
pub const SafetyMonitor = safety.SafetyMonitor;
pub const Error = error.Error;
pub const ErrorContext = error.ErrorContext;
pub const ForwardKinematics = forward_kinematics.ForwardKinematics;
pub const TransformMatrix = forward_kinematics.TransformMatrix;
pub const Vec3 = types.Vec3;
pub const JointAngles = types.JointAngles;
pub const InverseKinematics = inverse_kinematics.InverseKinematics; 