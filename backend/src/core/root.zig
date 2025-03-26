const std = @import("std");

// Core initialization and system
const init_module = @import("init.zig");
pub const CoreSystem = init_module.CoreSystem;
pub const init = init_module.init;

// Core types and error handling
pub const types = @import("types.zig");
pub const error_handler = @import("error.zig");
pub const timing = @import("timing");

// Control system
pub const control = @import("control");
pub const pid = control.pid;
pub const motion_planning = control.motion_planning;

// Kinematics
pub const kinematics = @import("kinematics");
pub const forward_kinematics = kinematics.forward_kinematics;
pub const inverse_kinematics = kinematics.inverse_kinematics;
pub const collision_detection = kinematics.collision_detection;

// Safety
pub const safety = @import("safety");

// Re-export timing types
pub const TimingSystem = timing.TimingSystem;
pub const TimeUnit = timing.TimeUnit;
pub const TimerConfig = timing.TimerConfig;
pub const TimingConfig = timing.TimingConfig;

// Re-export core types
pub const JointConfig = types.JointConfig;
pub const JointState = types.JointState;
pub const RobotState = types.RobotState;
pub const RobotStatus = types.RobotStatus;
pub const CartesianPosition = types.CartesianPosition;
pub const Orientation = types.Orientation;
pub const EndEffectorPose = types.EndEffectorPose;
pub const BoundingBox = types.BoundingBox;
pub const LinkGeometry = types.LinkGeometry;
pub const LinkId = types.LinkId;
pub const LinkDimensions = types.LinkDimensions;
pub const CollisionResult = types.CollisionResult;
pub const CollisionConfig = types.CollisionConfig;

// Re-export commonly used types for convenience
pub const PIDController = control.PIDController;
pub const MotionPlanner = control.MotionPlanner;
pub const MotionConfig = control.MotionConfig;
pub const PIDConfig = control.PIDConfig;
pub const ControlConfig = control.ControlConfig;
pub const SafetyMonitor = safety.SafetyMonitor;
pub const Error = error_handler.Error;
pub const ErrorContext = error_handler.ErrorContext;
pub const ForwardKinematics = kinematics.ForwardKinematics;
pub const TransformMatrix = kinematics.TransformMatrix;
pub const Vec3 = types.Vec3;
pub const JointAngles = types.JointAngles;
pub const InverseKinematics = kinematics.InverseKinematics; 