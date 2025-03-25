const std = @import("std");
const core = @import("core");

// Import submodules
pub const pid = @import("pid.zig");
pub const joint_manager = @import("joint_manager/root.zig");
pub const motion_planning = @import("motion_planning.zig");
pub const types = @import("types.zig");

// Re-export commonly used types
pub const PIDController = pid.PIDController;
pub const JointManager = joint_manager.JointManager;
pub const MotionPlanner = motion_planning.MotionPlanner;
pub const MotionConfig = types.MotionConfig;
pub const PIDConfig = pid.PIDConfig;
pub const ControlConfig = struct {
    joint_configs: [NUM_JOINTS]JointConfig,
    pid_configs: [NUM_JOINTS]PIDConfig,
    motion_config: motion_planning.MotionConfig,
};

pub const JointConfig = joint_manager.JointConfig;
pub const JointState = joint_manager.JointState;
pub const JointId = joint_manager.JointId;
pub const RobotState = joint_manager.RobotState;
pub const NUM_JOINTS = joint_manager.NUM_JOINTS;

pub const init = joint_manager.init;
pub const updateStates = joint_manager.updateStates;
pub const getJointState = joint_manager.getJointState;
pub const getRobotState = joint_manager.getRobotState;
pub const hasReachedTarget = joint_manager.hasReachedTarget;
pub const update = joint_manager.update;
pub const setTargets = joint_manager.setTargets;
pub const reset = joint_manager.reset; 