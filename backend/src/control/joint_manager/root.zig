const std = @import("std");
const types = @import("types.zig");
const init_mod = @import("init.zig");
const control_mod = @import("control.zig");
const state_mod = @import("state.zig");
const config_mod = @import("config.zig");
const safety_mod = @import("safety.zig");

pub const JointManager = types.JointManager;
pub const JointConfig = types.JointConfig;
pub const JointState = types.JointState;
pub const JointId = types.JointId;
pub const RobotState = types.RobotState;
pub const NUM_JOINTS = types.NUM_JOINTS;

pub const init = init_mod.init;
pub const updateStates = state_mod.updateStates;
pub const getJointState = state_mod.getJointState;
pub const getRobotState = state_mod.getRobotState;
pub const hasReachedTarget = control_mod.hasReachedTarget;
pub const update = control_mod.update;
pub const setTargets = control_mod.setTargets;
pub const reset = safety_mod.reset; 