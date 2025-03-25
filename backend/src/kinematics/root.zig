const std = @import("std");

// Kinematics initialization and system
const init_module = @import("init.zig");
pub const KinematicsSystem = init_module.KinematicsSystem;
pub const init = init_module.init;

// Forward kinematics
pub const forward_kinematics = @import("forward_kinematics.zig");

// Inverse kinematics
pub const inverse_kinematics = @import("inverse_kinematics.zig");

// Collision detection
pub const collision_detection = @import("collision_detection.zig");

// Re-export commonly used types
pub const ForwardKinematics = forward_kinematics.ForwardKinematics;
pub const InverseKinematics = inverse_kinematics.InverseKinematics;
pub const CollisionDetection = collision_detection.CollisionDetection;
pub const TransformMatrix = forward_kinematics.TransformMatrix; 