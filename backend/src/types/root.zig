const std = @import("std");

pub const NUM_JOINTS = 6;

pub const JointId = u8;

pub const JointConfig = struct {
    id: JointId,
    min_position: f32,
    max_position: f32,
    max_velocity: f32,
    max_torque: f32,
    max_temperature: f32,
    max_current: f32,
    pid_gains: PIDGains,
};

pub const JointState = struct {
    id: JointId,
    position: f32,
    velocity: f32,
    torque: f32,
    temperature: f32,
    current: f32,
};

pub const RobotState = struct {
    joints: [NUM_JOINTS]JointState,
    configs: [NUM_JOINTS]JointConfig,
};

pub const PIDGains = struct {
    p: f32,
    i: f32,
    d: f32,
};

pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,
};

pub const JointAngles = [NUM_JOINTS]f32;

pub const CartesianPosition = Vec3;

pub const Orientation = struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
};

pub const EndEffectorPose = struct {
    position: CartesianPosition,
    orientation: Orientation,
};

pub const BoundingBox = struct {
    min: Vec3,
    max: Vec3,
};

pub const LinkGeometry = struct {
    type: LinkGeometryType,
    dimensions: LinkDimensions,
};

pub const LinkGeometryType = enum {
    box,
    cylinder,
    sphere,
};

pub const LinkId = u8;

pub const LinkDimensions = struct {
    width: f32,
    height: f32,
    depth: f32,
    radius: f32,
};

pub const CollisionResult = struct {
    detected: bool,
    link1_name: []const u8,
    link2_name: []const u8,
    position: Vec3,
    penetration_depth: f32,
    contact_normal: Vec3,
};

pub const CollisionConfig = struct {
    enabled: bool,
    check_frequency: f32,
    max_penetration: f32,
}; 