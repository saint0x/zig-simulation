const std = @import("std");

/// Number of joints in our KUKA arm
pub const NUM_JOINTS = 7;

/// Joint identifiers for the KUKA arm
pub const JointId = enum(u8) {
    base_rotation = 0,
    shoulder_rotation = 1,
    elbow_rotation = 2,
    wrist_bend = 3,
    wrist_rotation = 4,
    tool_rotation = 5,
    gripper = 6,
};

/// Joint configuration parameters
pub const JointConfig = struct {
    /// Joint identifier
    id: JointId,
    /// Minimum joint angle in degrees
    min_angle: f32,
    /// Maximum joint angle in degrees
    max_angle: f32,
    /// Maximum joint velocity in degrees/second
    max_velocity: f32,
    /// Maximum joint acceleration in degrees/second²
    max_acceleration: f32,
    /// Control loop time step in seconds
    dt: f32,
    /// PID controller gains
    pid_gains: struct {
        kp: f32,  // Proportional gain
        ki: f32,  // Integral gain
        kd: f32,  // Derivative gain
        i_max: f32,  // Maximum integral term
    },
};

/// Joint state information
pub const JointState = struct {
    /// Current joint angle in degrees
    current_angle: f32,
    /// Current joint velocity in degrees/second
    current_velocity: f32,
    /// Target joint angle in degrees
    target_angle: f32,
    /// Target joint velocity in degrees/second
    target_velocity: f32,
    /// Current joint torque in Nm
    current_torque: f32,
    /// Target joint torque in Nm
    target_torque: f32,
    /// Motor temperature in degrees Celsius
    temperature: f32,
    /// Motor current in Amperes
    current: f32,
    /// Integral term for PID control
    integral_term: f32,
    /// Last error for derivative term
    last_error: f32,
};

/// Robot operational state
pub const RobotState = enum {
    /// Robot is powered off
    powered_off,
    /// Robot is in emergency stop state
    emergency_stop,
    /// Robot is ready for operation
    ready,
    /// Robot is executing a movement
    moving,
    /// Robot has encountered an error
    fault,
};

/// Control modes for joints
pub const ControlMode = enum {
    position,
    velocity,
    torque,
};

/// Complete robot state information
pub const RobotStatus = enum {
    idle,
    moving,
    fault,
    emergency_stop,
    disabled,
};

/// Cartesian coordinates (x, y, z) in millimeters
pub const CartesianPosition = struct {
    x: f32,
    y: f32,
    z: f32,
};

/// Orientation in Euler angles (roll, pitch, yaw) in degrees
pub const Orientation = struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
};

/// Complete end-effector pose
pub const EndEffectorPose = struct {
    /// Position in Cartesian coordinates
    position: CartesianPosition,
    /// Orientation in Euler angles
    orientation: Orientation,
};

/// Represents a 3D bounding box for collision detection
pub const BoundingBox = struct {
    /// Minimum point (x, y, z)
    min: CartesianPosition,
    /// Maximum point (x, y, z)
    max: CartesianPosition,
};

/// Represents a robot link for collision detection
pub const LinkGeometry = struct {
    /// Link identifier
    id: u8,
    /// Bounding box for the link
    bounds: BoundingBox,
    /// Current position of the link's center
    center: CartesianPosition,
    /// Current orientation of the link
    orientation: Orientation,
};

/// Link identifiers for collision detection
pub const LinkId = enum(u8) {
    base = 0,
    shoulder = 1,
    upper_arm = 2,
    elbow = 3,
    forearm = 4,
    wrist_rotation = 5,
    wrist_bend = 6,
    tool_rotation = 7,
    gripper = 8,
};

/// Link dimensions for collision detection
pub const LinkDimensions = struct {
    /// Link identifier
    id: LinkId,
    /// Length of the link in millimeters
    length: f32,
    /// Width of the link in millimeters
    width: f32,
    /// Height of the link in millimeters
    height: f32,
    /// Joint offset from previous link in millimeters
    joint_offset: CartesianPosition,
};

/// Collision detection result
pub const CollisionResult = struct {
    /// Whether a collision was detected
    collision_detected: bool,
    /// Type of collision (self or environment)
    collision_type: enum {
        none,
        self_collision,
        environment_collision,
    },
    /// First colliding link
    link1: ?LinkId,
    /// Second colliding link (if self collision)
    link2: ?LinkId,
    /// Collision point in world coordinates
    collision_point: ?CartesianPosition,
    /// Minimum distance to collision (if no collision, distance to nearest obstacle)
    min_distance: f32,
    /// Joint angles at time of collision
    joint_angles: [NUM_JOINTS]f32,
};

/// Collision detection configuration
pub const CollisionConfig = struct {
    /// Minimum allowed distance between links for self-collision
    min_link_distance: f32,
    /// Minimum allowed distance from environment obstacles
    min_environment_distance: f32,
    /// Whether to enable continuous collision detection
    enable_continuous_detection: bool,
    /// Update frequency for collision checks (Hz)
    check_frequency: f32,
    /// Link dimensions for collision detection
    link_dimensions: [9]LinkDimensions,
}; 