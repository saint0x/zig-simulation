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