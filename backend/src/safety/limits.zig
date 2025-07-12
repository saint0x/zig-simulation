const std = @import("std");

pub const JointLimits = struct {
    min_angle: f32,
    max_angle: f32,
    
    pub fn init(min: f32, max: f32) JointLimits {
        return JointLimits{
            .min_angle = min,
            .max_angle = max,
        };
    }
    
    pub fn isWithinLimits(self: JointLimits, angle: f32) bool {
        return angle >= self.min_angle and angle <= self.max_angle;
    }
};

pub const VelocityLimits = struct {
    max_velocity: f32,
    
    pub fn init(max: f32) VelocityLimits {
        return VelocityLimits{
            .max_velocity = max,
        };
    }
};

pub const AccelerationLimits = struct {
    max_acceleration: f32,
    
    pub fn init(max: f32) AccelerationLimits {
        return AccelerationLimits{
            .max_acceleration = max,
        };
    }
}; 