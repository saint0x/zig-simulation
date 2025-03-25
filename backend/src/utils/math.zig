const std = @import("std");

pub fn lerp(a: f32, b: f32, t: f32) f32 {
    return a + (b - a) * t;
}

pub fn clamp(value: f32, min: f32, max: f32) f32 {
    return @min(@max(value, min), max);
}

pub fn radiansToDegrees(radians: f32) f32 {
    return radians * (180.0 / std.math.pi);
}

pub fn degreesToRadians(degrees: f32) f32 {
    return degrees * (std.math.pi / 180.0);
} 