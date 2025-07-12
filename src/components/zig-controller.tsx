"use client"

import { useState } from "react"
import { Card, CardContent, CardTitle, CardHeader } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Code } from "lucide-react"

export function ZigController({ controlSignals, setControlSignals }) {
  const [showCode, setShowCode] = useState(false)

  return (
    <div
      className={`absolute ${showCode ? "bottom-4 right-4 w-96 h-96" : "bottom-4 right-4 w-auto"} bg-black/80 backdrop-blur-md rounded-lg text-white overflow-hidden transition-all duration-300`}
    >
      {!showCode ? (
        <Button
          variant="outline"
          size="sm"
          className="m-2 bg-transparent border-orange-600 text-orange-400 hover:bg-orange-950/30"
          onClick={() => setShowCode(true)}
        >
          <Code className="w-4 h-4 mr-2" />
          Zig Controller
        </Button>
      ) : (
        <Card className="border-0 bg-transparent text-white h-full">
          <CardHeader className="pb-2">
            <div className="flex items-center justify-between">
              <CardTitle className="text-lg font-mono">Zig Microcontroller</CardTitle>
              <Button variant="ghost" size="sm" className="h-6 text-xs" onClick={() => setShowCode(false)}>
                Close
              </Button>
            </div>
          </CardHeader>
          <CardContent className="overflow-auto h-[calc(100%-60px)]">
            <pre className="text-xs font-mono bg-black/50 p-3 rounded overflow-auto h-full">
              <code className="text-green-400">
                {`// KUKA Robot Arm Controller
// Implemented in Zig for memory safety and performance

const std = @import("std");
const microzig = @import("microzig");
const gpio = microzig.gpio;
const time = microzig.time;

// Motor control pins
const motor_pins = [_]u8{ 2, 3, 4, 5, 6, 7, 8 };

// Sensor input pins
const sensor_pins = [_]u8{ A0, A1, A2, A3, A4, A5, A6 };

// Status LEDs
const status_led = gpio.pin(13);
const error_led = gpio.pin(12);

// Motor current readings
var motor_currents: [7]f32 = undefined;

// Sensor readings
var sensor_values: [7]f32 = undefined;

// System status
var system_status: enum {
    READY,
    BUSY,
    ERROR,
    WARNING
} = .READY;

// Error codes
var error_code: ?[]const u8 = null;

// PID controller for each joint
const PIDController = struct {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    last_error: f32,
    integral: f32,
    
    pub fn init(kp: f32, ki: f32, kd: f32) PIDController {
        return PIDController{
            .kp = kp,
            .ki = ki,
            .kd = kd,
            .setpoint = 0,
            .last_error = 0,
            .integral = 0,
        };
    }
    
    pub fn compute(self: *PIDController, current: f32, dt: f32) f32 {
        const error = self.setpoint - current;
        self.integral += error * dt;
        const derivative = (error - self.last_error) / dt;
        self.last_error = error;
        
        return self.kp * error + self.ki * self.integral + self.kd * derivative;
    }
};

// Initialize PID controllers for each joint
var pid_controllers: [7]PIDController = undefined;

pub fn main() !void {
    // Initialize hardware
    try init();
    
    // Main control loop
    var last_time = time.millis();
    
    while (true) {
        const current_time = time.millis();
        const dt = @intToFloat(f32, current_time - last_time) / 1000.0;
        last_time = current_time;
        
        // Read sensor values
        try readSensors();
        
        // Check for errors
        try checkErrors();
        
        // Update motor controls
        try updateMotors(dt);
        
        // Update status LED
        updateStatusLED();
        
        // Wait for next cycle
        time.sleep(1); // 1ms delay for 1kHz control loop
    }
}

fn init() !void {
    // Initialize GPIO pins
    for (motor_pins) |pin| {
        gpio.setOutput(pin);
    }
    
    for (sensor_pins) |pin| {
        gpio.setInput(pin);
    }
    
    gpio.setOutput(status_led);
    gpio.setOutput(error_led);
    
    // Initialize PID controllers
    for (pid_controllers) |*pid| {
        pid.* = PIDController.init(2.0, 0.1, 0.5);
    }
    
    // Signal ready
    gpio.write(status_led, 1);
    time.sleep(500);
    gpio.write(status_led, 0);
}

fn readSensors() !void {
    for (sensor_pins) |pin, i| {
        const raw_value = gpio.analogRead(pin);
        sensor_values[i] = @intToFloat(f32, raw_value) / 1023.0 * 5.0;
        
        // Calculate motor current from sensor reading
        motor_currents[i] = sensor_values[i] * 2.0; // Simplified conversion
    }
}

fn checkErrors() !void {
    // Check for overheating
    for (motor_currents) |current, i| {
        if (current > 4.5) {
            system_status = .WARNING;
            error_code = "TEMP_HIGH";
            gpio.write(error_led, 1);
            return;
        }
    }
    
    // Check for stall conditions
    for (sensor_values) |value, i| {
        if (value < 0.1 && pid_controllers[i].setpoint != 0) {
            system_status = .ERROR;
            error_code = "MOTOR_STALL";
            gpio.write(error_led, 1);
            return;
        }
    }
    
    // No errors
    system_status = .READY;
    error_code = null;
    gpio.write(error_led, 0);
}

fn updateMotors(dt: f32) !void {
    for (motor_pins) |pin, i| {
        const pid_output = pid_controllers[i].compute(sensor_values[i], dt);
        const pwm_value = @floatToInt(u8, std.math.clamp(pid_output * 255.0, 0, 255));
        gpio.analogWrite(pin, pwm_value);
    }
}

fn updateStatusLED() void {
    switch (system_status) {
        .READY => {
            // Solid on
            gpio.write(status_led, 1);
        },
        .BUSY => {
            // Fast blink
            if (time.millis() % 200 < 100) {
                gpio.write(status_led, 1);
            } else {
                gpio.write(status_led, 0);
            }
        },
        .WARNING => {
            // Slow blink
            if (time.millis() % 1000 < 500) {
                gpio.write(status_led, 1);
            } else {
                gpio.write(status_led, 0);
            }
        },
        .ERROR => {
            // Double blink
            const t = time.millis() % 1000;
            if (t < 100 or (t > 300 and t < 400)) {
                gpio.write(status_led, 1);
            } else {
                gpio.write(status_led, 0);
            }
        },
    }
}`}
              </code>
            </pre>
          </CardContent>
        </Card>
      )}
    </div>
  )
}

