const std = @import("std");

pub const TaskPriority = enum(u8) { low = 0, medium = 1, high = 2 };

pub const Task = struct {
    name: []const u8,
    priority: TaskPriority,
    interval: i64,
    last_run: i64,
    
    pub fn init(name: []const u8, priority: TaskPriority, interval: i64) Task {
        return Task{
            .name = name,
            .priority = priority,
            .interval = interval,
            .last_run = 0,
        };
    }
    
    pub fn shouldRun(self: *const Task, current_time: i64) bool {
        return (current_time - self.last_run) >= self.interval;
    }
};

pub const TaskScheduler = struct {
    allocator: std.mem.Allocator,
    tasks: std.ArrayList(Task),
    
    pub fn init(allocator: std.mem.Allocator) !TaskScheduler {
        return TaskScheduler{
            .allocator = allocator,
            .tasks = std.ArrayList(Task).init(allocator),
        };
    }
    
    pub fn deinit(self: *TaskScheduler) void {
        self.tasks.deinit();
    }
    
    pub fn addTask(self: *TaskScheduler, task: Task) !void {
        try self.tasks.append(task);
    }
    
    pub fn update(self: *TaskScheduler, current_time: i64) void {
        for (self.tasks.items) |*task| {
            if (task.shouldRun(current_time)) {
                task.last_run = current_time;
                // TODO: Execute task
            }
        }
    }
}; 