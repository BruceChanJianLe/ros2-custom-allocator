# ROS2 Custom Allocator

A simple publisher and subscriber that uses the standard memory resource.

## Usage

With monotonic resource:  
```bash
ros2 run ros2-custom-allocator pmr_simple_example mono
```

With unsync pool resource:  
```bash
ros2 run ros2-custom-allocator pmr_simple_example
```
