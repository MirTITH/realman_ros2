# Realman ROS2 Packages

该仓库包含了睿尔曼机械臂的 ROS2 包。其中，rm_xacro_description 中的网格模型和描述文件来自 <https://github.com/RealManRobot/ros2_rm_robot>

## 注意事项

本仓库使用的 MoveIt 由 main 分支源码编译得到

其中的 xxx_moveit_autogen 包是由 MoveIt Setup Assistant 生成的。然而，直接生成的包并不能直接运行，需要做如下修改：

1. 在 config/moveit_controllers.yaml 中添加 `action_ns`，例如：
```yaml
# ...
  rm_manipulator_controller:
    type: FollowJointTrajectory
    action_ns: "follow_joint_trajectory"
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7
# ...
```

2. 在 config/joint_limits.yaml 中添加加速度限制，例如：
```yaml
# ...
joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 3.1400000000000001
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint2:
    # ...
# ...
```
4. 
