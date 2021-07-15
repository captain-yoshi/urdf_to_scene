# Name overlap bug 

To reproduce the issue:

Add these lines in the `moveit_resources_panda_description` package `/panda_description/urdf/panda.urdf`
```xml
  <link name="world"/>
  <joint name="world_joint" type="fixed">
    <parent link="world"/>
    <child link="panda_link0"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
  </joint>

```

Load the panda URDF as a collision object:
```shell
roslaunch moveit_resources_panda_moveit_config demo.launch

roslaunch scene_parser load_urdf_robot.launch
```

Targeting another panda urdf in the `load_urdf_robot.launch` with other link names (so that they do not overlap) fixes the issue.
