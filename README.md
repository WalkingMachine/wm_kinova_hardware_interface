# wm_kinova_hardware_interface

This Package is a plugin for combined hardware interface.
It provide a Hardware interface as a plugin so no launchfile is needed.

### Installation
```sh
cd <this package>
sudo ./install.sh
apt-get install ros-kinetic-ros-control
```

### Config file example
```yaml
robot_hardware:
  - MyKinovaJoint1

MyKinovaJoint1:
  type: wm_kinova_hardware_interface/WMKinovaHardwareInterface
  index: 0  # The address of the actuator.
  offset: -3.14159
  speed_ratio: 1
  complience_level: 2 # Set to 0 for maximum stifness.
  complience_threshold: 12
  complience_derivation_factor: 0.03
  complience_loss_factor: 0.75
  complience_resistance: 0.2
  joints:
    - kinova_joint_1
```
