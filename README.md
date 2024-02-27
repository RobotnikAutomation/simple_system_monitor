# simple_system_monitor

The simple_system_monitor package, monitor for the properties and usage of the system's disk, memory and CPU.

## Installation

This packages depends on the following packages:

- robotnik_msgs[🔗](https://github.com/RobotnikAutomation/robotnik_msgs)

```
git clone https://github.com/RobotnikAutomation/robotnik_msgs
```

- rcomponent[🔗](https://github.com/RobotnikAutomation/rcomponent)

```
git clone https://github.com/RobotnikAutomation/rcomponent
```

Install the package:

```
git clone https://github.com/RobotnikAutomation/simple_system_monitor.git
```

Install other ros dependencies:

```
rosdep install --from-path src --ignore-src -y -r
```

Build the package
```
catkin build
source devel/setup.bash
```

## Package configuration

For the moment there are no configuration parameters available for this package.

## Bringup

Launch the package:

```
roslaunch simple_system_monitor simple_system_monitor.launch
```

## 1. simple_system_monitor

It publishes the system status

### 1.1 Published Topics

* simple_system_monitor/system_status (robotnik_msgs/SimpleSystemStatus):
  * System status, publishes the following info:
    * disk_capacity: float32
    * disk_usage: float32
    * memory_capacity: float32
    * memory_usage: float32
    * cpu_usage: float32
    * cpu_temperature: float32
    * core_temperatures: float32[]
    * timestamp: string
---