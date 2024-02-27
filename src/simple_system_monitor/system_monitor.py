#!/usr/bin/env python3
"""simple_system_monitor ROS node"""

import time
import psutil
import rospy
from rcomponent.rcomponent import RComponent  # pylint: disable=import-error, no-name-in-module
from robotnik_msgs.msg import SimpleSystemStatus


class SimpleSystemMonitor(RComponent):  # pylint: disable=too-many-instance-attributes
    """SimpleSystemMonitor class, which contains all the logic of the simple_system_monitor ROS node"""

    def __init__(self):
        RComponent.__init__(self)
        self.logger = RComponent.initialize_logger(self)

        self.disk_capacity = -1.
        self.disk_usage = -1.
        self.memory_capacity = -1.
        self.memory_usage = -1.
        self.cpu_usage = -1.
        self.cpu_temperature = -1.
        self.core_temperatures = []
        self.simple_system_status_publisher = None

    def ros_read_params(self) -> None:
        """Gets params from param server"""
        RComponent.ros_read_params(self)

    def ros_setup(self) -> None:
        """Creates and inits ROS components"""
        RComponent.ros_setup(self)

        self.simple_system_status_publisher = rospy.Publisher(
            '~system_status', SimpleSystemStatus, queue_size=10)

    def init_state(self) -> None:
        """Actions performed in init state"""
        return RComponent.init_state(self)

    def publish_status(self):
        """Publishes the status"""

        system_status = SimpleSystemStatus()
        system_status.disk_capacity = self.disk_capacity
        system_status.disk_usage = self.disk_usage
        system_status.memory_capacity = self.memory_capacity
        system_status.memory_usage = self.memory_usage
        system_status.cpu_usage = self.cpu_usage
        system_status.core_temperatures = self.core_temperatures
        system_status.cpu_temperature = self.cpu_temperature
        system_status.header.stamp = rospy.Time.now()

        self.simple_system_status_publisher.publish(system_status)

    def ready_state(self):
        """Actions performed in ready state"""
        self.update_system_status()
        self.publish_status()

    def update_system_status(self):
        """Updates the system status readings"""
        # Update disk capacity and usage
        self.disk_capacity = psutil.disk_usage('/').total / 1e9
        self.disk_usage = psutil.disk_usage('/').percent

        # Update memory capacity and usage
        self.memory_capacity = psutil.virtual_memory().total / 1e9
        self.memory_usage = psutil.virtual_memory().percent

        # Update CPU usage and temperature
        self.cpu_usage = psutil.cpu_percent()
        cpu_temperatures = {}
        try:
            for sensor_data in psutil.sensors_temperatures()['coretemp']:
                if 'core' in sensor_data.label.lower():
                    cpu_temperatures[sensor_data.label] = sensor_data.current
                elif 'Package id 0' == sensor_data.label:
                    self.cpu_temperature = sensor_data.current
        except KeyError:
            self.cpu_temperature = 0.0

        # Order the dictionary as list to get the temperature for ordered cores
        keys = [int(key[5:]) for key in cpu_temperatures]
        keys.sort()
        keys = [f'Core {key}' for key in keys]
        cpu_temperatures = {i: cpu_temperatures[i] for i in keys}

        self.core_temperatures = list(cpu_temperatures.values())
