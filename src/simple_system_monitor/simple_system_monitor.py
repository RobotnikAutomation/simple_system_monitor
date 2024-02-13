#!/usr/bin/env python3
import psutil
import time
import rospy
from rcomponent.rcomponent import RComponent
from robotnik_msgs.msg import SimpleSystemStatus


class SimpleSystemMonitor(RComponent):

    def __init__(self):
        RComponent.__init__(self)
        self.logger = RComponent.initialize_logger(self)

        self.disk_capacity = -1.
        self.disk_usage = -1.
        self.memory_capacity = -1.
        self.memory_usage = -1.
        self.cpu_usage = -1.
        self.cpu_temperature = -1.
        self.core_temperatures = list()

    def ros_read_params(self) -> None:
        """"""
        RComponent.ros_read_params(self)

    def ros_setup(self) -> None:
        """"""
        RComponent.ros_setup(self)

        self.simple_system_status_publisher = rospy.Publisher('~system_status', SimpleSystemStatus, queue_size=10)

    def init_state(self) -> None:
        """
        """
        return RComponent.init_state(self)
    
    def ros_publish(self):
        """"""
        RComponent.ros_publish(self)

        system_status = SimpleSystemStatus()
        system_status.disk_capacity = self.disk_capacity
        system_status.disk_usage = self.disk_usage
        system_status.memory_capacity = self.memory_capacity
        system_status.memory_usage = self.memory_usage
        system_status.cpu_usage = self.cpu_usage
        system_status.core_temperatures = self.core_temperatures
        system_status.cpu_temperature = self.cpu_temperature
        system_status.timestamp = str(time.time())

        self.simple_system_status_publisher.publish(system_status)

    def ready_state(self):
        """"""
        RComponent.ready_state(self)

        self.update_system_status()
    
    def update_system_status(self):
        """"""
        # Update disk capacity and usage
        self.disk_capacity = psutil.disk_usage('/').total / 1e9
        self.disk_usage = psutil.disk_usage('/').percent

        # Update memory capacity and usage
        self.memory_capacity = psutil.virtual_memory().total / 1e9
        self.memory_usage = psutil.virtual_memory().percent

        # Update CPU usage and temperature
        # TODO: check psutil outputs for non-INtel CPUs
        self.cpu_usage = psutil.cpu_percent()
        cpu_temperatures = dict()
        for sensor_data in psutil.sensors_temperatures()['coretemp']:
            if 'core' in sensor_data.label.lower():
                cpu_temperatures[sensor_data.label] = sensor_data.current
            elif 'Package id 0' == sensor_data.label:
                self.cpu_temperature = sensor_data.current

        # Order the dictionary as list to get the temperature for ordered cores
        keys = [int(key[5:]) for key in cpu_temperatures.keys()]
        keys.sort()
        keys = [f'Core {key}' for key in keys]
        cpu_temperatures = {i: cpu_temperatures[i] for i in keys}

        self.core_temperatures = [cpu_temp for cpu_temp in cpu_temperatures.values()]
