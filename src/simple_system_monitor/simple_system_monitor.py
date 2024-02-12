#!/usr/bin/env python3
import threading
import psutil
import time
import rospy
from rcomponent.rcomponent import RComponent
from robotnik_msgs.msg import SimpleSystemStatus


class SimpleSystemMonitor(RComponent):

    def __init__(self):
        RComponent.__init__(self)
        self.logger = RComponent.initialize_logger(self)

        self.disk_capacity = 0.
        self.disk_usage = 0.
        self.memory_capacity = 0.
        self.memory_usage = 0.
        self.cpu_usage = 0.
        self.cpu_temperatures = dict()

    def ros_read_params(self) -> None:
        """"""
        RComponent.ros_read_params(self)

        self.update_hz = rospy.get_param('update_hz', '2')

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
        system_status.cpu_temperature = self.cpu_temperatures

        self.simple_system_status_publisher.publish(system_status)

    def ready_state(self):
        """"""
        RComponent.ready_state(self)

        publish_system_status_thread = threading.Thread(target=self.publish_system_status, daemon=True)
        publish_system_status_thread.start()

    def publish_system_status(self):
        """"""
        while True:
            self.update_system_status()
            self.ros_publish()
            time.sleep(1 / self.update_hz)
            break
    
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

        # Order the dictionary as list to get the temperature for ordered cores
        keys = [int(key[5:]) for key in cpu_temperatures.keys()]
        keys.sort()
        keys = [f'Core {key}' for key in keys]
        cpu_temperatures = {i: cpu_temperatures[i] for i in keys}

        self.cpu_temperatures = [cpu_temperature for cpu_temperature in cpu_temperatures.values()]
