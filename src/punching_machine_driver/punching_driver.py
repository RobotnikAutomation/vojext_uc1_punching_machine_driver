#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ctypes import sizeof
from time import sleep
from rcomponent.rcomponent import *

# Insert here general imports:
import math
import threading 

# Insert here msg and srv imports:
from std_msgs.msg import String
from std_msgs.msg import Int16

from robotnik_msgs.msg import StringStamped
from punching_machine_driver.msg import punching_data

from std_srvs.srv import Trigger, TriggerResponse
from robotnik_msgs.srv import set_modbus_register, set_modbus_registerResponse
from robotnik_msgs.srv import get_modbus_register, get_modbus_registerResponse
from punching_machine_driver.srv import punching_machine_data, punching_machine_dataResponse
from punching_machine_driver.srv import set_modbus_coil, set_modbus_coilResponse
from punching_machine_driver.srv import get_modbus_coil, get_modbus_coilResponse


#Modbus
from pyModbusTCP.client import ModbusClient

class punchingMachineDriver(RComponent):
    """
    driver for sir lif
    """


    def __init__(self):
        self.unit_id = 1
        self.timeout = 1.0
        self.modbus_in_use = False

        self.lock = threading.Lock()

        #Only Read address
        self.start_reading_data = 16384


        #Read & write address
        self.start_rw_data = 16512
        self.pillowLoaded = 16512

        RComponent.__init__(self)


    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)
        self.host = rospy.get_param('~punching_ip')
        self.port = rospy.get_param('~punching_port')


    def ros_setup(self):
        """Creates and inits ROS components"""

        self.modbus_module = ModbusClient()
        self.modbus_module.host(self.host)
        self.modbus_module.port(self.port)
        self.modbus_module.unit_id(self.unit_id)
        self.modbus_module.timeout(self.timeout)

        if not self.modbus_module.is_open():
            if not self.modbus_module.open():
                rospy.logerr("Unable to open modbus client")

        RComponent.ros_setup(self)

        # Publisher
        self.status_pub = rospy.Publisher('~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher('~status_stamped', StringStamped, queue_size=10)
        
        self.punching_state_pub = rospy.Publisher('~punching_state', punching_data, queue_size=10)

        self.set_modbus_register_service = rospy.Service('~set_modbus_register', set_modbus_register, self.set_modbus_register_cb)
        self.get_modbus_register_service = rospy.Service('~get_modbus_register', get_modbus_register, self.get_modbus_register_cb)
        self.set_modbus_coil_service = rospy.Service('~set_modbus_coil', set_modbus_coil, self.set_modbus_coil_cb)
        self.get_modbus_coil_service = rospy.Service('~get_modbus_coil', get_modbus_coil, self.get_modbus_coil_cb)
        self.set_target_position_service = rospy.Service('~set_pillowLoaded', punching_machine_data, self.set_punching_state_cb)


        return 0


    def init_state(self):
        self.status = String()
        return RComponent.init_state(self)


    def ready_state(self):
        """Actions performed in ready state"""

        # Publish topic with status

        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        punching_status = punching_data()

        if self.is_open_modbus_lock():  
            read_column_data = self.read_modbus_coil_lock(self.start_reading_data, 1)
            read_rw_params = self.read_modbus_coil_lock(self.start_rw_data, 1)

            #Only Read column data
            #print(read_column_data)
            #print(read_rw_params)
            #if(read_column_data==None or read_rw_params==None):
            #    print("no data")
            #else:
            try:
                punching_status.pillowFinished = read_column_data[0]
                
                #Read & write address            
                punching_status.pillowLoaded = read_rw_params[0]

                self.punching_state_pub.publish(punching_status)
            except:
                rospy.logwarn("Missed data")
        else:
            rospy.logerr("Reconecting MODBUS")
            self.open_modbus_lock()
        
        return RComponent.ready_state(self)


    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)


    def shutdown(self):
        self.close_modbus_lock()
        """Shutdowns device
        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)


    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)


    def set_modbus_register_cb(self, msg):
        response = set_modbus_registerResponse()
        response.ret = self.write_modbus_lock(msg.address, msg.value)
        return response


    def get_modbus_register_cb(self, msg):
        response = get_modbus_registerResponse()
        print(self.read_modbus_lock(msg.address,1))
        response.ret = True
        return response


    def set_modbus_coil_cb(self, msg):
        response = set_modbus_coilResponse()
        response.ret = self.write_modbus_coil_lock(msg.address, msg.value)
        return response


    def get_modbus_coil_cb(self, msg):
        response = get_modbus_coilResponse()
        response.value = self.read_modbus_coil_lock(msg.address,1)
        response.ret = True
        return response


    def set_punching_state_cb(self, msg):
        response = punching_machine_dataResponse()
        response.ret = self.write_modbus_coil_lock(self.pillowLoaded, msg.value)
        return response


    # Locked MODBUS functions
    def write_modbus_lock(self, address, value):
        self.lock.acquire()
        response = self.modbus_module.write_single_register(address, value)
        self.lock.release()
        return response


    def read_modbus_lock(self, address, n_registers):
        self.lock.acquire()
        response = self.modbus_module.read_input_registers(address, n_registers)
        self.lock.release()
        return response


    def read_modbus_coil_lock(self, address, n_registers):
        self.lock.acquire()
        response = self.modbus_module.read_coils(address, n_registers)
        self.lock.release()
        return response


    def write_modbus_coil_lock(self, address, value):
        self.lock.acquire()
        response = self.modbus_module.write_single_coil(address, value)
        self.lock.release()
        return response


    def is_open_modbus_lock(self):
        self.lock.acquire()
        response = self.modbus_module.is_open()
        self.lock.release()
        return response


    def open_modbus_lock(self):
        self.lock.acquire()
        response = self.modbus_module.open()
        self.lock.release()
        return response


    def close_modbus_lock(self):
        self.lock.acquire()
        response = self.modbus_module.close()
        self.lock.release()
        return response
