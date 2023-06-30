#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ctypes import sizeof
from time import sleep
from rcomponent.rcomponent import *
import yaml

# Insert here general imports:
import math
import threading 

# Insert here msg and srv imports:
from std_msgs.msg import String
from std_msgs.msg import Int16

from robotnik_msgs.msg import StringStamped
from robotnik_msgs.msg import named_input_output
from punching_machine_driver.msg import modbus_input_output
from punching_machine_driver.msg import named_modbus_inputs_outputs

from std_srvs.srv import Trigger, TriggerResponse
from robotnik_msgs.srv import set_modbus_register, set_modbus_registerResponse
from robotnik_msgs.srv import get_modbus_register, get_modbus_registerResponse
from punching_machine_driver.srv import set_modbus_coil, set_modbus_coilResponse
from punching_machine_driver.srv import get_modbus_coil, get_modbus_coilResponse
from punching_machine_driver.srv import set_named_modbus, set_named_modbusResponse


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

        
        self.punching_state_pub = rospy.Publisher('~punching_state', named_modbus_inputs_outputs, queue_size=10)

        self.set_modbus_register_service = rospy.Service('~set_modbus_register', set_modbus_register, self.set_modbus_register_cb)
        self.get_modbus_register_service = rospy.Service('~get_modbus_register', get_modbus_register, self.get_modbus_register_cb)
        self.set_modbus_coil_service = rospy.Service('~set_modbus_coil', set_modbus_coil, self.set_modbus_coil_cb)
        self.get_modbus_coil_service = rospy.Service('~get_modbus_coil', get_modbus_coil, self.get_modbus_coil_cb)
#        self.set_target_position_service = rospy.Service('~set_pillowLoaded', punching_machine_data, self.set_punching_state_cb)
        self.set_named_modbus_signal = rospy.Service('~set_named_signal', set_named_modbus, self.set_named_modbus_signal_cb)


        return 0


    def init_state(self):
        self.status = String()
        self.yaml_registers_rs = rospy.get_param('~MODBUS_REGISTERS')
        self.yaml_registers_cb = rospy.get_param('~MODBUS_REGISTERS')
        return RComponent.init_state(self)


    def ready_state(self):

        punching_status = named_modbus_inputs_outputs()
        appended_data = []

        if self.is_open_modbus_lock():

            try:
                
                for register in self.yaml_registers_rs:
                    if register['permission']<=1:
                        register_status = modbus_input_output()
                        register_status.name = register['name']
                        if register['type'] == 0:
                            value = self.read_modbus_coil_lock(register['register'],register['lenght'])
                        else:
                            value = self.read_modbus_lock(register['register'],register['lenght'])
                        register_status.value = value
                        appended_data.append(register_status)
                read_result = True
            except:
                rospy.logerr("%s::ready_state: ERROR READING" % (self._node_name))
                read_result = False
            if read_result:
                try:
                    punching_status.named_registers = appended_data
                    self.punching_state_pub.publish(punching_status)
                except:
                    rospy.logerr("%s::ready_state: ERROR PUBLISHING" % (self._node_name))
        else:
            rospy.logerr("Reconecting MODBUS")
            self.open_modbus_lock()
        
        return RComponent.ready_state(self)


    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)


    def shutdown(self):
        self.close_modbus_lock()
        return RComponent.shutdown(self)


    def switch_to_state(self, new_state):
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


    def set_named_modbus_signal_cb(self, msg):
        response = set_named_modbusResponse()
        for signal in self.yaml_registers_cb:
            if signal['name'] == msg.name:
                if signal['permission'] >= 1:
                    response.msg = ''
                    if signal['type'] == 0:
                        value_list = [True if x > 0 else False for x in msg.value]
                        response.ret = self.write_modbus_coil_lock(signal['register'], value_list)
                    elif signal['type'] == 1:
                        value_list = list(map(int, msg.value))
                        response.ret = self.write_modbus_coil_lock(signal['register'], value_list)
                    elif signal['type'] == 2:
                        value_list = msg.value
                        response.ret = self.write_modbus_coil_lock(signal['register'], value_list)
                    else:
                        logmsg = ("%s::set_named_modbus_signal_cb: not valid type" % (self._node_name))
                        response.msg = 'not valid type'
                        response.ret = False
                    return response
                else:
                    logmsg = ("%s::set_named_modbus_signal_cb: not write permissions" % (self._node_name))
                    response.msg = 'not write permissions'
                    response.ret = False
                    return response
            else:
                logmsg = ("%s::set_named_modbus_signal_cb: missng signal name" % (self._node_name))
                response.msg = 'missng signal name'
                response.ret = False
        rospy.logerr(logmsg)
        return response


    # Locked MODBUS functions
    def write_modbus_lock(self, address, value):
        self.lock.acquire()
        response = self.modbus_module.write_multiple_registers(address, value)
        self.lock.release()
        if type(response) != type(True):
            response = False
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
        response = self.modbus_module.write_multiple_coils(address, value)
        self.lock.release()
        if type(response) != type(True):
            response = False
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
