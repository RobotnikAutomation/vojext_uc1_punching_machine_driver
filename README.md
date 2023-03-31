# Punching machine driver

ROS driver for UC1 punching PEMU machine 

Note:
It is needed to install the pyModbusTCP library as follows (this version is required):

sudo pip install pyModbusTCP==v0.1.10

## MODBUS-TCP address

test punching machine
PLC1      IP addrress is: 192.168.17.195 (testing)
PLC2      IP addrress is: 192.168.17.196

### Only Read register

        punched_pillow = 16384


### Read and write registers

        pillow_ready = 16385

## Topics

        ~punching_state (punching_machine_driver/punching_data)


## Services

        ~set_pillowLoaded (punching_machine_driver/punching_machine_data)
        ~set_modbus_register (robotnik_msgs/set_modbus_register)
        ~get_modbus_register (robotnik_msgs/get_modbus_register)
        ~set_modbus_coil (punching_machine_driver/set_modbus_coil)
        ~get_modbus_coil (punching_machine_driver/get_modbus_coil)
