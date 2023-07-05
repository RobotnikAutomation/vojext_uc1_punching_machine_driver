# Punching machine driver

ROS driver for UC1 punching PEMU machine 

Note:
It is needed to install the pyModbusTCP library as follows (this version is required):

sudo pip install pyModbusTCP==v0.1.10

## MODBUS-TCP address

test punching machine
PLC1      IP addrress is: 192.168.17.195 (testing)
PLC2      IP addrress is: 192.168.17.196

## Launch
By default the IP is 192.168.17.195 and takes the data from "modbusSignals.yaml"

        roslaunch punching_machine_driver punching_driver.launch

If we want to connect with two devices we need to launch

                roslaunch punching_machine_driver two_punching_driver.launch enable_pm2:=true
this launcher will take the registers from the "modbusSignalsPM1.yaml" and "modbusSignalsPM2.yaml"


### Registers

The registers are available at config/modbusSignals.yaml


## Topics

#### ~punching_state (punching_machine_driver/punching_data)
will provide a list with the name of the registers and a list with the values



## Services

#### ~set_named_signal (punching_machine_driver/set_named_modbus)
##### Example
        rosservice call /punching_machine_driver/set_named_signal "name: 'pillowCanBeTaken'
        value: [0]" 

#### ~set_modbus_register (robotnik_msgs/set_modbus_register)
##### Example
        rosservice call /punching_machine_driver/set_modbus_register "address: 1    
        value: 65535" 

#### ~get_modbus_register (robotnik_msgs/get_modbus_register)
##### Example
        rrosservice call /punching_machine_driver/get_modbus_register "address: 1"
 
#### ~set_modbus_coil (punching_machine_driver/set_modbus_coil)
##### Example
        rosservice call /punching_machine_driver/set_modbus_coil "address: 16384
        value: false" 

#### ~get_modbus_coil (punching_machine_driver/get_modbus_coil)
##### Example
        rosservice call /punching_machine_driver/get_modbus_coil "address: 16389"

## msg

#### modbus_input_output
                string name
                float32[] value

#### named_modbus_inputs_outputs
                punching_machine_driver/modbus_input_output[] named_registers

## srv

#### get_modbus_coil
                int32 address
                ---
                bool ret 
                bool value

#### set_modbus_coil
                int32 address
                bool value
                ---
                bool ret 

#### set_named_modbus
                string name
                float32[] value
                ---
                bool ret
                string msg
