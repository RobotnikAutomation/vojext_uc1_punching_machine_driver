#______Column data Only Read address
int32 bar_code_1
int32 bar_code_2
int32 bar_code_3
int32 bar_code_4
int32 software_version
int32 hardware_version
int32 serial_number_1
int32 serial_number_2
int32 serial_number_3
int32 serial_number_4
#______Column state Only Read address
int32 elevation                  #mm/10
int32 motor_status               #Bit_0 1 when_moving #Bit_1 1 when_moving_external #Bit_2 1 when_moving_internal
int32 pilar_speed                #mm/s
int32 alarm                      #
int32 motor_working_hour         #hours
int32 motor_working_minutes      #minutes
int32 board_working_hour         #hours
int32 board_working_minutes      #minutes
int32 odometer_1                 #cm
int32 odometer_2
int32 S3_service_limit           #seconds
int32 S3_service_counter         #seconds
int32 S3_service_rest            #seconds
int32 input_voltage              #V
int32 heat_sink_temperature      #ºC
int32 motor_voltage              #V/10
int32 motor_current              #Arms
int32 motor_speed                #rpm
int32 overload_percentage        #%
#_______Read & write address            
int32 elevation_target             #0-10000 mm/10 
int32 commands                     #0-No commands //1-Stop all movements //2-Move internal //3-Move external //4-Search Internal //5-Search external
int32 motor_acceleration           #3-100 s/10 (5)
int32 movement_speed               #10-10 % (85)
