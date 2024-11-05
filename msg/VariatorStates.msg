uint8 status # смотри константы ниже
string error_message
float64 battery_voltage # в вольтах 
uint8 battery_charge # в процентах
uint8 led_state # смотри константы ниже (можно представить что это bool)
float64 variator_ratio 

# status 
uint8 OK = 0
uint8 Warning = 1
uint8 Error = 2

# led
uint8 OFF = 0
uint8 ON = 1

