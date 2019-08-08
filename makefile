mavlink_control: mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread


