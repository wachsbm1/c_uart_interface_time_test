C-UART Interface Time Test Example (derived from the C-UART Interface Example)
========================

This is a simple MAVLink to UART interface example for *nix systems that can allow communication between Pixhawk and an offboard computer. 

This example will receive one MAVLink message and send one MAVLink message. Furthermore, this example will also time how long it takes to send and receive the message. 


Building
========

```
$ cd c_uart_interface_example/
$ make
```
If the makefile does not work, try compiling with: 
g++ -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread

Hardware Setup
=========

Connect the USB programming cable to your Pixhawk.  

If you want to be able to interact with this example in Pixhawk's NuttX shell, you'll need a Telemetry Radio or an FTDI developer's cable.  See the Exploration section below for more detail.

Also Note: Using a UART (serial) connection should be preferred over using the USB port for flying systems.  The reason being that the driver for the USB port is much more complicated, so the UART is a much more trusted port for flight-critical functions.  To learn how this works though the USB port will be fine and instructive.

Execution
=========

You have to pick a port name, try searching for it with 
```
$ ls /dev/ttyACM* 
$ ls /dev/ttyUSB*
```

Alternatively, plug in Pixhawk USB cable again and issue the command:
```
$ dmesg
```
The device described at the bottom of dmesg's output will be the port on which the Pixhawk is mounted. 

The Pixhawk USB port will show up on a `ttyACM*`, an FTDI cable will show up on a `ttyUSB*`.


Run the example executable on the host shell:

```
$ cd c_uart_interface_time_test_example/
$ ./mavlink_control -d /dev/ttyACM0
```

To stop the program, use the key sequence `Ctrl-C`.

Here's an example output:

```
OPEN PORT
Connected to /dev/ttyACM0 with 57600 baud, 8 data bits, no parity, 1 stop bit (8N1)

START READ THREAD 

CHECK FOR MESSAGES
Found

GOT VEHICLE SYSTEM ID: 1
GOT AUTOPILOT COMPONENT ID: 1

INITIAL POSITION XYZ = [ 0.0000 , 0.0000 , 0.0000 ] 
INITIAL POSITION YAW = 0.0254 

START WRITE THREAD 

ENABLE OFFBOARD MODE

SEND OFFBOARD COMMANDS
Sent 13 bits
1) Sending message took: 119.461 microseconds
1) Sent message at: 8370.93 Hz

POSITION SETPOINT XYZ = [ -5.0000 , -5.0000 , 0.0000 ] 
POSITION SETPOINT YAW = 0.0254 
0 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 
1 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 
2 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 
3 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 
4 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 
5 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 
6 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 
7 CURRENT POSITION XYZ = [  0.0000 ,  0.0000 ,  0.0000 ] 

DISABLE OFFBOARD MODE

READ SOME MESSAGES 
1) Receiving messages took: 6.26964e+06 microseconds
1) Received messages at: 0.159499 Hz

Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)
    pos  (NED):  0.000000 0.000000 0.000000 (m)
Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)
    ap time:     0 
    acc  (NED):   0.000000  0.000000  0.000000 (m/s^2)
    gyro (NED):   0.000000  0.000000  0.000000 (rad/s)
    mag  (NED):   0.000000  0.000000  0.000000 (Ga)
    baro:        0.000000 (mBar) 
    altitude:    0.000000 (m) 
    temperature: 0.000000 C 

CLOSE THREADS
time_to_exit and pthread have been reached

CLOSE PORT
```

Exploration
===========

There are a few things to explore past this example.

First you can connect via:
* a [Telemetry Radio](https://docs.px4.io/en/telemetry/) on TELEM1 or 2
* an [FTDI cable](https://www.sparkfun.com/products/9718) on TELEM2 or Serial 4

> **Note** 
> * Serial 5 can't be used to receive data without reconfiguration (its receive pin is occupied by a second NuttX shell).
> * TELEM1 is typically dedicated to Telemetry Radio, but if you're using another port you will need to make sure it is not configured for use by another peripheral.
> * If using FTDI with a TELEM port, connect all the pins to corresponding pins on port.
> * If using FTDI with SERIAL4 connect the TX, RX GND and 5V pins (CTS, RTS need not be connected).


With this you'll be able to start a second port for communication, and leave the USB port available for viewing prints in the NuttX shell.  

For steps 2 and 3 from the above tutorial, you'll use a different port.  On the off-board computer side, the port might now be `/dev/ttyUSB0`.  On the Pixhawk side, here the port mappings are in the table below.

| PX4 UART | NuttX UART |
|----------|------------|
| Telem 1  | /dev/ttyS1 |
| Telem 2  | /dev/ttyS2 |
| Serial 4 | /dev/ttyS6 |

Now add a print statement in the Pixhawk Firmware to see received messages.  Build and upload this to Pixhawk.

```
[Firmware/src/modules/mavlink/mavlink_receiver.cpp]
/* if read failed, this loop won't execute */
for (ssize_t i = 0; i < nread; i++) {
	if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &status)) {

		/* --- REPORT HANDLING OF MESSAGE --- */
		printf("\n");
		printf("HANDLE MESSAGE\n");
		printf("MSGID:%i\n",msg.msgid);

		/* handle generic messages and commands */
		handle_message(&msg);
```

Open the system terminal as described here: https://dev.px4.io/en/debug/system_console.html 

On the off-board side, in another terminal run the `c_uart_interface_example` executable. You should see output in the NuttX shell similar to this:

```
HANDLE MESSAGE
MSGID:76

HANDLE MESSAGE
MSGID:84

(...)

HANDLE MESSAGE
MSGID:84

HANDLE MESSAGE
MSGID:76
```

Past this, you can:
- Modify the received message data type
- Modify the sent message data type

