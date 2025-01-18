# sensor_communication

We are using jetson orin nano as the controller and node mcu with MPU650 sensor which will act as TCP server and communication with jetson using TCP port 2000.

We are getting yaw, pitch, roll values from MPU 6050 sensor which we are encoding and sending to jetson orin nano, which then decodes and publishes to respective topics.


