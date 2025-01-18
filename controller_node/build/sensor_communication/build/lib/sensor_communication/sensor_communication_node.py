import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.service import Service
from example_interfaces.srv import SetBool
import socket
import time
import select

class SensorCommunicationNode(Node):

    def __init__(self):
        super().__init__('sensor_communication_node')

        # Declare parameters
        self.declare_parameter('sensor_ip', '192.168.1.2')
        self.declare_parameter('sensor_port', 2000)
        self.declare_parameter('interval', 1000)  # Default interval
        self.sensor_ip = self.get_parameter('sensor_ip').get_parameter_value().string_value
        self.sensor_port = self.get_parameter('sensor_port').get_parameter_value().integer_value
        self.interval = self.get_parameter('interval').get_parameter_value().integer_value

        # Setup TCP connection to NodeMCU (Sensor)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.sensor_ip, self.sensor_port))
        self.client_socket.setblocking(False)  # Set the socket to non-blocking mode

        # ROS2 Publishers
        self.supply_voltage_pub = self.create_publisher(Int16, 'supply_voltage', 10)
        self.env_temp_pub = self.create_publisher(Int16, 'env_temp', 10)
        self.yaw_pub = self.create_publisher(Int16, 'yaw', 10)
        self.pitch_pub = self.create_publisher(Int16, 'pitch', 10)
        self.roll_pub = self.create_publisher(Int16, 'roll', 10)

        # Service to start and stop the sensor
        self.start_stop_service = self.create_service(SetBool, 'start_stop_sensor', self.start_stop_sensor_callback)

        # Start sensor flag
        self.sensor_active = False

        # Polling the socket for data continuously in the main loop
        self.create_timer(0.5, self.receive_status)  # Poll every 500ms to receive data

        # Timer to send start command at the given interval
        self.start_command_timer = None

    def start_stop_sensor_callback(self, request, response):
        """
        Callback for the service that starts or stops the sensor.
        """
        if request.data:
            # Start sensor
            self.start_sensor()
            response.success = True
            response.message = "Sensor started."
        else:
            # Stop sensor
            self.stop_sensor()
            response.success = True
            response.message = "Sensor stopped."
       
        return response

    def start_sensor(self):
        """
        Starts the sensor communication.
        """
        if not self.sensor_active:
            self.sensor_active = True
            self.send_start_command()  # Send the start command once immediately
            self.get_logger().info(f"Sensor started with interval {self.interval} ms")
            
            # Set a timer to keep sending the start command every interval
            self.start_command_timer = self.create_timer(self.interval / 1000.0, self.send_start_command)

    def stop_sensor(self):
        """
        Stops the sensor communication.
        """
        if self.sensor_active:
            self.sensor_active = False
            self.send_stop_command()  # Send the stop command to the sensor
            self.get_logger().info("Sensor stopped.")
            
            # Stop the start command timer if it's active
            if self.start_command_timer:
                self.start_command_timer.cancel()
                self.start_command_timer = None
            
            self.get_logger().info("Shutting down the node.")
            self.destroy_node()  # Kill the node to stop the process

    def send_start_command(self):
        """
        Sends the start command with the specified interval to the sensor.
        """
        command = f"#03{self.interval:04X}\r\n"  # Format: "#03<interval in hex><CR><LF>"
        self.client_socket.send(command.encode())
        self.get_logger().info(f"Sent Start command with interval {self.interval} ms")

    def send_stop_command(self):
        """
        Sends the stop command to the sensor.
        """
        command = "#09\r\n"  # Stop command
        self.client_socket.send(command.encode())
        self.get_logger().info("Sent Stop command")

    def receive_status(self):
        """
        This method continuously checks the socket for incoming data.
        """
        try:
            # Use select to check if there's data available to read
            readable, _, _ = select.select([self.client_socket], [], [], 0.5)  # 0.5 sec timeout
            if readable:
                data = self.client_socket.recv(1024)  # Receive data from the sensor
                if data:
                    self.decode_status(data)  # Process the data if received
        except Exception as e:
            self.get_logger().error(f"Error in receiving data: {e}")

    def decode_status(self, data):
        # Log the raw data received from the sensor for debugging
        self.get_logger().info(f"Raw data received: {data}")

        try:
            # Decode the response message, skip the start "$11" part and end <CR><LF>
            payload = data[3:-2]  # Remove "$11" and <CR><LF> (or <LF>)

            # Log the payload length for debugging
            payload_length = len(payload)
            self.get_logger().info(f"Decoded payload: {payload} (Length: {payload_length})")

            # Check if the payload length is as expected (20 bytes)
            if payload_length != 20:
                self.get_logger().error(f"Invalid payload length: {payload_length}. Expected 20 bytes.")
                return  # Early return if the payload length is incorrect

            # Extract and decode the values
            supply_voltage = int(payload[0:4], 16)  # First 4 hex chars -> SUPPLY_VOLTAGE (2 bytes)
            env_temp = int(payload[4:8], 16)  # Next 4 hex chars -> ENV_TEMP (2 bytes)

            # Decode yaw, pitch, and roll as signed 16-bit integers
            yaw = self.decode_signed_int(payload[8:12])  # Next 4 hex chars -> YAW (2 bytes)
            pitch = self.decode_signed_int(payload[12:16])  # Next 4 hex chars -> PITCH (2 bytes)
            roll = self.decode_signed_int(payload[16:20])  # Next 4 hex chars -> ROLL (2 bytes)

            # Publish data to ROS2 topics
            self.supply_voltage_pub.publish(Int16(data=supply_voltage))
            self.env_temp_pub.publish(Int16(data=env_temp))
            self.yaw_pub.publish(Int16(data=yaw))
            self.pitch_pub.publish(Int16(data=pitch))
            self.roll_pub.publish(Int16(data=roll))

            self.get_logger().info(f"Received status: Supply Voltage={supply_voltage}, Env Temp={env_temp}, Yaw={yaw}, Pitch={pitch}, Roll={roll}")

        except Exception as e:
            self.get_logger().error(f"Error in decoding status: {e}")

    def decode_signed_int(self, hex_value):
        # Decode a 4-character hex value into a signed 16-bit integer
        value = int(hex_value, 16)
        if value > 0x7FFF:  # If the value is greater than 32767, it's a negative number
            value -= 0x10000  # Convert to negative signed integer in the 16-bit range
        return value


def main(args=None):
    rclpy.init(args=args)
    node = SensorCommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

