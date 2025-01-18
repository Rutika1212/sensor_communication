import rclpy
from rclpy.node import Node
import socket
import time
from std_msgs.msg import Float32, Int16

class SensorCommunicationNode(Node):
    def __init__(self):
        super().__init__('sensor_communication_node')
        # Parameters
        self.declare_parameter('sensor_ip', '192.168.1.2')
        self.declare_parameter('sensor_port', 2000)
        self.declare_parameter('interval', 1000)  # Default interval
        self.sensor_ip = self.get_parameter('sensor_ip').get_parameter_value().string_value
        self.sensor_port = self.get_parameter('sensor_port').get_parameter_value().integer_value
        self.interval = self.get_parameter('interval').get_parameter_value().integer_value

        # Setup TCP connection to NodeMCU (Sensor)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.sensor_ip, self.sensor_port))
       
        # ROS2 Publishers
        self.supply_voltage_pub = self.create_publisher(Int16, 'supply_voltage', 10)
        self.env_temp_pub = self.create_publisher(Int16, 'env_temp', 10)
        self.yaw_pub = self.create_publisher(Int16, 'yaw', 10)
        self.pitch_pub = self.create_publisher(Int16, 'pitch', 10)
        self.roll_pub = self.create_publisher(Int16, 'roll', 10)

        # Start command to sensor
        self.send_start_command()
       
        # Timer to receive data
        self.timer = self.create_timer(1.0, self.receive_status)

    def send_start_command(self):
        # Sending start command with the set interval
        command = f"#03{self.interval:04X}\r\n"  # Format: "#03<interval in hex><CR><LF>"
        self.client_socket.send(command.encode())
        self.get_logger().info(f"Sent Start command with interval {self.interval} ms")

    def send_stop_command(self):
        # Send stop command to NodeMCU
        command = "#09\r\n"  # Stop command
        self.client_socket.send(command.encode())
        self.get_logger().info("Sent Stop command")

    def receive_status(self):
        # Receive and decode the status message from NodeMCU
        data = self.client_socket.recv(1024)
        if data:
            self.decode_status(data)

    def decode_status(self, data):
        # Example data format: "$11<SUPPLY_VOLTAGE><ENV_TEMP><YAW><PITCH><ROLL><CR><LF>"
        try:
            # Decode the response message, skip the start "$11" part
            payload = data[3:-2]  # Remove "$11" and <CR><LF>
            if len(payload) != 20:
                self.get_logger().error("Invalid payload length")
                return

            # Extract and decode the values
            supply_voltage = int(payload[0:4], 16)  # First 4 hex chars -> SUPPLY_VOLTAGE (2 bytes)
            env_temp = int(payload[4:8], 16)  # Next 4 hex chars -> ENV_TEMP (2 bytes)
            yaw = int(payload[8:12], 16)  # Next 4 hex chars -> YAW (2 bytes)
            pitch = int(payload[12:16], 16)  # Next 4 hex chars -> PITCH (2 bytes)
            roll = int(payload[16:20], 16)  # Next 4 hex chars -> ROLL (2 bytes)

            # Publish data to ROS2 topics
            self.supply_voltage_pub.publish(Int16(data=supply_voltage))
            self.env_temp_pub.publish(Int16(data=env_temp))
            self.yaw_pub.publish(Int16(data=yaw))
            self.pitch_pub.publish(Int16(data=pitch))
            self.roll_pub.publish(Int16(data=roll))

            self.get_logger().info(f"Received status: Supply Voltage={supply_voltage}, Env Temp={env_temp}, Yaw={yaw}, Pitch={pitch}, Roll={roll}")
        except Exception as e:
            self.get_logger().error(f"Error in decoding status: {e}")

    def on_shutdown(self):
        # Stop command on shutdown
        self.send_stop_command()
        self.client_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = SensorCommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

