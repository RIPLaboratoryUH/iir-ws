import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import socket
import threading
import math

# Customize this
topic_name = 'm16_picolistener'
imu_topic_name = 'm16_picolistener_imu'

# Set your UDP listening address and port
LISTEN_IP = ''  # Listen on all interfaces
UDP_PORT = 8816        # Adjust as needed

class UDPSensorNode(Node):
    def __init__(self):
        super().__init__('m16_picolistener_node')

        # Create publisher
       
        self.publisher_ = self.create_publisher(Imu, imu_topic_name, 10)
        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, UDP_PORT))
        self.get_logger().info(f"Listening for UDP packets on port {UDP_PORT}")

        # Start listening in a separate thread
        udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        udp_thread.start()

    def udp_listener(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode().strip()
                self.get_logger().info(f"Received message")

                parts = message.split(',')
                msg = Imu()
                # msg.header.stamp = parts[0] #first item is timestamp
                msg.header.frame_id = imu_topic_name
                # Parse roll, pitch, yaw
                #parts [1] is the shunt value
                roll = float(parts[2])
                pitch = float(parts[3])
                yaw = float(parts[4])
                quat = quaternion_from_euler(roll, pitch, yaw)
                msg.orientation.x = quat[0]
                msg.orientation.y = quat[1]
                msg.orientation.z = quat[2]
                msg.orientation.w = quat[3]
                
                self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
