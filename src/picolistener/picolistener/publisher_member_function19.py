import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import socket
import threading
import math

# Customize this
topic_name = 'm19_picolistener'
imu_topic_name = 'm19_picolistener_imu'

# Set your UDP listening address and port
LISTEN_IP = ''  # Listen on all interfaces
UDP_PORT = 8819 #make sure this is the right port mark      # Adjust as needed

class UDPSensorNode(Node):
    def __init__(self):
        super().__init__('m19_picolistener_node')

        # Create publisher
       
        #self.imupublisher_ = self.create_publisher(Imu, imu_topic_name, 10)
        self.shuntpublisher_ = self.create_publisher(String, topic_name, 10)
        #self.get_logger().info(f"Publishing IMU data on topic: {imu_topic_name}")
        self.get_logger().info(f"Publishing shunt data on topic: {topic_name}")
        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, UDP_PORT))
        self.get_logger().info(f"Listening for UDP packets on port {UDP_PORT}")

        # Start listening in a separate thread
        udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        udp_thread.start()

    def udp_listener(self):
        first = True
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode().strip()
                if first:
                    first = False
                    self.get_logger().info(f"Received a message from port {UDP_PORT}. Hiding this statement from now on")

                parts = message.split(',')
              #  msg = Imu()
                # msg.header.stamp = parts[0] #first item is timestamp
              #  msg.header.frame_id = "imu_frame"
                # Parse roll, pitch, yaw
                #parts [1] is the shunt value
              #  roll = float(parts[2])
              #  pitch = float(parts[3])
              #  yaw = float(parts[4])
              #  quat = quaternion_from_euler(roll, pitch, yaw)
              #  msg.orientation.x = quat[0]
              #  msg.orientation.y = quat[1]
             #   msg.orientation.z = quat[2]
             #   msg.orientation.w = quat[3]
             #   msg.orientation_covariance = [0.0] * 9 # do i need to set these to real values, considering that the imu does filtering
             #   self.imupublisher_.publish(msg)

                msg2 = String()
                msg2.data = f"{parts[0]}, {parts[1]}" 
                self.shuntpublisher_.publish(msg2)
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
