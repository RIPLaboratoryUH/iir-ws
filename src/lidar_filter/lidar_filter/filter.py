import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FilterScanNode(Node):
    def __init__(self):
        super().__init__('filter_scan_node')

        # Declare parameters for range filtering


        # Declare parameters for angular sector filtering (in degrees)
        self.declare_parameter('filter_min_angle_deg', 75.0)
        self.declare_parameter('filter_max_angle_deg', 120.0)

        # Get the declared parameter values.

        self.filter_min_angle_rad = math.radians(self.get_parameter('filter_min_angle_deg').get_parameter_value().double_value)
        self.filter_max_angle_rad = math.radians(self.get_parameter('filter_max_angle_deg').get_parameter_value().double_value)

        self.get_logger().info(
            f'Filter initialized with angular sector [{self.filter_min_angle_rad:.2f} rad, {self.filter_max_angle_rad:.2f} rad]'
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10
        )

    def scan_callback(self, msg):
        """
        Callback function for the LaserScan subscriber.
        Filters the scan data by range and a specific angular sector.
        """
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        filtered_ranges = []
        filtered_intensities = []

        # Iterate through each range measurement.
        for i in range(len(msg.ranges)):
            current_range = msg.ranges[i]
            current_intensity = msg.intensities[i] if msg.intensities else 0.0

            # Calculate the angle for the current data point.
            current_angle = (msg.angle_min + i) * msg.angle_increment

            # Check if the current angle is within the filter sector.
            if self.filter_min_angle_rad <= current_angle <= self.filter_max_angle_rad:
                # If the point is in the filtered sector, replace it with NaN.
                filtered_ranges.append(float('nan'))
                filtered_intensities.append(float('nan'))
            else:
                # Otherwise, keep the original value.
                filtered_ranges.append(current_range)
                filtered_intensities.append(current_intensity)

        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = filtered_intensities
        self.scan_publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    filter_scan_node = FilterScanNode()
    rclpy.spin(filter_scan_node)
    filter_scan_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()