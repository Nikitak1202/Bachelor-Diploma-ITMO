#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Twist
import random
import time


class ObstaclesController(Node):
    """
    ROS2 node that spawns cylindrical obstacles and a target in Gazebo,
    then makes them wander randomly using the planar_move plugin.
    """
    def __init__(self):
        super().__init__('obstacles_controller')

        # Declare configurable parameters
        self.declare_parameter('number_of_obstacles', 5)
        self.declare_parameter('target_speed', 1)
        self.declare_parameter('obstacle_speed', 1)
        self.declare_parameter('update_rate', 0.1)
        self.declare_parameter('spawn_area_min_x', -5.0)
        self.declare_parameter('spawn_area_max_x', 5.0)
        self.declare_parameter('spawn_area_min_y', -5.0)
        self.declare_parameter('spawn_area_max_y', 5.0)
        self.declare_parameter('cylinder_height', 0.2)       
        self.declare_parameter('cylinder_radius', 0.3)       

        self.num_obstacles = self.get_parameter('number_of_obstacles').value
        self.target_speed = self.get_parameter('target_speed').value
        self.obstacle_speed = self.get_parameter('obstacle_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.spawn_min_x = self.get_parameter('spawn_area_min_x').value
        self.spawn_max_x = self.get_parameter('spawn_area_max_x').value
        self.spawn_min_y = self.get_parameter('spawn_area_min_y').value
        self.spawn_max_y = self.get_parameter('spawn_area_max_y').value
        self.cylinder_height = self.get_parameter('cylinder_height').value
        self.cylinder_radius = self.get_parameter('cylinder_radius').value

        self.get_logger().info('ObstaclesController started')

        # Wait for Gazebo spawn service
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        self.model_names = []
        self.vel_publishers = {}

        self.spawn_cylinder('target', color='Blue')
        self.model_names.append('target')
        time.sleep(1.0)  # allow plugin to load

        # Spawn obstacles (red)
        for i in range(self.num_obstacles):
            name = f'obstacle_{i}'
            self.spawn_cylinder(name, color='Red')
            self.model_names.append(name)
            time.sleep(1.0)

        # Create publishers for cmd_vel topics (correct topic: /<model_name>/cmd_vel)
        for name in self.model_names:
            topic = f'/{name}/cmd_vel'
            self.vel_publishers[name] = self.create_publisher(Twist, topic, 10)

        # Start periodic velocity updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_velocities)

    def spawn_cylinder(self, model_name, color='Gray'):
        """
        Spawn a cylinder model with planar_move plugin at a random position.
        Uses wider, shorter dimensions to prevent tipping.
        Inertia computed for a solid cylinder (disk-like).
        """
        x = random.uniform(self.spawn_min_x, self.spawn_max_x)
        y = random.uniform(self.spawn_min_y, self.spawn_max_y)
        z = self.cylinder_height / 2.0  # base on ground

        mass = 1.0
        r = self.cylinder_radius
        h = self.cylinder_height
        # Inertia for solid cylinder about principal axes
        ixx = (1.0/12.0) * mass * (3*r*r + h*h)
        iyy = ixx
        izz = 0.5 * mass * r*r

        sdf = f'''
        <?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="{model_name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <static>false</static>
            <link name="base_link">
              <inertial>
                <mass>{mass}</mass>
                <inertia>
                  <ixx>{ixx:.6f}</ixx>
                  <ixy>0.0</ixy>
                  <ixz>0.0</ixz>
                  <iyy>{iyy:.6f}</iyy>
                  <iyz>0.0</iyz>
                  <izz>{izz:.6f}</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>{r}</radius>
                    <length>{h}</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>{r}</radius>
                    <length>{h}</length>
                  </cylinder>
                </geometry>
                <material>
                  <script>
                    <name>Gazebo/{color}</name>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                  </script>
                </material>
              </visual>
            </link>
            <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
              <robotNamespace>{model_name}</robotNamespace>
              <commandTopic>cmd_vel</commandTopic>
              <odometryTopic>odom</odometryTopic>
              <odometryFrame>odom</odometryFrame>
              <robotBaseFrame>base_link</robotBaseFrame>
              <updateRate>20.0</updateRate>
            </plugin>
          </model>
        </sdf>
        '''

        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = sdf
        request.robot_namespace = model_name
        request.reference_frame = 'world'

        future = self.client.call_async(request)
        self.get_logger().info(f'Spawning {model_name} at ({x:.2f}, {y:.2f})')
        # Do not wait for future – model appears asynchronously

    def update_velocities(self):
        """Publish random velocities to each model."""
        for name in self.model_names:
            max_speed = self.target_speed if name == 'target' else self.obstacle_speed
            twist = Twist()
            twist.linear.x = random.uniform(-max_speed, max_speed)
            twist.linear.y = random.uniform(-max_speed, max_speed)
            twist.angular.z = random.uniform(-1.0, 1.0)
            self.vel_publishers[name].publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()