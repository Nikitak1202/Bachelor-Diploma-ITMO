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

        # Dynamic behavior parameters.
        self.declare_parameter('number_of_obstacles', 3)
        self.declare_parameter('target_speed', 0.5)
        self.declare_parameter('obstacle_speed', 0.5)
        self.declare_parameter('update_rate', 0.1)
        self.declare_parameter('cylinder_height', 0.2)
        self.declare_parameter('cylinder_radius', 0.3)

        # Deterministic spawn parameters.
        self.declare_parameter('target_spawn_x', 7.0)
        self.declare_parameter('target_spawn_y', -0.5)
        self.declare_parameter(
            'obstacle_spawn_positions',
            [0.8, 0.8, 0.8, 3.2, -0.8, 2.0]
        )

        self.num_obstacles = self.get_parameter('number_of_obstacles').value
        self.target_speed = self.get_parameter('target_speed').value
        self.obstacle_speed = self.get_parameter('obstacle_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.cylinder_height = self.get_parameter('cylinder_height').value
        self.cylinder_radius = self.get_parameter('cylinder_radius').value
        self.target_spawn_x = self.get_parameter('target_spawn_x').value
        self.target_spawn_y = self.get_parameter('target_spawn_y').value
        raw_positions = self.get_parameter('obstacle_spawn_positions').value
        self.obstacle_spawn_positions = self._parse_obstacle_positions(raw_positions)

        self.get_logger().info('ObstaclesController started')

        # Wait for Gazebo spawn service
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        self.model_names = []
        self.vel_publishers = {}

        self.spawn_cylinder(
            'target',
            x=self.target_spawn_x,
            y=self.target_spawn_y,
            color='Blue'
        )
        self.model_names.append('target')
        time.sleep(1.0)  # allow plugin to load

        # Spawn obstacles (red) using deterministic positions.
        for i in range(self.num_obstacles):
            name = f'obstacle_{i}'
            x, y = self._obstacle_pose(i)
            self.spawn_cylinder(name, x=x, y=y, color='Red')
            self.model_names.append(name)
            time.sleep(1.0)

        # Create publishers for cmd_vel topics
        for name in self.model_names:
            topic = f'/{name}/cmd_vel'
            self.vel_publishers[name] = self.create_publisher(Twist, topic, 10)

        # Start periodic velocity updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_velocities)

    def _parse_obstacle_positions(self, raw_positions):
        points = []
        if len(raw_positions) % 2 != 0:
            self.get_logger().warn(
                'obstacle_spawn_positions must contain an even number of values; '
                'the last value will be ignored.'
            )
        usable_len = len(raw_positions) - (len(raw_positions) % 2)
        for i in range(0, usable_len, 2):
            points.append((float(raw_positions[i]), float(raw_positions[i + 1])))
        if not points:
            points = [(0.8, 0.8), (0.8, 3.2), (-0.8, 2.0)]
        return points

    def _obstacle_pose(self, index):
        if index < len(self.obstacle_spawn_positions):
            return self.obstacle_spawn_positions[index]

        # Deterministic fallback if more obstacles than configured points.
        row = index // len(self.obstacle_spawn_positions)
        base_x, base_y = self.obstacle_spawn_positions[index % len(self.obstacle_spawn_positions)]
        return base_x + 0.7 * row, base_y

    def spawn_cylinder(self, model_name, x, y, color='Gray'):
        """
        Spawn a cylinder model with planar_move plugin at a fixed position.
        """
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