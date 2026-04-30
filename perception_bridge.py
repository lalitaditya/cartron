#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, TransformStamped

class PerceptionBridge(Node):
    def __init__(self):
        super().__init__('perception_bridge')

        # TF listener to find the tag in robot base frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to update the MoveIt planning scene
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # Internal state to avoid overwhelming the scene with updates
        self.last_pose = None
        self.threshold = 0.01  # 1cm movement threshold to trigger update

        # Timer to check for tag updates at 2Hz
        self.timer = self.create_timer(0.5, self.update_scene)
        self.get_logger().info("Perception Bridge Started. Waiting for 'shopping_cart_tag'...")

    def update_scene(self):
        try:
            # Look up the transform from robot base to the shopping cart tag
            # We use base_link as the root for MoveIt planning
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'shopping_cart_tag',
                now,
                timeout=Duration(seconds=0.1)
            )
            
            # If the tag has moved significantly, update the planning scene
            if self._has_moved(t):
                self._publish_cart_to_moveit(t)
                self.last_pose = t
                
        except TransformException as ex:
            # This is expected if the tag is not currently visible
            pass

    def _has_moved(self, current_t):
        if self.last_pose is None:
            return True
        
        dx = current_t.transform.translation.x - self.last_pose.transform.translation.x
        dy = current_t.transform.translation.y - self.last_pose.transform.translation.y
        dz = current_t.transform.translation.z - self.last_pose.transform.translation.z
        dist = (dx**2 + dy**2 + dz**2)**0.5
        return dist > self.threshold

    def _publish_cart_to_moveit(self, t):
        # Create the collision object for the shopping cart
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'shopping_cart'
        
        # Define the cart's bounding box (approximate)
        # sy=0.4, sx=0.6, sz=0.8 for a standard cart
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.1, 0.4, 0.4] # Let's start with the handle area
        
        # Set the pose based on the AprilTag
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        # Wrap in a PlanningScene message to force an immediate update
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)
        
        self.get_logger().info(f"Updated shopping_cart pose in MoveIt (dist: {t.transform.translation.x:.2f})")

def main():
    rclpy.init()
    node = PerceptionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
