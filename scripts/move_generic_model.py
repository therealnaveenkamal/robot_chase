#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations

from std_msgs.msg import String

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
# from gazebo_msgs.msg import ModelState

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Coordinates:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        
class CamBotMove(Node):

    def __init__(self, timer_period=0.05, model_name="morty", init_dest_frame="carrot", trans_speed=1.0, rot_speed=0.1):
        super().__init__('force_move_cam_bot')
        
        self._model_name = model_name

        self.set_destination_frame(new_dest_frame=init_dest_frame)
        self.set_translation_speed(speed=trans_speed)
        self.set_rotation_speed(speed=rot_speed)

        # For the TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer_period = timer_period
        self.timer_rate = 1.0 / self.timer_period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.subscriber= self.create_subscription(
            String,
            '/destination_frame',
            self.move_callback,
            QoSProfile(depth=1))

        self.set_entity_client = self.create_client(SetEntityState, "/morty/set_entity_state")
        while not self.set_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service READY...')
        
        # create a request
        self.req = SetEntityState.Request()

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.move_step_speed()
        self.get_logger().info("Moved the Robot to frame ="+str(self.objective_frame))

    def set_translation_speed(self, speed):
        self.trans_speed = speed
    
    def set_rotation_speed(self, speed):
        self.rot_speed = speed

    def set_destination_frame(self, new_dest_frame):
        self.objective_frame = new_dest_frame

    def move_callback(self, msg):
        self.set_destination_frame(new_dest_frame=msg.data)
        
    def move_step_speed(self):
        coordinates_to_move_to = self.calculate_coord()
        if coordinates_to_move_to is not None:
            self.move_model(coordinates_to_move_to)
        else:
            self.get_logger().warning("No Coordinates available yet...")

    def get_model_pose_from_tf(self, origin_frame="rick/base_link", dest_frame="morty/base_link"):
        """
        Extract the pose from the TF
        """
        # Look up for the transformation between dest_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach dest_frame
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                origin_frame,
                dest_frame,
                now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {origin_frame} to {dest_frame}: {ex}')
            return None


        translation_pose = trans.transform.translation
        rotation_pose = trans.transform.rotation

        self.get_logger().info("type translation_pose="+str(type(translation_pose)))
        self.get_logger().info("type rotation_pose="+str(type(rotation_pose)))


        pose = Pose()
        pose.position.x = translation_pose.x
        pose.position.y = translation_pose.y
        pose.position.z = translation_pose.z
        pose.orientation.x = rotation_pose.x
        pose.orientation.y = rotation_pose.y
        pose.orientation.z = rotation_pose.z
        pose.orientation.w = rotation_pose.w

        return pose



    def calculate_coord(self):
        """
        Gets the current position of the model and adds the increment based on the Publish rate
        """
        pose_dest = self.get_model_pose_from_tf(origin_frame="world", dest_frame=self.objective_frame)
        self.get_logger().error("POSE DEST="+str(pose_dest))
        if pose_dest is not None:

            explicit_quat = [pose_dest.orientation.x, pose_dest.orientation.y,
                             pose_dest.orientation.z, pose_dest.orientation.w]
            pose_now_euler = tf_transformations.euler_from_quaternion(explicit_quat)

            roll = pose_now_euler[0]
            pitch = pose_now_euler[1]
            yaw = pose_now_euler[2]

            coordinates_to_move_to = Coordinates(x=pose_dest.position.x,
                                                y=pose_dest.position.y,
                                                z=pose_dest.position.z,
                                                roll=roll,
                                                pitch=pitch,
                                                yaw=yaw)
        else:
            coordinates_to_move_to = None

        return coordinates_to_move_to

    def move_model(self, coordinates_to_move_to):

    
        pose = Pose()

        pose.position.x = coordinates_to_move_to.x
        pose.position.y = coordinates_to_move_to.y
        pose.position.z = coordinates_to_move_to.z


        quaternion = tf_transformations.quaternion_from_euler(coordinates_to_move_to.roll,
                                                              coordinates_to_move_to.pitch,
                                                              coordinates_to_move_to.yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # You set twist to Null to remove any prior movements
        twist = Twist()
        linear = Vector3()
        angular = Vector3()

        linear.x = 0.0
        linear.y = 0.0
        linear.z = 0.0

        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0

        twist.linear = linear
        twist.angular = angular

        state = EntityState()
        state.name = self._model_name
        state.pose = pose
        state.twist = twist
        state.reference_frame = "world"

        self.req.state = state

        self.get_logger().error("STATE to SEND="+str(self.req.state))

        # send the request
        try:     
            self.future = self.set_entity_client.call_async(self.req)
        except Exception as e:
            self.get_logger().error('Error on calling service: %s', str(e))
        
            

def main(args=None):
    rclpy.init()

    move_obj = CamBotMove()

    print("Start Moving")
    rclpy.spin(move_obj)


if __name__ == '__main__':
    main()