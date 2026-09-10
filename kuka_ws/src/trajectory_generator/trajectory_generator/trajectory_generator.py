import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from interfaces.srv import TrajectoryRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped

import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3, UnitQuaternion


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        self.get_logger().info('trajectory_generator node started')

        #DH Parameters for the Kuka iiwa14 manipulator
        #https://www.researchgate.net/figure/KUKA-LBR-iiwa-14-R820-DH-table_tbl1_377325054
        self.kuka_robot = rtb.DHRobot([
            rtb.RevoluteDH(alpha=np.pi/2, d=0.36, a=0.0),
            rtb.RevoluteDH(alpha=-np.pi/2, d=0.0, a=0.0),
            rtb.RevoluteDH(alpha=-np.pi/2, d=0.42, a=0.0),
            rtb.RevoluteDH(alpha=np.pi/2, d=0.0, a=0.0),
            rtb.RevoluteDH(alpha=np.pi/2, d=0.4, a=0.0),
            rtb.RevoluteDH(alpha=-np.pi/2, d=0.0, a=0.0),
            rtb.RevoluteDH(alpha=0.0, d=0.126, a=0.0),
        ])

        self.declare_parameter('dt', 0.01)

        self.get_logger().info('Robot model:\n')
        self.get_logger().info(str(self.kuka_robot))

        self.generate_trajectory_srv = self.create_service(TrajectoryRequest, 'generate_trajectory', self.generate_trajectory)

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_state',
            self.broadcast_endeffector_tf,
            10)

        self.cartesian_state_publisher = self.create_publisher(Float64MultiArray, 'cartesian_state', 10)

        self.endeffector_tf_broadcaster = TransformBroadcaster(self)

    def tf_to_E3(tf):

        pos_vector = [tf.translation.x,
                      tf.translation.y,
                      tf.translation.z]
        quaternion_elements = [tf.orientation.w,
                               tf.orientation.x,
                               tf.orientation.y,
                               tf.orientation.z]
        orientation_quaternion = UnitQuaternion(quaternion_elements)
        return SE3.Trans(pos_vector)*SE3(orientation_quaternion)
        

    def generate_trajectory(self, request, response):
        
        self.get_logger().info('Received generate_trajectory request')

        initial_tf = request.initial_tf
        final_tf = request.final_tf
        time = request.time
        dt = self.get_parameter('dt').get_parameter_value().double_value

        t = np.linspace(0, time, np.round(time/dt).astype(int) )

        try:
            Ti = self.tf_to_E3(initial_tf)
            Tf = self.tf_to_E3(final_tf)
        except Exception as e:
            self.get_logger().error(f'Exception when defining pose: {str(e)}')
            response.success = False
            response.message = f'Exception when defining pose: {str(e)}'
            return response

        try:
            q0 = self.kuka_robot.ikine_LM(Ti).q
            qf = self.kuka_robot.ikine_LM(Tf).q
        except Exception as e:
            self.get_logger().error(f'Exception when computing inverse kinematics: {str(e)}')
            response.success = False
            response.message = f'Exception when computing inverse kinematics: {str(e)}'
            return response
        
        joint_trajectory = rtb.tools.trajectory.jtraj(q0, qf, t)

        response.joint_pos = joint_trajectory.q.flatten()
        response.joint_vel = joint_trajectory.qd.flatten()
        response.joint_acc = joint_trajectory.qdd.flatten()
        
        response.dt = dt
        response.success = True
        response.message = 'Trajectory successfuly generated'

        return response

    def broadcast_endeffector_tf(self, msg):

        joint_coords = msg.position

        forward_kinematics = self.kuka_robot.fkine(np.array(joint_coords))
        position = forward_kinematics.t

        endeffector_tf = TransformStamped()
        endeffector_tf.header.stamp = self.get_clock().now().to_msg()
        endeffector_tf.header.frame_id = "world"
        endeffector_tf.child_frame_id = "endeffector"

        endeffector_tf.transform.translation.x = position[0]
        endeffector_tf.transform.translation.y = position[1]
        endeffector_tf.transform.translation.z = position[2]

        quaternion_orientation = UnitQuaternion(forward_kinematics)
        endeffector_tf.transform.rotation.x = quaternion_orientation.v[0]
        endeffector_tf.transform.rotation.y = quaternion_orientation.v[1]
        endeffector_tf.transform.rotation.z = quaternion_orientation.v[2]
        endeffector_tf.transform.rotation.w = quaternion_orientation.s

        self.endeffector_tf_broadcaster.sendTransform(endeffector_tf)


def main():
    rclpy.init()
    trajectory_generator = TrajectoryGenerator()
    rclpy.spin(trajectory_generator)
    trajectory_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()