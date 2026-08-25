import rclpy
from rclpy.node import Node
import roboticstoolbox as rtb
import numpy as np
from interfaces.srv import TrajectoryRequest
from spatialmath import SE3

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

        self.declare_parameter('dt', 0.001)

        self.get_logger().info('Robot model:\n')
        self.get_logger().info(str(self.kuka_robot))

        self.generate_trajectory_srv = self.create_service(TrajectoryRequest, 'generate_trajectory', self.generate_trajectory)

    def generate_trajectory(self, request, response):
        
        self.get_logger().info('Received generate_trajectory request')

        initial_cartesian_coordinates = request.initial_cartesian_coord
        final_cartesian_coordinates = request.final_cartesian_coord
        time = request.time
        dt = self.get_parameter('dt').get_parameter_value().double_value

        t = np.linspace(0, time, np.round(time/dt).astype(int) )

        try:
            Ti = SE3.Trans(initial_cartesian_coordinates[:3])*SE3.RPY(initial_cartesian_coordinates[-3:])
            Tf = SE3.Trans(final_cartesian_coordinates[:3])*SE3.RPY(final_cartesian_coordinates[-3:])
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
        
        response.num_joints = 7
        response.num_timesteps = len(t)
        response.success = True
        response.message = 'Trajectory successfuly generated'

        return response


def main():
    rclpy.init()
    trajectory_generator = TrajectoryGenerator()
    rclpy.spin(trajectory_generator)
    trajectory_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()