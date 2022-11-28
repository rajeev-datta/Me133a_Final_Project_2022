'''hw5p2.py

   This is skeleton code for HW5 Problem 2a.  Please EDIT.

   This moves the tip in a straight line (tip splne), then returns in
   a joint spline.  Note we do not pre-compute a list of segment.
   Instead we create the necessary spline segment on the fly.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from hw5code.GeneratorNode  import GeneratorNode
from hw3code.Segments       import Hold, Stay, GotoCubic, SplineCubic
from hw5code.hw4p3sol       import fkin, Jac

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Define the known tip/joint positions.
        self.qA = np.radians(np.array([0, 60, -120]).reshape(3,1))
        self.xB = np.array([0.5, -0.5, 1.0]).reshape(3,1)

        # Pre-compute what we can.
        X0 = fkin(self.qA)
        self.T = 3
        
        # Define the current segment.  The first segment will be the
        # task-space tip movement from xA to xB.  Zero the start time.
        self.t0      = 0.0
        self.segment = GotoCubic(X0, self.xB, self.T, space='Tip')

        self.qp = self.qA
        self.xp = X0

        # Pick the convergence bandwidth.
        self.lam = 50


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, tabsolute, dt):
        # If the movement is completed, putting us at point B, end :)
        if self.segment.completed(tabsolute - self.t0):
            return None

        # If the movement is ongoing, compute the current values.
        (xd, xddot) = self.segment.evaluate(tabsolute - self.t0)
        
        xt = self.xp - fkin(self.qp)
        q = self.qp
	
	# Newton-Raphson Algorithm: Iterate seven times.
        for k in range(7):
            x = fkin(q)
            J = Jac(q)
            qnext = q + np.linalg.inv(J) @ (xd - x)
            q = qnext
	        
        J = Jac(q)
        
        qdot = np.linalg.inv(J) @ (xddot + self.lam*xt)
        
        self.xp = xd
        self.qp = q
            

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
