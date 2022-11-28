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
        # Take action, based on whether the current segment's space.
        if self.segment.space() == 'Tip':
            # If the movement is completed, putting us at point B, end :)
            if self.segment.completed(tabsolute - self.t0):
            	self.t0 = tabsolute
            	self.segment = GotoCubic(self.qp, self.qA, self.T, space='Joint')
            	return self.evaluate(tabsolute,dt)
                
            # If the movement is ongoing, compute the current values.
            (xd, xddot) = self.segment.evaluate(tabsolute - self.t0)
        
            e = self.xp - self
            xrdot = xddot + self.lam * e
            J = Jac(self.q)        
            
            qdot = np.linalg.inv(J) @ xrdot            
            q = self.q + dt * qdot
	
        
            self.xp = xd
            self.qp = q
            
            # Return the position and velocity as python lists.
            return (q.flatten().tolist(), qdot.flatten().tolist())

        # For joint splines:
        else:
            # If the movement is completed, putting us at point A, add
            # a new straight line to xB.  Re-evaluate on new segment.
            if self.segment.completed(tabsolute - self.t0):
                return None
                #self.t0 = tabsolute
                #self.segment = GotoCubic(self.xp, self.xB, self.T, space='Tip')
                #return self.evaluate(tabsolute,dt)
	    
            # If the movement is ongoing, compute the current values.
            (q, qdot) = self.segment.evaluate(tabsolute - self.t0)

            # Store the same stuff as needed in the task-space moves.
            # This will make the transitions easier and consistent.
            self.qp = q
            self.xp = fkin(q) 
            

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
