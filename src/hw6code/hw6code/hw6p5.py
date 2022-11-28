'''hw6p5.py

   This is the skeleton code for HW6 Problem 5.

   This explores the singularity handing while following a circle
   outside the workspace.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from hw6code.GeneratorNode     import GeneratorNode
from hw6code.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        
        # Define the various points.
        self.q0 = np.radians(np.array([0, 28.955, 0, -57.910, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.7, 0.6]).reshape((-1,1))
        self.R0 = Reye()


        # Initialize the current joint position and chain data.
        self.q = self.q0
        self.chain.setjoints(self.q)

        self.err = np.array([0,0,0,0,0,0]).reshape(-1,1)
        self.lam = 80


    # Declare the joint names.
    def jointnames(self):
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        pd = np.array([0,
                       0.95-0.25*np.cos(t),
                       0.60+0.25*np.sin(t)]).reshape((-1,1))
        vd = np.array([0,
                       0.25*np.sin(t),
                       0.25*np.cos(t)]).reshape((-1,1))
        Rd = Reye()
        wd = pzero()
        
        q   = self.q
        err = self.err
        
        # Compute the inverse kinematics
        J = np.vstack((self.chain.Jv(), self.chain.Jw())) 
        
        JT = np.transpose(J)
        gam = 0.2
        Jsw = JT @ np.linalg.inv(J @ JT + gam**2 * np.eye(6))
                
        xdot = np.vstack((vd, wd))
        
        qgoal  = np.array([0, 0, 0, -np.pi/2, 0,0,0]).reshape((-1,1))
        
        lam = 10
        qdot2 = lam*(qgoal - q)
  
        
        qdot = Jsw @ (xdot + self.lam * err) + (np.eye(7) - Jsw @ J) @ qdot2
        
	# Integrate the joint position and update the kin chain data.
        q = q + dt * qdot
        self.chain.setjoints(q)

        # Save the joint value and precompute the task error for next cycle.
        self.q   = q
        self.err = np.vstack((ep(pd, self.chain.ptip()),eR(Rd, self.chain.Rtip())))
        
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
