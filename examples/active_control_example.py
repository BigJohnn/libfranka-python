#!/usr/bin/env python3

import numpy as np
import time
from franka_bindings import (
    Robot, 
    ControllerMode, 
    Torques, 
    JointPositions, 
)

def main():
    # Connect to robot
    robot = Robot("127.0.0.1    ")  # Replace with your robot's IP
    
    try:
        # Set collision behavior
        lower_torque_thresholds = [20.0] * 7  # Nm
        upper_torque_thresholds = [40.0] * 7  # Nm
        lower_force_thresholds = [10.0] * 6   # N (linear) and Nm (angular)
        upper_force_thresholds = [20.0] * 6   # N (linear) and Nm (angular)
        
        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds
        )
        
        # Set joint impedance
        joint_stiffness = [400.0] * 7  # Nm/rad
        robot.set_joint_impedance(joint_stiffness)
        
        # Set cartesian impedance
        cartesian_stiffness = [3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0]  # N/m and Nm/rad
        robot.set_cartesian_impedance(cartesian_stiffness)
        
        # Example of torque control
        print("Starting torque control...")
        control = robot.start_torque_control()
        
        # Control loop
        for i in range(1000):  # Run for 1000 iterations
            state = control.readOnce()
            
            # Example: Apply gravity compensation
            gravity_compensation = [0.0] * 7  # Replace with actual gravity compensation
            torques = Torques(gravity_compensation)
            
            if i == 999:  # Last iteration
                torques.motion_finished = True
                
            control.writeOnce(torques)
            time.sleep(0.001)  # 1kHz control loop
            
        # Example of joint position control
        print("Starting joint position control...")
        control = robot.start_joint_position_control(ControllerMode.JointImpedance)
        
        # Move to home position
        home_position = [0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4]
        joint_positions = JointPositions(home_position)
        
        state = control.readOnce()
        joint_positions.motion_finished = True
        control.writeOnce(joint_positions)
        
    except Exception as e:
        print(f"Error occurred: {e}")
        robot.stop()
    
if __name__ == "__main__":
    main() 