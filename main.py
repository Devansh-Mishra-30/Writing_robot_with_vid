"""
Runs simulation of Panda 7-DOF robot arm following trajectory
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

from PandaMechanics import PandaMechanics
from PandaPlot import PandaPlot
from trajectory import circle_trajectory, point_trajectory, svg_trajectory
from controller import PD, PD_gravity,  PD_velocity, CLF_QP_with_error

# UPDATE THIS TO CHANGE SVGs
SVG_FILE = "svg/portfolio_writing1 (8).svg"
MAX_t = 70

# Set up simulator
TIME_STEP = 1/1000
physics_client = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "writing_simulation_video8.mp4")
p.setGravity(0, 0, -9.81)
p.setTimeStep(TIME_STEP)
p.resetDebugVisualizerCamera(cameraDistance=1,
                             cameraYaw=-20,
                             cameraPitch=-40,
                             cameraTargetPosition=[0, 0, 0.6])


# Load URDFs
plane_ID = p.loadURDF("plane.urdf")
start_pos = [0,0,0]
start_orientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("urdfs/panda_arm_no_hand.urdf", start_pos, start_orientation, useFixedBase=True)
table = p.loadURDF("urdfs/writing_surface.urdf", [0.45,0,0], start_orientation, useFixedBase=True)


NUM_JOINTS = p.getNumJoints(robot) - 1
JOINTS = [i for i in range(NUM_JOINTS)]
simulation_time = 0
i = 0
panda_mech = PandaMechanics()
plot = PandaPlot(NUM_JOINTS)
p.changeDynamics(bodyUniqueId=table, linkIndex=-1, lateralFriction=0.1) # Reduce table friction

# Start robot in "default" position to not exceed joint limits
q = np.array([0, -0.785, 0, -2.355, 0, 1.57, 0.785])
for i in range(NUM_JOINTS):
    p.resetJointState(bodyUniqueId = robot, jointIndex = i, targetValue = q[i])


while(True):
    
    Y = panda_mech.solve_fk(q)
    Y_des = svg_trajectory(simulation_time, SVG_FILE)
    q_des = panda_mech.solve_ik(q=q, x=Y_des[0], y=Y_des[1], z=Y_des[2])
    q = np.array([p.getJointState(robot, i)[0] for i in JOINTS])
    dq = np.array([p.getJointState(robot, i)[1] for i in JOINTS])
    J = panda_mech.get_Jacobian(q)


    # Only plot ever so often to reduce simulation slow down
    if i % 200 == 0:
        plot.update_plot(simulation_time, q, q_des, Y, Y_des)


    v = PD_velocity(q, dq, q_des)
    # UNCOMMENT EITHER OF THE FOLLOWING TO TEST DIFFERENT CONTROLLERS
    # u = PD(q, dq, q_des)
    # u = CLF_QP_with_error(q, dq, q_des, dq_des=np.zeros_like(q), ddq_des=np.zeros_like(q), Fz=1, J=J)

    p.setJointMotorControlArray( bodyIndex=robot, jointIndices=JOINTS, 
        controlMode=p.VELOCITY_CONTROL, targetVelocities = v)
        # USE THE FOLLOWING WHEN TESTING EITHER OF THE u (TORQUE) CONTROLLERS
        # controlMode=p.TORQUE_CONTROL, forces = u)

    
    p.stepSimulation() # Default is 1/240hz
    simulation_time += TIME_STEP
    i +=1
    time.sleep(TIME_STEP)  # Match simulation with real-life time
    # Check if robot has stopped moving
    stagnant_counter = 0
    warmup_time = 10.0

    if simulation_time > warmup_time:
        motion_threshold = 1e-3  # Tweak as needed
        if np.linalg.norm(dq) < motion_threshold:
            stagnant_counter += 1
        else:
            stagnant_counter = 0

        if stagnant_counter > 1000:  # ~1 second at 1000Hz
            print("[INFO] Robot stopped moving. Exiting early.")
            break


p.stopStateLogging(log_id)
p.disconnect()
plot.save_videos()