import openravepy as orpy
# Load the robot
env = orpy.Environment()
env.Load('./osr_openrave/robots/denso_robotiq_85_gripper.robot.xml')
env.SetDefaultViewer()
robot = env.GetRobot('denso_robotiq_85_gripper')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())


# setting dof values
robot.SetActiveDOFValues([0.1, 0.7, 1.5, -0.5, -0.8, -1.2])
print(manipulator.GetEndEffectorTransform())

# robot.SetActiveDOFValues([0.8, -0.4, 1.5, -0.5, -0.8, -1.2])
# print(manipulator.GetEndEffectorTransform())

# print(robot.GetLinks())

link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
print(link_idx)

link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
print(robot.GetLink('robotiq_85_base_link').GetTransform())

# Improve the visualization settings
import numpy as np
np.set_printoptions(precision=6, suppress=True)
# Print the result
print(robot.ComputeJacobianTranslation(link_idx, link_origin))
print(robot.ComputeJacobianAxisAngle(link_idx))