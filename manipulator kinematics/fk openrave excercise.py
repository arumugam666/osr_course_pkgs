import openravepy as orpy
import numpy as np
from scipy import linalg
np.set_printoptions(precision=6, suppress=True)

# Load the robot
env = orpy.Environment()
env.Load('./osr_openrave/robots/denso_robotiq_85_gripper.robot.xml')
env.SetDefaultViewer()
robot = env.GetRobot('denso_robotiq_85_gripper')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())

# q1, delta_t, qdot1
q1 = np.array([-0.1,1.8,1.0,0.5,0.2,1.3])
qdot1 = np.array([1.2, -0.7, -1.5, -0.5, 0.8, -1.5])
delta_t = 0.1

# obtain transformation matrices of robotiq_85_base_link at t1,t2

robot.SetActiveDOFValues(q1)

print("\n")
print("Transformation matrix at t1")
print(robot.GetLink('robotiq_85_base_link').GetTransform())
initial = robot.GetLink('robotiq_85_base_link').GetTransform()

robot.SetActiveDOFValues(q1+delta_t*qdot1)
print("\n")
print("Transformation matrix at t1 + delta t")
print(robot.GetLink('robotiq_85_base_link').GetTransform())
target = robot.GetLink('robotiq_85_base_link').GetTransform()

# linear and angular jacobians at t1
robot.SetActiveDOFValues(q1)
R1 = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,:3]

link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]

linear_jacobian = robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:-1]
angular_jacobian = robot.ComputeJacobianAxisAngle(link_idx)[:,:-1]

print("\n")
print("linear jacobian matrix")
print(linear_jacobian)
print("\n")
print("angular jacobian matrix")
print(angular_jacobian)

# calculate linear and angular velocities from jacobians
linear_velocity = np.dot(linear_jacobian,qdot1)
angular_velocity = np.dot(angular_jacobian,qdot1)

print("\n")
print("linear veolcity")
print(linear_velocity)

print("\n")
print("angular velocity")
print(angular_velocity)

# aproximate new translation matrix

x2 = link_origin + delta_t*linear_velocity
w_carat = np.array(
[
    [0, -angular_velocity[2], angular_velocity[1]],
    [angular_velocity[2], 0, -angular_velocity[0]],
    [-angular_velocity[1], angular_velocity[0], 0]
]
)

R2 = np.dot(linalg.expm(w_carat*delta_t),R1)

output_transform = np.eye(4)
output_transform[:3,:3] = R2
output_transform[:3,3] = x2
print("\n")
print("Difference between the output transform and target transform")
print(output_transform-target)
