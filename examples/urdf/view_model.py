import pybullet as p  
import time
import pybullet_data

client_id = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('.')

base = p.loadURDF("raphael.urdf", useFixedBase=True)
for i in range(p.getNumJoints(base)):
    print('')
    print(p.getJointInfo(base, i))

right_arm_joints=[0,1,2,3,4,5,6]
delta = 0.001

def control_joint(joint, value, minn, maxx):
    global base
    # minn, maxx = get_joint_limits(base, joint)
    # if value < minn:
    #     value = minn
    # if value > maxx:
    #     value = maxx
    p.setJointMotorControl2(base, joint,
                controlMode=p.POSITION_CONTROL,targetPosition=value,
                force=30)

def joint_teleop(): 
    while True:
        joint_angles = [x[0] for x in p.getJointStates(base, right_arm_joints)]
        keys = p.getKeyboardEvents()
        if ord('a') in keys:
            joint = right_arm_joints[1]
            angle = p.getJointState(base, joint)[0]
            angle -= delta
            control_joint(joint, angle, 0.0, 0.3)

        if ord('s') in keys:
            joint = right_arm_joints[1]
            angle = p.getJointState(base, joint)[0]
            angle += delta
            control_joint(joint, angle, 0.0, 0.3)

        if ord('z') in keys:
            joint = right_arm_joints[6]
            angle = p.getJointState(base, joint)[0]
            angle -= delta
            control_joint(joint, angle, 0.0, 0.3)

        if ord('x') in keys:
            joint = right_arm_joints[6]
            angle = p.getJointState(base, joint)[0]
            angle += delta
            control_joint(joint, angle, 0.0, 0.3)

        p.stepSimulation()

joint_teleop()