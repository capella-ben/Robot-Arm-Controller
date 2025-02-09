import roboticstoolbox as rtb           # https://petercorke.github.io/robotics-toolbox-python/
from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath import SE3
from math import pi, atan2
import numpy as np
import warnings, sys
from tabulate import tabulate
from communications import COMMUNICATIONS


class mark4(DHRobot):
    """
    Class that models the mark IVa Robot Arm
    Defined joint configurations are:
        - qu, vertical 'READY' configuration
        - qh, Homed position
    .. note:: SI units of metres are used.
    """  # noqa
    def __init__(self):
        deg = pi/180
        # Note on offsets:  The logical model uses join angles either siode of 0, 
        #     but the physical model uses only positive joint angles.
        #     As such the offset is used to move the logical home position to vertical.
        #     Then a simple mapping can be used to convert logical joint angles
        #     to physical joint angles. 

        # visualisation:  https://vis-ro.web.app/robotics/modified-dh-model
        L = [
            RevoluteMDH(                     # Table
                d=0.11,             # distance from Deck to Shoulder axis
                a=0,
                alpha=0,
                offset=177*deg,           # Theta.  This is the value that changes during operation
                qlim=[-177*deg, 177*deg],
                flip=True           # in the visualiation tool.  If positive Theta is a move in the positive direction then  Flip = False
            ),

            RevoluteMDH(                     # Shoulder
                d=0.05,
                a=0,
                alpha=90*deg,
                offset=90*deg,
                qlim=[-90*deg, 88*deg],      # perhaps this can be change to 0-180 deg
                flip=False
            ),

            RevoluteMDH(                     # Elbow
                d=-0.05,
                a=0.22,
                alpha=0,
                offset=85*deg,              # non-normalised offset is 220
                qlim=[-135*deg, 135*deg],
                flip=True
            ),

            RevoluteMDH(                     # Forearm
                d=0.250,
                a=0,
                alpha=90*deg,
                offset=45*deg,
                qlim=[-45*deg, 45*deg],
                flip=False
            ),

            RevoluteMDH(                     # Wrist
                d=0,
                a=0,
                alpha=-90*deg,
                offset=8*deg,               # non-normalised joint angle is -80 
                qlim=[-72*deg, 72*deg],
                flip=False
            ),

            RevoluteMDH(                     # Radius
                d=0.100,
                a=0,
                alpha=90*deg,
                offset=135*deg,
                qlim=[-135*deg, 135*deg],
                flip=False
                
            )
        ]

        super().__init__(
            L,
            # basemesh="ABB/IRB140/link0.stl",
            name='MarkIVa',
            manufacturer='Ben',
            #meshdir="meshes/ABB/IRB140"
            )

        self.qh = np.array([-177*deg, -90*deg, -135*deg, -45*deg, -72*deg, -135*deg])
        self.qu = np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 0*deg])

        self.addconfiguration("qh", self.qh)     # Home Position
        self.addconfiguration("qu", self.qu)

    def convert_to_physical_angles(self, poses):
        deg = pi/180
        phyPoses = []
        for pose in poses:
            phyPose = []
            for i, a in enumerate(pose):
                limit = self.links[i].qlim[1]
                angle = a + limit
                phyPose.append(angle)
            phyPoses.append(phyPose)

        return phyPoses



def trajInfo(traj, full = False):
    print("Trajectory Information")
    print("   ", traj)
    print("   Number of Axis:", traj.naxes)
    if full: print("   Independent Variable:", traj.t)
    if full: print("   Is Time:", traj.istime)
    if full: print("   Position:", traj.s)
    if full: print("   Velocity:", traj.sd)
    if full: print("   Acceleration:", traj.sdd)
    print()


def printPoses(qt: np.ndarray):
    tbl = []
    for step in qt:
        row = []
        for joint in step:
            row.append("%.3f" % np.rad2deg(joint))
            #print("%.3f" % np.rad2deg(joint) + "\t", end="")
        tbl.append(row)
    
    print(tabulate(tbl, headers=['Table', 'Shoulder', 'Elbow', 'Forearm', 'Wrist', 'Radius']))

def sendPoses(qt: np.ndarray, send=False):
    for step in qt:
        command = 'move '
        for joint in step:
            command = command + f'{"%.3f" % np.rad2deg(joint)} '
        
        print(command)
        if send: sorterComms.send_command(command)


def genPose(robot: DHRobot, x: float, y: float, z: float, pitch = -90, roll = 0, minResidualAllowed = 0.0000009, printSol=False):
    """Generate the pose (joint angles) for a given point in space using inverse kinematics.  The end effector is always pointing away from the base.

    Args:
        x (float): x in meters
        y (float): y in meters
        z (float): z in meters
        pitch (int): The pitch of the end effector.  -90 is parallel to the ground, -90 -> -179 is pointing down, 0 is vertical pointing up
        roll  (int): This is the twist of the end effector. Positive angles (0 -> 180) rotate it clockwise when viewed from the centre (i.e. over the table), Negative angles (0 -> -180) rotate it anti-clockwise when viewed from the centre (i.e. over the table)
    Returns:
        IKsolution
    """
    T = SE3(x, y, z) * SE3.RPY(pitch, roll, (-1 * atan2(x, y)*(180/pi)), unit='deg', order='zyx')    # Pitch, Roll, Yaw
    #print(T)
    #  for SE3.RPY(z): Pitch
    #                  -90 is parallel to the ground
    #                  -90 -> -179 is pointing down
    #                  0 is vertical pointing exactly down.

    #  for SE3.RPY(x):  Yaw
    #                   (-1 * atan2(x, y)*(180/pi)) will point the end effector away from the centre

    #  for SE3.RPY(y):  Roll
    #                   This is the twist of the end effector
    #                   Positive angles (0 -> 180) rotate it clockwise when viewed from the centre (i.e. over the table)
    #                   Negative angles (0 -> -180) rotate it anti-clockwise when viewed from the centre (i.e. over the table)

    

    #sol = robot.ikine_GN(T, joint_limits=False)
    #sol = robot.ik_GN(T, joint_limits=True)                #  C++ version
    sol = robot.ik_LM(T, joint_limits=True, ilimit=3000)                #  C++ version
    print(sol)
    success = sol[1]
    iterations = sol[2]
    searches = sol[3]
    residual = sol[4]
    
    
    if residual > minResidualAllowed:
        sys.exit(f"A residual distance of {residual:.6f} meters is too great. The minimum allowed is {minResidualAllowed:.9f} meters")
        
    #elif not success:
    #    sys.exit("Failed to solve inverse kinematics")

    else:
        if printSol:
            print(f"Inverse Kinematics solution successfully solved for x={x}, y={y}, z={z}, pitch={pitch}, roll={roll}")
            print(f"  Table:       {np.rad2deg(sol[0][0]):.3f} degrees")
            print(f"  Shoulder:    {np.rad2deg(sol[0][1]):.3f} degrees")
            print(f"  Elbow:       {np.rad2deg(sol[0][2]):.3f} degrees")
            print(f"  Wrist 1:     {np.rad2deg(sol[0][3]):.3f} degrees")
            print(f"  Wrist 2:     {np.rad2deg(sol[0][4]):.3f} degrees")
            print(f"  Wrist 3:     {np.rad2deg(sol[0][5]):.3f} degrees")
            print(f"  Iterations:  {sol.iterations}")
            print(f"  Residual:    {sol.residual:.9f} meters")
            print()
        return sol[0]
    


    



def multiTrajectory(poses: list, steps: int):
    """Compute a trajectory (i.e an array of joint angles) from one pose to another with the specified number fo steps (e.g. 50 steps)

    Args:
        poses (list): A list of poses as waypoints for the trajectory
        steps (int): number of steps in each trajectory between waypoints
    """
    # create an empty ndarray 
    trajPoses = np.ndarray((0,6))
    
    for i in range(0, len(poses)-1):
        T2 = rtb.jtraj(poses[i], poses[i+1], steps)
        trajPoses = np.concatenate((trajPoses, T2.q))

    return trajPoses


"""
-----------------------------------------------------------------------------------------------------------------
"""


warnings.simplefilter("ignore", UserWarning)
print()
print()
print()
deg = pi/180

robot = mark4()
#print(robot.reach)

sorterComms = COMMUNICATIONS(debug=False)


poses = []
poses.append(robot.qh)
#poses.append(robot.qu)

poses.append(genPose(robot, -0.10, -0.10, 0.45, pitch=0, printSol=False))
#poses.append(genPose(robot, 0, 0, 0.45, pitch=0, printSol=False, minResidualAllowed=0.1))
poses.append(genPose(robot, 0, 0, 0.45, pitch=90, printSol=False))



trajPoses = multiTrajectory(poses, 30)
# print the joint angles of a trajectory as a series of poses (in degrees)
#printPoses(trajPoses)

print()
printPoses(robot.convert_to_physical_angles(poses))
print()
sendPoses(robot.convert_to_physical_angles(poses), send=False)

#print()
#sendPoses(robot.convert_to_physical_angles(trajPoses))

# plot joint coordinates V time
#rtb.xplot(trajPoses, block=False)

# Show the animation
#cm = robot.linkcolormap(['red', 'g', (0,0.5,0), '#0f8040', 'yellow', 'cyan'])

robot.plot(trajPoses, block=True, limits=[-0.6, 0.6, -0.6, 0.6, 0, 0.6], jointaxes=True, dt=0.1, vellipse=True)



