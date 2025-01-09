import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath import SE3
from math import pi, atan2
import numpy as np
import warnings, sys
from tabulate import tabulate

class mark4(DHRobot):
    """
    Class that models the mark IVa Robot Arm
    Defined joint configurations are:
        - qz, zero joint angle configuration
        - qr, vertical 'READY' configuration
        - qh, Homed position
    .. note:: SI units of metres are used.
    """  # noqa
    def __init__(self):
        deg = pi/180

        # visualisation:  https://vis-ro.web.app/robotics/modified-dh-model
        L = [
            RevoluteMDH(                     # Table
                d=0.11,             # distance from Deck to Shoulder axis
                a=0,
                alpha=0,
                offset=0,           # Theta.  This is the value that changes during operation
                #qlim=[0*deg, 354*deg],
                qlim=[-177*deg, 177*deg],
                flip=True           # in the visualiation tool.  If positive Theta is a move in the positive direction then  Flip = False
            ),

            RevoluteMDH(                     # Shoulder
                d=0.05,
                a=0,
                alpha=90*deg,
                offset=0,
                #qlim=[0*deg, 172*deg],      # perhaps this can be change to 0-180 deg
                qlim=[-86*deg, 86*deg],      # perhaps this can be change to 0-180 deg
                flip=False
            ),

            RevoluteMDH(                     # Elbow
                d=-0.05,
                a=0.22,
                alpha=0,
                offset=220*deg,
                #qlim=[0*deg, 270*deg],
                qlim=[-135*deg, 135*deg],
                flip=True
            ),

            RevoluteMDH(                     # Forearm
                d=0.250,
                a=0,
                alpha=90*deg,
                offset=0,
                #qlim=[0*deg, 90*deg],
                qlim=[-45*deg, 45*deg],
                flip=True
            ),

            RevoluteMDH(                     # Wrist
                d=0,
                a=0,
                alpha=-90*deg,
                offset=-80*deg,
                #qlim=[0*deg, 144*deg],
                qlim=[-72*deg, 72*deg],
                flip=False
            ),

            RevoluteMDH(                     # Radius
                d=0.100,
                a=0,
                alpha=90*deg,
                offset=0,
                #qlim=[0*deg, 270*deg],
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

        self.qh = np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 0*deg])
        self.qu = np.array([177*deg, 90.3*deg, 137*deg, 45*deg, 72*deg, 180*deg])

        self.addconfiguration("qh", self.qh)     # Home Position
        self.addconfiguration("qu", self.qu)





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


def genPose(robot: DHRobot, x: float, y: float, z: float, pitch = -90, roll = 0, minResidualAllowed = 0.0000009, printSol=False):
    """Generate the pose (joint angles) for a given point in space using inverse kinematics.  The end effector is always pointing away from the base.

    Args:
        x (float): x in meters
        y (float): y in meters
        z (float): z in meters
        pitch (int): The pitch of the end effector.  -90 is parallel to the ground, -90 -> -179 is pointing down, 0 is vertical pointing down
        roll  (int): This is the twist of the end effector. Positive angles (0 -> 180) rotate it clockwise when viewed from the centre (i.e. over the table), Negative angles (0 -> -180) rotate it anti-clockwise when viewed from the centre (i.e. over the table)
    Returns:
        IKsolution
    """
    T = SE3(x, y, z) * SE3.RPY(pitch, roll, (-1 * atan2(x, y)*(180/pi)), unit='deg', order='zyx')    # Pitch, Roll, Yaw
    print(T)
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
    sol = robot.ik_GN(T)                #  C++ version
    
    success = sol[1]
    iterations = sol[2]
    searches = sol[3]
    residual = sol[4]
    
    
    if residual > minResidualAllowed:
        sys.exit(f"A residual distance of {residual:.6f} meters is too great. The minimum allowed is {minResidualAllowed:.9f} meters")
        
    elif not success:
        reason = sol[5]
        print("Unable to solve inverse kinematice")
        print("  Reason: ", reason)
        sys.exit(sol.reason)

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
    

def genPose2(robot: DHRobot, x: float, y: float, z: float, pitch = -90, roll = 0, minResidualAllowed = 0.0000009, printSol=False):
    """Generate the pose (joint angles) for a given point in space using inverse kinematics.  The end effector is always pointing away from the base.

    Args:
        x (float): x in meters
        y (float): y in meters
        z (float): z in meters
        pitch (int): The pitch of the end effector.  -90 is parallel to the ground, -90 -> -179 is pointing down, 0 is vertical pointing down
        roll  (int): This is the twist of the end effector. Positive angles (0 -> 180) rotate it clockwise when viewed from the centre, Negative angles (0 -> -180) rotate it anti-clockwise
    Returns:
        IKsolution
    """
    # Convert angles to radians
    pitch_rad = pitch * deg
    roll_rad = roll * deg
    yaw_rad = -1 * atan2(x, y)

    # Calculate the approach vector (z-axis of end effector)
    ax = -np.sin(pitch_rad) * np.cos(yaw_rad)
    ay = -np.sin(pitch_rad) * np.sin(yaw_rad)
    az = -np.cos(pitch_rad)
    
    # Calculate the orientation vector (y-axis of end effector)
    ox = np.cos(roll_rad) * np.cos(yaw_rad) - np.sin(roll_rad) * np.sin(pitch_rad) * np.sin(yaw_rad)
    oy = np.cos(roll_rad) * np.sin(yaw_rad) + np.sin(roll_rad) * np.sin(pitch_rad) * np.cos(yaw_rad)
    oz = -np.sin(roll_rad) * np.cos(pitch_rad)

    T = SE3(x, y, z) * SE3.OA([ox, oy, oz], [ax, ay, az])
    print(T)
    sol = robot.ik_GN(T)
    
    residual = sol[4]
    success = sol[1]

    if residual > minResidualAllowed:
        sys.exit(f"A residual distance of {residual:.6f} meters is too great. The minimum allowed is {minResidualAllowed:.9f} meters")
        
    elif not success:
        print("Unable to solve inverse kinematice")
        print("  Reason: ", sol.reason)
        sys.exit(sol.reason)

    else:
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

print(robot.reach)


poses = []
poses.append(robot.qh)
poses.append(robot.qu)
#poses.append(np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 0*deg]))
#poses.append(np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 90*deg]))
#poses.append(np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 0*deg]))
#poses.append(np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 90*deg]))
#poses.append(np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 0*deg]))

#poses.append(genPose(robot, 0.4, 0.4, 0.30, pitch=180, printSol=False, minResidualAllowed=0.1))
poses.append(genPose(robot, 0, 0, 0.5, pitch=0, printSol=False))
poses.append(genPose(robot, 0, 0, 0.45, pitch=0, printSol=False))


trajPoses = multiTrajectory(poses, 30)



# print the joint angles of a trajectory as a series of poses (in degrees)
printPoses(trajPoses)

# plot joint coordinates V time
#rtb.xplot(trajPoses, block=False)

# Show the animation
#cm = robot.linkcolormap(['red', 'g', (0,0.5,0), '#0f8040', 'yellow', 'cyan'])
robot.plot(trajPoses, block=True, limits=[-0.6, 0.6, -0.6, 0.6, 0, 0.6], jointaxes=True, dt=0.1, vellipse=True)



