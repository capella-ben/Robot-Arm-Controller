import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from math import pi, atan2
import numpy as np
import warnings, sys


class mark4(DHRobot):
    """
    Class that models Axel's mark IV arm
    Defined joint configurations are:
        - qz, zero joint angle configuration
        - qr, vertical 'READY' configuration
        - qh, Homed position
        - q1, Extreme position
        - q2, The other extreme position
    .. note:: SI units of metres are used.
    """  # noqa
    def __init__(self):
        deg = pi/180

        # robot length values (metres)
        d1 = 0.142      # Distance from the table to the axis of the shoulder joint
        a1 = 0
        d2 = 0
        a2 = 0.140      # Distance from shoulder axis to elbow axis
        d3 = 0
        a3 = 0.0
        d4 = 0.255      # Distance from elbow axis to Wrist 2 axis
        a4 = 0
        d5 = 0
        a5 = 0
        d6 = 0.2        # Distance from wrist 2 to tip of gripper
        a6 = 0

        L = [
            RevoluteDH(                     # Table
                d=d1,
                a=a1,
                alpha=-pi/2,
                offset=-157.5*deg,
                flip=True,
                qlim=[-157.5*deg, 180*deg]   # <-  check!!!!!

            ),

            RevoluteDH(                     # Shoulder
                d=d2,
                a=a2,
                alpha=pi,
                qlim=[-90*deg, 90*deg],
                flip=True,
                offset=-90*deg
            ),

            RevoluteDH(                     # Elbow
                d=d3,
                a=a3,
                alpha=-pi/2,
                qlim=[-135*deg, 120*deg],
                flip=False,
                offset=-90*deg
            ),

            RevoluteDH(                     # Wrist1
                d=d4,
                a=a4,
                alpha=pi/2,
                qlim=[-90*deg, 90*deg],
                offset=90*deg
            ),

            RevoluteDH(                     # Wrist 2
                d=d5,
                a=a5,
                alpha=-pi/2,
                qlim=[-90*deg, 90*deg],
            ),

            RevoluteDH(                     # Wrist 3
                d=d6,
                a=a6,
                alpha=0,
                qlim=[-90*deg, 90*deg]
            )
        ]

        super().__init__(
            L,
            # basemesh="ABB/IRB140/link0.stl",
            name='Axel MarkIV',
            manufacturer='AXEL',
            #meshdir="meshes/ABB/IRB140"
            )

        self.qz = np.array([0, 0, 0, 0, 0, 0])
        self.qr = np.array([0*deg, 0*deg, 0*deg, 0*deg, 0*deg, 0*deg])
        self.qh = np.array([-157.5*deg, -90*deg, -135*deg, -90*deg, -90*deg, -90*deg])
        self.q1 = np.array([-157.5*deg, -90*deg, -135*deg, -90*deg, -90*deg, -90*deg])
        self.q2 = np.array([180*deg, 90*deg, 120*deg, 90*deg, 90*deg, 90*deg])

        self.addconfiguration("qz", self.qz)                                             # Zero position (Straight up)
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qh", self.qh)     # Home Position
        self.addconfiguration("q1", self.q1)     # one extreme
        self.addconfiguration("q2", self.q2)             # another extreme




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
    for step in qt:
        for joint in step:
            print("%.3f" % np.rad2deg(joint) + "\t", end="")
        print()


def getPose(robot: DHRobot, x: float, y: float, z: float, pitch = -90, roll = 0, minResidualAllowed = 0.0000009, printSol=False):
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

    

    #sol = robot.ikine_min(T, qlim=True, method='trust-constr')         # Slow but accurate method
    #sol = robot.ikine_GN(T, qlim=True, method='L-BFGS-B')         # Fast method
    sol = robot.ikine_GN(T)         # Fast method

    
    
    if sol.residual > minResidualAllowed:
        sys.exit(f"A residual distance of {sol.residual:.6f} meters is too great. The minimum allowed is {minResidualAllowed:.9f} meters")
        
    elif not sol.success:
        print("Unable to solve inverse kinematice")
        print("  Reason: ", sol.reason)
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
        return sol.q
    



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

poses = []
poses.append(robot.qh)
poses.append(robot.qz)
poses.append(getPose(robot, -0.35, 0.2, 0.2, pitch=-135, printSol=False))
poses.append(getPose(robot, -0.35, 0.2, 0.0, pitch=-170, printSol=False))
poses.append(getPose(robot, -0.35, 0.2, 0.2, pitch=-135, printSol=False))
poses.append(getPose(robot, 0.2, -0.2, 0.2, pitch=-135, printSol=False))
poses.append(getPose(robot, 0.2, -0.2, 0.0, pitch=-170, printSol=False))
poses.append(getPose(robot, 0.2, -0.2, 0.2, pitch=-135, printSol=False))
poses.append(robot.qz)

trajPoses = multiTrajectory(poses, 30)

# Test each joint
"""trajPoses = multiTrajectory([robot.qh, [-157.5*deg, 90*deg, -135*deg, -90*deg, -90*deg, -90*deg], 
    robot.qh, [-157.5*deg, -90*deg, 120*deg, -90*deg, -90*deg, -90*deg], 
    robot.qh, [-157.5*deg, -90*deg, -135*deg, 90*deg, -90*deg, -90*deg], 
    robot.qh, [-157.5*deg, -90*deg, -135*deg, -90*deg, 90*deg, -90*deg], 
    robot.qh, [-157.5*deg, -90*deg, -135*deg, -90*deg, -90*deg, 90*deg], 
    robot.qh], 30)
"""

# print the joint angles of a trajectory as a series of poses (in degrees)
#printPoses(trajPoses)

# plot joint coordinates V time
#rtb.xplot(trajPoses, block=False)

# Show the animation
#cm = robot.linkcolormap(['red', 'g', (0,0.5,0), '#0f8040', 'yellow', 'cyan'])
robot.plot(trajPoses, block=True, limits=[-0.6, 0.6, -0.6, 0.6, 0, 0.6], jointaxes=True, dt=0.1, vellipse=True)



