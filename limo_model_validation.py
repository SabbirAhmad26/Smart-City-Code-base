#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# Bicycle model differential equations
def bicycle_model(state, t, delta):
    """
    :param state: State vector [x, y, theta, v]
    :param t: Time [s]
    :param delta: Steering angle [rad]
    :return: Derivatives [dx/dt, dy/dt, dtheta/dt, dv/dt]
    """
    L = 0.2  # Wheelbase [m]
    x, y, theta, v = state
    dxdt = v * np.cos(theta)
    dydt = v * np.sin(theta)
    dthetadt = v / L * np.tan(delta)
    # dvdt = 0.2 if t<=5 else -0.2  # Assuming constant velocity for simplicity
    dvdt = math.sin(math.pi*t)
    return [dxdt, dydt, dthetadt, dvdt]

class listener:
    '''
    This class gets the ROS messages for position and orientation and processes them for
    further use. 
    '''
    def __init__(self):
        self.data = None
        self.xprev = 0
        self.yprev = 0

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.data = data
        
    def listener(self):
        rospy.init_node("listen_pos", anonymous=True)
        rospy.Subscriber("/vrpn_client_node/LIMO_000770/pose", PoseStamped, self.callback)
    
    def quaternion_to_euler(self):
        # return value Z corresponds to Yaw angle. 
        x = self.data.pose.orientation.x
        y = self.data.pose.orientation.y
        z = self.data.pose.orientation.z
        w = self.data.pose.orientation.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    def get_yaw(self):
        # Uses two consecutive position data to calculate yaw.
        dx = self.data.pose.position.x - self.xprev
        dy = self.data.pose.position.y - self.yprev

        self.xprev = self.data.pose.position.x
        self.yprev = self.data.pose.position.y

        yaw = math.atan2(dx, -dy)
        return yaw


# Simulation parameters
listener_ins = listener()  # instance of the "listener" class
# offsets are used to set initial value of simulation
offset_x = None
offset_y = None
listener_ins.listener()  # get ros messages
while listener_ins.data is None:
    listener_ins.listener()
if listener_ins.data is not None:
    if offset_x is None:
        offset_x = listener_ins.data.pose.position.x
        offset_y = listener_ins.data.pose.position.y
        ## to calculate yaw data initialize previous position point.
        listener_ins.xprev = offset_x
        listener_ins.yprev = offset_y
        Pitch,Yaw,Roll = listener_ins.quaternion_to_euler()

initial_state = [-offset_y, offset_x, (Roll)*(np.pi/180), 0.0]  # Initial state [x, y, theta, v]
steering_angle = np.deg2rad(0)  # Steering angle (radians) 25 previously
L = 0.2 # Wheelbase [m]
min_turning_radius = 0.4 #  minimum turning radius 0.4 [m]
max_steering_angle = math.atan(L/0.4) # max steering angle [rad]
if steering_angle >=max_steering_angle:
    raise Exception("Steering angle is higher than max steering angle")
simulation_time = np.linspace(0, 10, 100)  # Time vector

# Solve the differential equations
solution = odeint(bicycle_model, initial_state, simulation_time, args=(steering_angle,))


## pass velocity and steering angle to LIMO to observe how it behaves. 
from pylimo import limo
limo=limo.LIMO()
limo.EnableCommand()

# arrays to hold data from Limo
vel_limo = np.zeros(100)
steering_angle_limo = np.zeros(100)
opt_x_pos = np.zeros(100)
opt_y_pos = np.zeros(100)
opt_yaw = np.zeros(100)
imu_yaw = np.zeros(100)

print(f"offset_x:{offset_x} offset_y:{offset_y}")

for i,v in enumerate(solution[:,3]):
    limo.SetMotionCommand(linear_vel=v, steering_angle=steering_angle)
    vel_limo[i] = limo.GetLinearVelocity()
    steering_angle_limo[i] = limo.GetSteeringAngle()
    
    listener_ins.listener()
    if listener_ins.data is not None:
        opt_x_pos[i] = listener_ins.data.pose.position.x
        opt_y_pos[i] =listener_ins.data.pose.position.y
        Pitch,Yaw,Roll = listener_ins.quaternion_to_euler()
        print("*****",Pitch,Yaw,Roll)
        imu_yaw[i] = limo.GetIMUYawData()*(np.pi/180)
        opt_yaw[i] = Roll*(np.pi/180)
        # opt_yaw[i] = listener_ins.get_yaw()
        # opt_yaw[i] = listener_ins.opti_yaw_range_arranged()
        print(opt_yaw[i])
    else:
        print("Skipping ros Messages!!!")
    time.sleep(0.1)  ## determined by spacing in simulation time.



# Plot the results
plt.figure(figsize=(12, 9))

# Plot 2D position
plt.subplot(3, 2, 1)
plt.plot(solution[:, 0], solution[:, 1])
# In order to align optitrack and simulation axis this x and y interchanged. 
plt.plot(-opt_y_pos, opt_x_pos) 
plt.title('2D Position')
plt.legend(['Simulation', 'Limo-Optitrack'])
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')

# Plot Yaw
plt.subplot(3, 2, 2)
plt.plot(simulation_time, solution[:, 2])
plt.plot(simulation_time, opt_yaw)
plt.plot(simulation_time, imu_yaw)
plt.title('Theta (Orientation)')
plt.legend(["Sim", "Opt","IMU"])
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')

# Plot Velocity
plt.subplot(3, 2, 3)
plt.plot(simulation_time, solution[:, 3])
plt.plot(simulation_time, vel_limo)
plt.legend(['Simulation', 'Limo'])
plt.title('Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')

# plt.subplot(2, 2, 4)
# plt.plot(simulation_time, math.sin(math.pi*simulation_time[:]) )
# plt.title('Input (m^2/s)')
# plt.xlabel('Time (s)')
# plt.ylabel('Acceleration(m^2/s)')

#plot time vs X
plt.subplot(3,2,4)
plt.plot(simulation_time, solution[:, 0])
plt.plot(simulation_time, -opt_y_pos)
plt.title('X')
plt.xlabel('Time (s)')
plt.ylabel('X position (m)')
plt.legend(['Simulation', 'Limo'])

#plot time vs Y
plt.subplot(3,2,5)
plt.plot(simulation_time, solution[:, 1])
plt.plot(simulation_time, opt_x_pos)
plt.title('Y')
plt.xlabel('Time (s)')
plt.ylabel('Y position (m)')
plt.legend(['Simulation', 'Limo'])

# vel_error =  np.linalg.norm(vel_limo-solution[:,3])
# print(vel_error)
# print(opt_x_pos)
plt.tight_layout()
plt.show()
