# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')


# 2. Start Simulation
sim.startSimulation()
print("Simulation Started! 🚀")

def transformMat(alpha, beta, gamma, tx, ty, tz):
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
        ])
    roty = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
        ])
    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0,0,1]
        ])
    rot_total = np.matmul(rotx, roty)
    rot_total = np.matmul(rot_total, rotz)
    trans_vector = np.array([
                    [tx],
                    [ty],
                    [tz]
                    ])
    R_t_3x4  = np.hstack((rot_total, trans_vector))
    homogeneous_row = np.array([[0, 0, 0, 1]])
    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4


# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")
LH_Handle = sim.getObject("/LH")
perp_Handle = sim.getObject("/Perp")

path_Handle = []
num_waypoints = 39

for i in range(0, num_waypoints):
    path_Handle.append(sim.getObject(f"/p[{i}]"))

sensor_FL = sim.getObject("/PioneerP3DX/ultrasonicSensor[0]") 
sensor_FC = sim.getObject("/PioneerP3DX/ultrasonicSensor[3]") 
sensor_FR = sim.getObject("/PioneerP3DX/ultrasonicSensor[4]") 
sensor_SR = sim.getObject("/PioneerP3DX/ultrasonicSensor[7]") 
my_sensors = [sensor_FL, sensor_FC, sensor_FR, sensor_SR]
sensor_names = ["Front-Left", "Front-Center", "Front-Right", "Side-Right"]

rw = 0.195/2
rb = 0.318/2
d = 0.05

dt = 0.01
x_dot_int = 0.0
y_dot_int = 0.0
gamma_int = 0.0

LH_distance = 1.5

x_odom = []
y_odom = []

map_x = []
map_y = []
robot_path_x = [] 
robot_path_y = []

try:
    # 4. Main Loop
    start_time = time.time()
    elapsed_prev = 0.0
    
    while (time.time() - start_time) < 45:
        
        elapsed = time.time() - start_time

        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        p3dx_position = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_orientation = sim.getObjectOrientation(p3dx, sim.handle_world)

        LH_position_to_world = transformMat(0, 0, p3dx_orientation[2], p3dx_position[0], p3dx_position[1], p3dx_position[2]) @ np.array([[LH_distance], [0], [0], [1]])
        LH_position_to_world = LH_position_to_world[:3, :]

        path_points = []
        for i in range(len(path_Handle)):
            path_point_position = sim.getObjectPosition(path_Handle[i], sim.handle_world)
            path_points.append(np.array(path_point_position).reshape((3,1)))

        vec_AB = []
        for i in range(len(path_points)-1):
            vec_AB.append(path_points[i+1] - path_points[i])
        vec_AB.append(path_points[0] - path_points[-1])  # Close the loop

        vec_ALH = []
        for i in range(len(path_points)):
            A = path_points[i]
            vec_ALH.append(LH_position_to_world - A)

        scalar_proj_points = []
        for i in range(len(vec_AB)):
            AB = vec_AB[i]
            ALH = vec_ALH[i]

            scalar_proj = (np.dot(ALH.T, AB) / (np.linalg.norm(AB)**2))[0][0]
            
            if scalar_proj < 0:
                scalar_proj = 0.0
            elif scalar_proj > 1:
                scalar_proj = 1.0
                
            A = path_points[i]
            scalar_proj_point = A + (scalar_proj * AB)
            scalar_proj_points.append(scalar_proj_point)

        closest_index = 0
        min_distance = np.linalg.norm(scalar_proj_points[0] - LH_position_to_world)
        for i in range(1, len(scalar_proj_points)):
            distance = np.linalg.norm(scalar_proj_points[i] - LH_position_to_world)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        robot_path_x.append(p3dx_position[0])
        robot_path_y.append(p3dx_position[1])

        for i, sensor in enumerate(my_sensors):
            result, distance, hit_point, detected_obj, normal_vec = sim.readProximitySensor(sensor)
            if result == 1:
                hit_world = sim.multiplyVector(sim.getObjectMatrix(sensor, sim.handle_world), hit_point)
                
                # We can comment out the print statement so it doesn't spam your terminal anymore
                # print(f"[{sensor_names[i]}] Wall/Desk found at X: {hit_world[0]:.2f}, Y: {hit_world[1]:.2f}")
                
                # Save the obstacle coordinates!
                map_x.append(hit_world[0])
                map_y.append(hit_world[1])

        desired_position = scalar_proj_points[closest_index]

        T_world_robot = transformMat(0,0, p3dx_orientation[2], p3dx_position[0], p3dx_position[1], p3dx_position[2])
        desired_position_wrt_robot = np.linalg.inv(T_world_robot) @ np.append(desired_position, np.array([[1]]), axis=0)
        desired_position_wrt_robot = desired_position_wrt_robot[:3, :]

        x_err = float(desired_position_wrt_robot[0][0])
        y_err = float(desired_position_wrt_robot[1][0])
        ed = math.sqrt(x_err**2 + y_err**2)
        eh = math.atan2(y_err, x_err)

        vx = 0.3 * ed
        wx = 0.9 * eh

        wr_vel = (vx + (rb*wx)/2)/rw   
        wl_vel = (vx - (rb*wx)/2)/rw

        sim.setJointTargetVelocity(p3dx_rw, wr_vel)
        sim.setJointTargetVelocity(p3dx_lw, wl_vel)

        sim.setObjectPosition(LH_Handle, sim.handle_world, LH_position_to_world.flatten().tolist())
        sim.setObjectPosition(perp_Handle, sim.handle_world, desired_position.flatten().tolist())

        for i, sensor in enumerate(my_sensors):
            result, distance, hit_point, detected_obj, normal_vec = sim.readProximitySensor(sensor)
            if result == 1:
                hit_world = sim.multiplyVector(sim.getObjectMatrix(sensor, sim.handle_world), hit_point)
                print(f"[{sensor_names[i]}] Wall/Desk found at X: {hit_world[0]:.2f}, Y: {hit_world[1]:.2f}")

finally:
    sim.setJointTargetVelocity(p3dx_rw, 0)
    sim.setJointTargetVelocity(p3dx_lw, 0)
    sim.stopSimulation()
    print("\nSimulation Stopped! 🏁")

    print("Generating Map... Check your taskbar if it doesn't pop over VS Code!")
    
    plt.figure(figsize=(10, 8))
    plt.scatter(map_x, map_y, c='black', s=10, label='Obstacles (Desks/Walls)')
    plt.plot(robot_path_x, robot_path_y, c='blue', linewidth=2, label='Robot Path')
    plt.title("B202 Exploration Map")
    plt.xlabel("X Position (meters)")
    plt.ylabel("Y Position (meters)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal') 
    plt.show()