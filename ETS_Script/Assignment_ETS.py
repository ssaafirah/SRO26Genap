import time
import math
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ==========================================
# 1. SETUP & KONEKSI
# ==========================================
client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()
print("Pertandingan Sepak Bola Dimulai ⚽")

# ==========================================
# 2. INISIALISASI OBJEK
# ==========================================
bola = sim.getObject("/Bola_Merah")
gawang_putih = sim.getObject("/Gawang_Putih")
gawang_kuning = sim.getObject("/Gwang_Kuning") 

# Kiper 1 (Jaga Gawang Putih)
gk = sim.getObject("/Robot_Pemain")
gk_rw = sim.getObject("/Robot_Pemain/rightMotor")
gk_lw = sim.getObject("/Robot_Pemain/leftMotor")

# Pemain 1 (Striker)
s1 = sim.getObject("/Robot_Lawan_01")
s1_rw = sim.getObject("/Robot_Lawan_01/rightMotor")
s1_lw = sim.getObject("/Robot_Lawan_01/leftMotor")

# Kiper 2 (Jaga Gawang Kuning)
s2 = sim.getObject("/Robot_Lawan_02")
s2_rw = sim.getObject("/Robot_Lawan_02/rightMotor")
s2_lw = sim.getObject("/Robot_Lawan_02/leftMotor")

rw = 0.195 / 2  
rb = 0.381 / 2  

# titik awal kedua kiper
pos_kiper1_awal = sim.getObjectPosition(gk, sim.handle_world)
pos_kiper2_awal = sim.getObjectPosition(s2, sim.handle_world)

# ==========================================
# 3. FUNGSI NAVIGASI KIPER
# ==========================================
def arahkan_odometri_kiper(robot_handle, target_pos, mode="kejar_bola"):
    posisi_robot = sim.getObjectPosition(robot_handle, sim.handle_world)
    orientasi_robot = sim.getObjectOrientation(robot_handle, sim.handle_world)
    theta = orientasi_robot[2]
    
    T_W_R = np.array([
        [math.cos(theta), -math.sin(theta), posisi_robot[0]],
        [math.sin(theta),  math.cos(theta), posisi_robot[1]],
        [0,               0,               1]
    ])
    
    target_pos_world = np.array([[target_pos[0]], [target_pos[1]], [1]])
    target_pos_robot = np.linalg.inv(T_W_R) @ target_pos_world
    
    dist_x = target_pos_robot[0, 0] 
    dist_y = target_pos_robot[1, 0] 
    
    distance = math.sqrt(dist_x**2 + dist_y**2)
    angle_to_target = math.atan2(dist_y, dist_x)

    if mode == "diam_tapi_awas":
        vx = 0.0
        wx = 2.0 * angle_to_target 
    elif mode == "kejar_bola":
        vx = min(0.6 * distance, 1.0) 
        wx = 3.0 * angle_to_target 
    elif mode == "nendang_gawang":
        vx = 1.3 
        wx = 3.0 * angle_to_target 
    elif mode == "kiper_balik":
        vx = min(0.5 * distance, 0.8) 
        wx = 2.0 * angle_to_target

    wx = max(min(wx, 2.5), -2.5)

    wr_vel = (vx + (rb * wx)) / rw
    wl_vel = (vx - (rb * wx)) / rw
    
    wr_vel = max(min(wr_vel, 8.0), -8.0)
    wl_vel = max(min(wl_vel, 8.0), -8.0)
    
    return wl_vel, wr_vel

# Fungsi otak kiper untuk Kiper 1 dan Kiper 2
def otak_kiper(robot_handle, lw_handle, rw_handle, pos_awal, pos_gawang_jaga, pos_gawang_target, pos_bola):
    pos_sekarang = sim.getObjectPosition(robot_handle, sim.handle_world)
    jarak_bola_ke_gawang = math.hypot(pos_bola[0] - pos_gawang_jaga[0], pos_bola[1] - pos_gawang_jaga[1])
    jarak_kiper_ke_pos_awal = math.hypot(pos_sekarang[0] - pos_awal[0], pos_sekarang[1] - pos_awal[1])
    jarak_kiper_ke_bola = math.hypot(pos_sekarang[0] - pos_bola[0], pos_sekarang[1] - pos_bola[1])

    if jarak_bola_ke_gawang < 3.0:
        if jarak_kiper_ke_bola < 0.4:
            wl, wr = arahkan_odometri_kiper(robot_handle, pos_gawang_target, mode="nendang_gawang")
        else:
            wl, wr = arahkan_odometri_kiper(robot_handle, pos_bola, mode="kejar_bola")
    else:
        if jarak_kiper_ke_pos_awal > 0.2:
            wl, wr = arahkan_odometri_kiper(robot_handle, pos_awal, mode="kiper_balik")
        else:
            wl, wr = arahkan_odometri_kiper(robot_handle, pos_bola, mode="diam_tapi_awas")
            
    sim.setJointTargetVelocity(lw_handle, wl)
    sim.setJointTargetVelocity(rw_handle, wr)

# ==========================================
# 4. MAIN LOOP
# ==========================================
try:
    start_time = time.time()
    durasi_main = 60 
    
    while (time.time() - start_time) < durasi_main:
        p_bola = sim.getObjectPosition(bola, sim.handle_world)
        p_gw_putih = sim.getObjectPosition(gawang_putih, sim.handle_world)
        p_gw_kuning = sim.getObjectPosition(gawang_kuning, sim.handle_world)
        
        # JALANKAN KIPER 1 
        otak_kiper(gk, gk_lw, gk_rw, pos_kiper1_awal, p_gw_putih, p_gw_kuning, p_bola)
        
        # JALANKAN KIPER 2 
        otak_kiper(s2, s2_lw, s2_rw, pos_kiper2_awal, p_gw_kuning, p_gw_putih, p_bola)
        
        # 3. JALANKAN PEMAIN 1 
        p3dx_position = sim.getObjectPosition(s1, sim.handle_world)
        p3dx_orientation = sim.getObjectOrientation(s1, sim.handle_world)
        sphere_position = p_bola
        
        theta = p3dx_orientation[2]
        T_W_R = np.array([
            [math.cos(theta), -math.sin(theta), p3dx_position[0]],
            [math.sin(theta),  math.cos(theta), p3dx_position[1]],
            [0,               0,               1]
        ])
        
        sphere_pos_world = np.array([[sphere_position[0]], [sphere_position[1]], [1]])
        sphere_pos_robot = np.linalg.inv(T_W_R) @ sphere_pos_world
        
        dist_x = sphere_pos_robot[0, 0] 
        dist_y = sphere_pos_robot[1, 0] 
        
        angle_to_sphere = math.atan2(dist_y, dist_x)
        distance = math.sqrt(dist_x**2 + dist_y**2)
        
        dribble_range = 0.35 
        
        if distance > dribble_range:
            if abs(angle_to_sphere) > 0.8: 
                vx_s1 = 0.0 
                wx_s1 = 4.0 * angle_to_sphere 
            else:
                vx_s1 = min(0.6 * distance, 1.2)  
                wx_s1 = 3.0 * angle_to_sphere 
        else:
            vx_s1 = 0.8 
            wx_s1 = 4.0 * angle_to_sphere 
            
        wr_vel_s1 = (vx_s1 + (rb * wx_s1)) / rw
        wl_vel_s1 = (vx_s1 - (rb * wx_s1)) / rw
        
        wr_vel_s1 = max(min(wr_vel_s1, 10.0), -10.0)
        wl_vel_s1 = max(min(wl_vel_s1, 10.0), -10.0)
        
        sim.setJointTargetVelocity(s1_rw, wr_vel_s1)
        sim.setJointTargetVelocity(s1_lw, wl_vel_s1)
            
        time.sleep(0.01)

finally:
    for motor in [gk_lw, gk_rw, s1_lw, s1_rw, s2_lw, s2_rw]:
        sim.setJointTargetVelocity(motor, 0)
    sim.stopSimulation()
    print("Pertandingan Selesai! 🏁")