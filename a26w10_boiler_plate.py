# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Get Handles & Constants
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor") 
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")

rw = 0.195/2  # wheel radius [cite: 102]
L = 0.381     # Jarak antar roda (body diameter/track width) [cite: 105]

# List untuk menampung data plot
time_data = []
wr_list = []
wl_list = []
vx_list = []
w_list = []

# %%
try:
    # 4. Main Loop (Run for 30 seconds)
    start_time = time.time()
    print("Recording data...")
    
    while (time.time() - start_time) < 30:
        elapsed = time.time() - start_time
        
        # Ambil kecepatan sudut roda (rad/s) [cite: 88, 114]
        wr_vel = sim.getJointTargetVelocity(p3dx_RW) 
        wl_vel = sim.getJointTargetVelocity(p3dx_LW) 
        
        # Hitung kecepatan linear tiap roda (m/s) [cite: 40]
        vr = wr_vel * rw
        vl = wl_vel * rw
        
        # Hitung kecepatan body (Rumus sesuai slide & gambar) [cite: 44, 45, 70]
        vx = (vr + vl) / 2
        wx = (vr - vl) / L
        
        # Simpan data ke list
        time_data.append(elapsed)
        wr_list.append(wr_vel)
        wl_list.append(wl_vel)
        vx_list.append(vx)
        w_list.append(wx)
        
        # Tampilkan di log CoppeliaSim [cite: 125]
        sim.addLog(1, f"Vx: {vx:.2f} m/s, W: {wx:.2f} rad/s")
        print(f"Time: {elapsed:.1f}s | Vx: {vx:.2f}", end="\r")
        
        time.sleep(0.05) 

finally:
    # 5. Stop Simulation
    sim.stopSimulation()
    print("\nSimulation Stopped. Generating Plots...")

    # 6. Plotting
    plt.figure(figsize=(10, 8))

    # Plot Kecepatan Roda [cite: 131, 133]
    plt.subplot(2, 1, 1)
    plt.plot(time_data, wr_list, label='Right Wheel ($\dot{\\varphi}_R$)')
    plt.plot(time_data, wl_list, label='Left Wheel ($\dot{\\varphi}_L$)')
    plt.title('P3DX Joint Velocity')
    plt.ylabel('Velocity (rad/s)')
    plt.legend()
    plt.grid(True)

    # Plot Kecepatan Body [cite: 136, 137]
    plt.subplot(2, 1, 2)
    plt.plot(time_data, vx_list, label='Linear Velocity ($v_x$)')
    plt.plot(time_data, w_list, label='Angular Velocity ($\omega$)', color='orange')
    plt.title('P3DX Body Velocity')
    plt.xlabel('Time (sec)')
    plt.ylabel('Velocity (m/s or rad/s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()