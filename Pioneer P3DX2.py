# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
print("Program Started")
client = RemoteAPIClient()
sim = client.require('sim')
# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
p3dx = sim.getObject("/PioneerP3DX")
rw= 0.195/2 #wheel radius
rb= 0.381/2 #body radius
d= 0.05
x_dot_int =0
y_dot_int =0
gamma_int=0

# Metode 1: pakai getObjectOrientation
x_odom =[]
y_odom =[]

# Metode 2: pakai integrasi gamma (angular velocity)
x_odom2 =[]
y_odom2 =[]
x_dot_int2 = 0
y_dot_int2 = 0

# Data storage
t_log  = []
wr_log = []
wl_log = []
vx_log = []
wx_log = []

# %%
try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 45:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")

        #time difference
        dt= elapsed-elapsed_prev
        elapsed_prev=elapsed
        
        wr_vel=sim.getJointTargetVelocity(p3dx_RW)
        wl_vel=sim.getJointTargetVelocity(p3dx_LW)  

        vx=(wr_vel+wl_vel)*rw/2
        wx=(wr_vel-wl_vel)*rw/rb  
        
        # getObjectOrientation
        euler_angle=sim.getObjectOrientation(p3dx, sim.handle_world)
        x_dot = vx*math.cos(euler_angle[2])
        y_dot = vx*math.sin(euler_angle[2])
        x_dot_int = x_dot_int+x_dot*dt
        y_dot_int = y_dot_int+y_dot*dt
        x_odom.append(x_dot_int)
        y_odom.append(y_dot_int)

        # integrasi gamma
        gamma_int = gamma_int + wx*dt
        x_dot2 = vx*math.cos(gamma_int)
        y_dot2 = vx*math.sin(gamma_int)
        x_dot_int2 = x_dot_int2 + x_dot2*dt
        y_dot_int2 = y_dot_int2 + y_dot2*dt
        x_odom2.append(x_dot_int2)
        y_odom2.append(y_dot_int2)

        sim.addLog(1, f"x_dot:{x_dot:.1f}m/s, y_dot:{y_dot:.1f}m/s, x_int:{x_dot_int:.1f}m, y_int:{y_dot_int:.1f}m, gamma_int:{gamma_int:.1f}rad, dt{dt:.1f}s")
    
        time.sleep(0.1) 

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")

# --- Plot perbandingan dua metode ---
plt.figure(figsize=(10, 6))
plt.plot(x_odom, y_odom, 'b-', label="Metode 1: Orientation Sensor")
plt.plot(x_odom2, y_odom2, 'r--', label="Metode 2: Integrasi Angular Velocity")
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Path from Odometry')
plt.legend()
plt.grid(True)
plt.show()