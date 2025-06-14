# Importing necessary libraries

import numpy as np
import control
import matplotlib.pyplot as plt

# defining system parameters

m = 50 # mass
b = 50 # damping

num = [1]
den = [m,b]

plant = control.tf(num, den)

# Defining the pid controller

Kp = 800
Ki = 40
Kd = 100

pid = control.tf([Kd,Kp,Ki],[1,0])

# Creating a closed loop system

system = control.feedback(pid * plant,1)

# Simulate the step response

t,y = control.step_response(system)

plt.plot(t,y)
plt.title("Vehicle Speed Control using PID")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.grid()
plt.show()
