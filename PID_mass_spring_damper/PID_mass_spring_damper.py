""" Script that generates various plots based on three different controllers that achieve a percent of overshoot of 10, 50 and 100%
    Author: Andres Pulido
    September 2021
"""

import numpy as np
import matplotlib.pyplot as plt  # MATLAB plotting functions
from control import TransferFunction as tf  # Load the controls systems library
import control

kp1 = 10  # kp to obtain 10% overshoot
ki = 0  # arbitrary value to reduce SSE
kd = 0
mass_kg = 1
damping_Ns = 10
spring_const_Npm = 5
time_s = np.linspace(0, 10, 1000)

# initialize systems
mass_spring_damper = tf(
    [1], [mass_kg, damping_Ns, spring_const_Npm])  # Plant (P)
actuator = tf([10], [1, 10])

# control.series is the same as actuator*mass_spring_damper in bloack diagram algebra
open_loop = control.series(actuator, mass_spring_damper)
t1, y1 = control.step_response(open_loop, T=time_s)

overshoot = 0
while overshoot < 10:
    kp1 += 1
    controller1 = tf([kd, kp1, ki], [1, 0])  # Controller (k)
    loop_gain1 = control.series(controller1, open_loop)  # Loop Gain (PK)
    tracking1 = control.feedback(loop_gain1, 1)  # same as PK/(1+PK)
    t2, y2 = control.step_response(tracking1, T=time_s)
    overshoot = (max(y2) - 1)*100

kp2 = kp1
while overshoot < 50:
    kp2 += 1
    controller2 = tf([kd, kp2, ki], [1, 0])
    loop_gain2 = control.series(controller2, open_loop)
    tracking2 = control.feedback(loop_gain2, 1)
    t3, y3 = control.step_response(tracking2, T=time_s)
    overshoot = (max(y3) - 1)*100

kp3 = kp2
while overshoot < 100:
    kp3 += 1
    controller3 = tf([kd, kp3, ki], [1, 0])
    loop_gain3 = control.series(controller3, open_loop)
    tracking3 = control.feedback(loop_gain3, 1)
    t4, y4 = control.step_response(tracking3, T=time_s)
    overshoot = (max(y4) - 1)*100

disturbance = tf([0.5], [1, 0, 1])
tracking_disturbance1 = control.feedback(control.series(controller1, control.parallel(
    actuator, disturbance), mass_spring_damper), 1)  # control.parallel is H1+H2
t5, y5 = control.step_response(tracking_disturbance1, T=time_s)
tracking_disturbance2 = control.feedback(control.series(controller2, control.parallel(
    actuator, disturbance), mass_spring_damper), 1)
t6, y6 = control.step_response(tracking_disturbance2, T=time_s)
tracking_disturbance3 = control.feedback(control.series(controller3, control.parallel(
    actuator, disturbance), mass_spring_damper), 1)
t7, y7 = control.step_response(tracking_disturbance3, T=time_s)


print("mass spring damper: ", mass_spring_damper)
print("actuator: ", actuator)
print("plant: ", open_loop)
print("controller1: ", controller1)
print("loop gain: ", loop_gain3)
print("tracking: ", tracking1)
print("disturbance: ", disturbance)

plt.figure(1)
plt.plot(t2, y2, label='Closed loop k1', linewidth=2)
plt.plot(t3, y3, label='Closed loop k2 ', linewidth=2)
plt.plot(t4, y4, label='Closed loop k3', linewidth=1)
plt.plot(t1, y1, label='Open loop ', linewidth=2)
plt.title("Step Response")
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.show

plt.figure(2)
plt.plot(t5, y5, label='k1', linewidth=2)
plt.plot(t6, y6, label='k2 ', linewidth=2)
plt.plot(t7, y7, label='k3', linewidth=1)
plt.title("Step Response with force disturbance of (0.5)sin(t)")
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.show


noise = 1 + np.random.uniform(-1, 1, 1000)
t8, y8 = control.input_output_response(
    control.tf2io(tracking1), T=time_s, U=noise)
t9, y9 = control.input_output_response(
    control.tf2io(tracking2), T=time_s, U=noise)
t10, y10 = control.input_output_response(
    control.tf2io(tracking3), T=time_s, U=noise)
plt.figure(3)
plt.plot(t2, y2, label='Closed loop k1', linewidth=2)
plt.plot(t8, y8, label='Closed loop k2', linewidth=2)
plt.plot(t10, y10, label='Closed loop k3', linewidth=1)
plt.title("Step Response with Noise +- 0.1m in sensor")
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.show

plt.figure(4)
control.bode(loop_gain1,
             dB=True, deg=True, margins=True, label='k1')
control.bode(loop_gain2,
             dB=True, deg=True, margins=True, label='k2')
control.bode(loop_gain3,
             dB=True, deg=True, margins=True, label='k3')
plt.legend()

# sensitivity hand calculated as 1/(1+PK)
sensitivity1 = tf([1, 20, 105, 50], [1, 20, 105, 50, 510])
sensitivity2 = tf([1, 20, 105, 50], [1, 20, 105, 50, 1130])
sensitivity3 = tf([1, 20, 105, 50], [1, 20, 105, 50, 2120])

plt.figure(5)
control.bode(sensitivity1,
             dB=False, deg=True, margins=True, label='k1')
control.bode(sensitivity2,
             dB=False, deg=True, margins=True, label='k2')
control.bode(sensitivity3,
             dB=False, deg=True, margins=True, label='k3')
plt.legend()

plt.figure(6)
control.bode(tracking1,
             dB=False, deg=True, margins=True, label='k1')  # Tracking = Complementary Sensitivity
control.bode(tracking2,
             dB=False, deg=True, margins=True, label='k2')
control.bode(tracking3,
             dB=False, deg=True, margins=True, label='k3')
plt.legend()

print("end")
