import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

pwm = np.array(
    [
        -150,
        -140,
        -130,
        -120,
        -110,
        -100,
        -90,
        -80,
        -70,
        -60,
        -50,
        -40,
        -30,
        -20,
        -10,
        0,
        10,
        20,
        30,
        40,
        50,
        60,
        70,
        80,
        90,
        100,
        110,
        120,
        130,
        140,
        150,
        160,
        170,
        180,
        190,
        200,
        210,
        220,
        230,
        240,
        250,
        255,
    ]
)

speed = np.array(
    [
        -2.782,
        -2.736,
        -2.449,
        -2.362,
        -2.135,
        -1.937,
        -1.730,
        -1.559,
        -1.312,
        -1.092,
        -0.952,
        -0.693,
        -0.525,
        -0.323,
        -0.115,
        0.000,
        0.141,
        0.315,
        0.543,
        0.742,
        0.934,
        0.998,
        1.370,
        1.546,
        1.760,
        1.541,
        2.083,
        2.352,
        2.367,
        2.707,
        2.984,
        2.981,
        3.042,
        3.174,
        3.506,
        3.670,
        3.764,
        3.619,
        3.963,
        3.666,
        4.185,
        4.141,
    ]
)

trend = UnivariateSpline(pwm, speed, s=1)

pwm_dense = np.linspace(pwm.min(), pwm.max(), 1000)
speed_dense = trend(pwm_dense)

plt.figure(figsize=(9, 5))

plt.scatter(pwm, speed, s=40, label="Calibration data")

plt.plot(pwm_dense, speed_dense, linewidth=2, label="Spline fit")

plt.axhline(0, color="k", linewidth=0.8)
plt.axvline(0, color="k", linewidth=0.8)

plt.xlabel("PWM")
plt.ylabel("Vehicle speed [m/s]")
plt.title("PWM → Vehicle Speed Calibration")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
