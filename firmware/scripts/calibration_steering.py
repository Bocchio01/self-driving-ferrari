import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

STEERING_ARM_LENGTH = 0.010
SERVO_ARM_LENGTH = 0.008
CONNECTION_ROD_LENGTH = 0.040

# Calibration data from the steering calibration procedure
pwm = np.array([115, 110, 100, 90, 60, 50, 40, 35])
angle_rad = np.array(
    [
        +0.35201,
        +0.31479,
        +0.23957,
        +0.14085,
        -0.14171,
        -0.23426,
        -0.32429,
        -0.35421,
    ]
)


# Define the steering model function based on the geometry of the steering mechanism
def steering_model(params, pwm, return_points=False):
    offset, h, pwm_to_deg = params

    beta0 = np.arcsin((h - SERVO_ARM_LENGTH) / CONNECTION_ROD_LENGTH)
    Bx0 = 0 + CONNECTION_ROD_LENGTH * np.cos(beta0)

    alpha = np.deg2rad((pwm - offset) * pwm_to_deg + 90)
    Ax = 0 + SERVO_ARM_LENGTH * np.cos(alpha)
    Ay = 0 + SERVO_ARM_LENGTH * np.sin(alpha)

    beta = np.arcsin((h - Ay) / CONNECTION_ROD_LENGTH)
    Bx = Ax + CONNECTION_ROD_LENGTH * np.cos(beta)
    By = Ay + CONNECTION_ROD_LENGTH * np.sin(beta)

    gamma = -np.arcsin((Bx - Bx0) / STEERING_ARM_LENGTH)

    if return_points:
        return gamma, Ax, Ay, Bx, By

    return gamma


# Define the inverse steering model function to compute PWM from a desired steering angle
def steering_model_inverse(params, angle_rad):
    offset, h, pwm_scale = params

    beta0 = np.arcsin((h - SERVO_ARM_LENGTH) / CONNECTION_ROD_LENGTH)
    Bx0 = CONNECTION_ROD_LENGTH * np.cos(beta0)

    Bx = Bx0 - STEERING_ARM_LENGTH * np.sin(angle_rad)
    By = h

    # We adopt cosine law to find the servo angle (alpha)
    D = np.sqrt(Bx**2 + By**2)
    phi = np.arctan2(By, Bx)

    # R_c^2 = R_s^2 + D^2 - 2 * R_s * D * cos(theta)
    cos_theta = (SERVO_ARM_LENGTH**2 + D**2 - CONNECTION_ROD_LENGTH**2) / (
        2 * SERVO_ARM_LENGTH * D
    )
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))

    alpha = phi + theta
    pwm = ((np.rad2deg(alpha) - 90) / pwm_scale) + offset

    return pwm


# Use least squares optimization to fit the steering model to the calibration data
result = least_squares(
    lambda params: steering_model(params, pwm) - angle_rad,
    [75, 0.02, 180 / 255],
)
offset, h, pwm_to_deg = result.x


print("Fitted parameters:")
print(f"offset = {offset:.8f} PWM")
print(f"h      = {1e3 * h:.8f} mm")
print(f"scale  = {pwm_to_deg:.8f} deg/PWM")
print(f"cost   = {1e3 * result.cost:.8f}")

print("\nSteering angle to PWM mapping:")
max_abs_angle = np.rad2deg(np.max(np.abs(angle_rad)))
# Round down to the nearest 0.5 degree
max_abs_angle = np.floor(max_abs_angle * 2) / 2.0
for angle in [-max_abs_angle, 0, max_abs_angle]:
    pwm_value = steering_model_inverse(result.x, np.deg2rad(angle))
    print(
        f"angle = {angle:+3.1f} deg ({np.deg2rad(angle):.5f} rad) -> pwm = {pwm_value:.5f}"
    )


# Generate a smooth curve for the fitted model
pwm_curve = np.linspace(min(pwm), max(pwm), 500)
angle_curve = steering_model(result.x, pwm_curve)

angle_fit, servo_x, servo_y, steering_x, steering_y = steering_model(
    result.x,
    pwm,
    return_points=True,
)


# Plot the geometry of the fitted model
plt.figure(figsize=(6, 6))
plt.scatter(pwm, angle_rad, label="Calibration data")
plt.plot(pwm_curve, angle_curve, label="Fitted model")
plt.plot(
    offset,
    steering_model(result.x, offset),
    "ro",
    label="Null angle point",
)
plt.xlabel("PWM")
plt.ylabel("Steering angle [rad]")
plt.grid(True)
plt.legend()

try:
    plt.show()
except KeyboardInterrupt:
    print("\nCalibration plot closed. Exiting cleanly...")
