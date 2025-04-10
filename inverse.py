import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo
#range 68 to 300mm
#robotic arm to be placed 20mm above the ground if object height is 30mm (enf effector will always come -5mm below the ground surface)
import numpy as np
x031 = 140
y031 = 80
z031 = -5 # keep it to -5mm only
a1 = 101.103 #mm
a2 = 13.728 #mm
a3 = 110.000 #mm
a4 = 80.8 #mm
d = 175.225 #mm
z032 = z031+d
r1 = np.sqrt(x031**2 + y031**2)
print("r1 = ", r1)
T1 = (np.arctan(y031/x031)) #radians
T2 = 0.0 
T3 = 0.0
def theta(z032):
    r2 = np.sqrt((z032-a1)**2 + (r1-a2)**2)
    psi3 = (np.arccos((a3**2+a4**2-r2**2)/(2*a3*a4)))
    T3 = (np.pi - psi3)
    psi1 = (np.arccos((a3**2+r2**2-a4**2)/(2*a3*r2)))
    psi2 = (np.arctan((z032-a1)/(r1-a2)))
    T2 = (psi2 - psi1)
    return T2,T3,psi2,psi1
if r1 <= 190 and z032 > a1:
    T2, T3, psi2, psi1 = theta(z032)
    if T2 < 0.436332 :
        T2_new = psi2 + psi1
        T3 = T3*(-1)
        T4 = -(T2_new + T3 + (np.pi)/2)
        Theta1 = np.degrees(T1)
        Theta2 = np.degrees(T2_new)
        Theta3 = np.degrees(T3)
        Theta4 = np.degrees(T4)
        print("theta3 is : ", Theta3)
    else:
        T4 = -(T2 + T3 +(np.pi)/2)
        Theta1 = np.degrees(T1)
        Theta2 = np.degrees(T2)
        Theta3 = np.degrees(T3)
        Theta4 = np.degrees(T4)
if r1 > 190 :
    r1new = r1
    r1 = 190
    T = np.arcsin((r1new-r1)/d)
    z032 = z031 + d*np.cos(T)
    T2, T3, psi2, psi1 = theta(z032)
    if T2 < 0.436332 :
        T2_new = psi2 + psi1
        T3 = T3*(-1)
        T4 = -(T2_new + T3 + (np.pi)/2-T)
        Theta1 = np.degrees(T1)
        Theta2 = np.degrees(T2_new)
        Theta3 = np.degrees(T3)
        Theta4 = np.degrees(T4)
        T = np.degrees(T)
        print("theta2 in if is : ", Theta2)
    else :
      T4 = -((np.pi/2)-T+T2+T3)
      Theta1 = np.degrees(T1)
      Theta2 = np.degrees(T2)
      Theta3 = np.degrees(T3)
      Theta4 = np.degrees(T4)
      T = np.degrees(T)
# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize the six servos on their respective channels
servo_base = Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_shoulder = Servo(pca.channels[6], min_pulse=500, max_pulse=2500)
servo_wrist = Servo(pca.channels[2], min_pulse=500, max_pulse=2500)
servo_gripper_roll = Servo(pca.channels[8], min_pulse=500, max_pulse=2500)
servo_gripper_pitch = Servo(pca.channels[4], min_pulse=500, max_pulse=2500)
servo_gripper_open_close = Servo(pca.channels[5], min_pulse=500, max_pulse=2500)

# Function to smoothly move the servos
def move_servo_smooth(servo_motor, target_angle, step_delay=0.02):
    current_angle = servo_motor.angle
    if current_angle is None:
        current_angle = 90  # Default to 90 degrees if undefined

    step_size = 1 if target_angle > current_angle else -1
    
    while abs(target_angle - current_angle) > abs(step_size):
        current_angle += step_size
        servo_motor.angle = current_angle
        time.sleep(step_delay)
    
    servo_motor.angle = target_angle

# Function to move the arm to a specific position using smooth function
def move_arm_to_position(angles):
    move_servo_smooth(servo_base, angles[0])
    move_servo_smooth(servo_shoulder, angles[1])
    move_servo_smooth(servo_wrist, angles[2])
    move_servo_smooth(servo_gripper_roll, angles[3])
    move_servo_smooth(servo_gripper_pitch, angles[4])
    move_servo_smooth(servo_gripper_open_close, angles[5])
    time.sleep(1)  # Small delay after movement

# Define the angle arrays for different positions
# Angles array: [base, shoulder, wrist, roll, pitch, gripper]
theta1 = 32 + (Theta1)
theta2 = (90-Theta2)-6
theta3 = (140+Theta3)
theta4 = -Theta4-9
print(Theta2)
print(f"T1 = {Theta1}, T2 = {Theta2}, T3 = {Theta2+Theta3}, T4 = {Theta2+Theta3+Theta4}")
print(f"Theta1: {theta1}, Theta2: {theta2}, Theta3: {theta3}, Theta4: {theta4}")

home_position_angles = [32, 0, 140, 0, 0, 0]  # Adjust as needed for your home position
pickup_position_angles = [theta1, theta2, theta3, theta4, 0, 0]  # Adjust these angles for the object pickup
return_position_angles = [theta1, theta2, theta3, theta4, 0, 90]  # Return to the home position

# Function to perform the entire task sequence
def perform_pick_and_place():
    try:
        print("Moving to home position...")
        move_arm_to_position(home_position_angles)

        time.sleep(2)

        print("Moving to the pickup position...")
        move_arm_to_position(pickup_position_angles)

        # Close the gripper to pick up the object
        print("Closing gripper to pick up object...")
        move_servo_smooth(servo_gripper_open_close, 70)  # Adjust this angle to close the gripper properly

        time.sleep(2)

        print("Returning to home position with the object...")
        move_arm_to_position(return_position_angles)
    
    except KeyboardInterrupt:
        print("\nProcess interrupted by the user.")
        move_arm_to_position(home_position_angles)  # Return to home position if interrupted
        pca.deinit()  # Deinitialize PCA9685 on KeyboardInterrupt
    except Exception as e:
        print(f"An error occurred: {e}")
        move_arm_to_position(home_position_angles)  # Return to home position in case of an error
    finally:
        pca.deinit()  # Safely deinitialize the PCA9685 module

# Main execution
if __name__ == "__main__":
    perform_pick_and_place()
    print("Pick and place sequence complete.")
