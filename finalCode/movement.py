from kobukidriversample import Kobuki
import time as t
   
def move_forward(kobuki, speed):
    while True:
        # Move forward by setting equal left and right wheel velocities
        kobuki.move(speed, speed, 0)

def move_backward(kobuki, speed):
    # Rotate for the specified duration
    duration = 5
    start_time = t.time()
    while t.time() - start_time < duration:
        kobuki.move(-speed,-speed,0)
        t.sleep(0.1)
    stop(kobuki)

def stop(kobuki):
    # Stop the robot by setting both wheel velocities to 0
    kobuki.move(0, 0, 0)

def rotate_left_drive(kobuki,duration):
    # For left rotation, set the left velocity negative and right velocity positive
    speed_left = -160
    speed_right = 80
    
    # Rotate for the specified duration
    start_time = t.time()
    while t.time() - start_time < duration:
        kobuki.move(speed_left, speed_right, 1)
        t.sleep(0.1)
    stop(kobuki)

def rotate_right_drive(kobuki,duration):
    # For left rotation, set the left velocity negative and right velocity positive
    speed_left = 80
    speed_right = -160
    
    # Rotate for the specified duration
    start_time = t.time()
    while t.time() - start_time < duration:
        kobuki.move(speed_left, speed_right, 1)
        t.sleep(0.1)
    stop(kobuki)
