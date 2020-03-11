# This file contains the functions that control the hardware of the robot
def drive_forward(roboclaw, address, speed, delay_time):
    # Move forward for delay_time seconds
    roboclaw.ForwardMixed(address, speed)
    time.sleep(delay_time)

def turn_right(robowclaw, address, speed, delay_time):
    # Turn right for delay_time seconds
    roboclaw.TurnRightMixed(address, speed)
    time.sleep(delay_time)


def turn_left(roboclaw, address, speed, delay_time):
    # Turn left for delay_time seconds
    roboclaw.TurnLeftMixed(address, speed)
    time.sleep(delay_time)

def drive_backward(roboclaw, address, speed, delay_time):
    # Drive backwards for delay_time seconds
    roboclaw.TurnLeftMixed(address, speed)
    time.sleep(delay_time)

def read_front_distance_sensor():
    # Get distance from front ToF sensor

def read_left_distance_sensor():
    # Get distance from left ToF sensor

def read_right_distance_sensor():
    # Get distance from right ToF sensor

def read_back_distance_sensor():
    # Get distance from back ToF sensor

def set_wheel_speed():
    # Increase speed of wheel

def calc_distance():
    # Interface with encoder to find distance traveled by robot

def search_for_handrail():
    # Drive in a circle with radius x
    # Stop every x seconds to take images
    # CameraController.get_image
    # If obstacle hit, return obstacle detected
    # Continually be updating position with SLAM

def go_to_handrail(current_position, handrail_position):
    # Do while robot_x != handrail_x
        # Move in x direction until robot_x = handrail_x
        # turn_left(90)
        # Take image
        # Update robot_x and handrail_x
    # Move in y-direction until robot_y - handrail_y = fixed_parameter

def move_toward_wall(current_position, wall_position):
    # Find turn angle to make robot and wall perpendicular
    # Turn x degrees

def adjust_wall_grip(drone_angle):
    # Tilt drone fan to proper angle
    # set_wheel_speed(desired_speed)
    # Stop drone fan
    # set_wheel_speed

def clean_handrail(roboclaw, handrailID, robotposition):
    # While (handrail_is_not_clean)
        # Lower cleaning mechanism
        # Initiate vibe motors
        # Wait x amount of time
        # Lift cleaning mechanism
        # If (handrail is long)
            # For (each section n>1)
                drive_backward(roboclaw, motor_address, speed, delay_time) # drive_backward
                turn_right(roboclaw, motor_address, speed, delay_time) # turn 90 degrees
                drive_forward(roboclaw, motor_address, speed, delay_time) # drive_forward
                turn_left(roboclaw, motor_address, speed, delay_time) # turn -90 degrees
                drive_forward(roboclaw, motor_address, speed, delay_time) #drive_forward
