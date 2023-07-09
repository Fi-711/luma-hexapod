#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .submodules.maestro import Controller
from .submodules.helper import servo_centre, linear_interpolation, bezier_interpolation, linear_interpolate_line, bezier_curve, sine_ease_in_out, euclidean_distance
import time 
import math


class HexapodMovement(Node):
    # Defaults
    SPEED = 100
    ACCELERATION = 30

    # Length of leg segments in millimeters
    COXA_LEN = 43
    FEMUR_LEN = 80
    TIBIA_LEN = 134

    # Correction angles
    OFFSET_COXA = -8
    OFFSET_FEMUR = -35.8
    OFFSET_TIBIA = 113.2
    COXA_OFFSET_ANGLES = offset_angles = [0, 45, 45, 0, -45, -45]        # leg_id 0-5 = middle right, rear right, rear left, middle left, front left, front right

    # legs
    FRONT_LEGS = [4, 5]
    MIDDLE_LEGS = [0, 3]
    BACK_LEGS = [1, 2]
    RIGHT_LEGS = [5, 0, 1]
    LEFT_LEGS = [2, 3, 4]
    RIGHT_TRIPOD_LEGS = [1, 3, 5]
    LEFT_TRIPOD_LEGS = [0, 2, 4]

    # start points
    TRIPOD_WIDE = [(198, -10, -70), (140, -140, -70), (140, -140, -70), (198, -10, -70), (140, 140, -70), (140, 140, -70)]
    TRIPOD_DEFAULT = [(148, -10, -70), (104, -104, -70), (104, -104, -70), (148, -10, -70), (104, 104, -70), (104, 104, -70)]
    SIDEWINDER = [(120, -6, -60)] * 6 # all legs start in the same position - 90 degrees to side


    # booleans
    must_stop = False
    is_standing = False
    is_walk_setup = False
    can_walk = False

    # coordinates look up table - initialize at start
    TRIPOD_BEZIER_COORDINATES = {
        "forward": {"forward_control_points": [], "backward_control_points": []},
        "back": {"forward_control_points": [], "backward_control_points": []},
        "rotate_left": {"forward_control_points": [], "backward_control_points": []},
        "rotate_right": {"forward_control_points": [], "backward_control_points": []}
    }

    TRIPOD_ROBOT_COORDINATES = {
        "forward": {"forward_control_points": [], "backward_control_points": []},
        "back": {"forward_control_points": [], "backward_control_points": []},
        "rotate_left": {"forward_control_points": [], "backward_control_points": []},
        "rotate_right": {"forward_control_points": [], "backward_control_points": []}
    }


    def __init__(self, device):
        """
        Initialization of Hexapod Movement
        """
        super().__init__('hexapod_movement')
        self.get_logger().info("\n*******************************\n\tHELLO FROM LUMA\n*******************************\n")
        self.hexapod_movement_publisher_ = self.create_publisher(String, '/hexapod_movement_topic', 10)
        self.hexapod_controller_subscriber_ = self.create_subscription(String, '/hexapod_controller_topic', self.hexapod_controller_callback, 10)
        self.hexapod_voice_commands_subscriber_ = self.create_subscription(String, '/hexapod_sound_voice_commands_topic', self.hexapod_voice_controller_callback, 10)
        # tmp = String()
        # tmp.data = "Test"
        # self.hexapod_movement_publisher_.publish(tmp)
        self.inputs_buffer = [None]

        # # Initialize the Maestro controller
        self.servo = Controller(device=device)

        # Servo pins and servo calibration valuesL [-45 usec] [+45 usec]
        self.servo_calibration = {
        "L11": [1904, 980],
        "L12": [1932, 996],
        "L13": [1992, 1052],
        "L21": [1936, 1000],
        "L22": [2012, 1088],
        "L23": [1966, 1010],
        "L31": [1908, 972],
        "L32": [1938, 1032],
        "L33": [1970, 1038],
        "R11": [2000, 1080],
        "R12": [2030, 1110],
        "R13": [2006, 1089],
        "R21": [1910, 980],
        "R22": [1931, 1021],
        "R23": [2016, 1102],
        "R31": [1938, 1002],
        "R32": [1955, 1071],
        "R33": [1924, 1000]
        }
        
        # Define the servo for each leg. For channels - 1st = coxa, 2nd = femur, 3rd = tibia in each list
        self.legs = [
            {
            "id": 0,
            "name": "middle_right",
            "start_pos": [servo_centre(self.servo_calibration["R21"]), servo_centre(self.servo_calibration["R22"]), servo_centre(self.servo_calibration["R23"])],
            "channels": [6, 7, 8],
            "servos": ["R21", "R22", "R23"]
            },
            {
            "id": 1,
            "name": "rear_right",
            "start_pos": [servo_centre(self.servo_calibration["R31"]), servo_centre(self.servo_calibration["R32"]), servo_centre(self.servo_calibration["R33"])],
            "channels": [0, 1, 2],
            "servos": ["R31", "R32", "R33"]
            },
            {
            "id": 2,
            "name": "rear_left",
            "start_pos": [servo_centre(self.servo_calibration["L31"]), servo_centre(self.servo_calibration["L32"]), servo_centre(self.servo_calibration["L33"])],
            "channels": [3, 4, 5],
            "servos": ["L31", "L32", "L33"]
            },
            {
            "id": 3,
            "name": "middle_left",
            "start_pos": [servo_centre(self.servo_calibration["L21"]), servo_centre(self.servo_calibration["L22"]), servo_centre(self.servo_calibration["L23"])],
            "channels": [9, 10, 11],
            "servos": ["L21", "L22", "L23"]
            },
            {
            "id": 4,
            "name": "front_left",
            "start_pos": [servo_centre(self.servo_calibration["L11"]), servo_centre(self.servo_calibration["L12"]), servo_centre(self.servo_calibration["L13"])],
            "channels": [15, 16, 17],
            "servos": ["L11", "L12", "L13"]
            },
            {
            "id": 5,
            "name": "front_right",
            "start_pos": [servo_centre(self.servo_calibration["R11"]), servo_centre(self.servo_calibration["R12"]), servo_centre(self.servo_calibration["R13"])],
            "channels": [12, 13, 14],
            "servos": ["R11", "R12", "R13"]
            }
        ]

        # initialize the coordinate lookup table
        self.set_tripod_robot_coordinates(stride=80, swing_angle=50)
        self.set_tripod_bezier_coordinates(stride=80, swing_angle=50)

        # default acceleration of 30 and speed 80 for all legs - good balance between speed and accuracy
        for leg in range(6):
            self.set_speed(leg, speed=self.SPEED, acceleration=self.ACCELERATION)

        # Create a timer to periodically perform the gait
        # self.timer = self.create_timer(1, self.sidewinder_bezier_walk)
        # self.timer = self.create_timer(1, self.tripod_bezier_walk)
        # self.sit()
        # self.stand()
        # time.sleep(1)
        # self.timer = self.create_timer(1, self.tripod_robot_walk)
        # self.stop()
        # self.neutral_standing_position(0.1)
        # for _ in range(18):
            # self.tripod_robot_walk(direction="rotate_right")
            # self.tripod_bezier_walk(direction="rotate_right")
        # time.sleep(0.5)
        # self.stop()

    def hexapod_voice_controller_callback(self, msg):
        # Process the received keyboard input
        input_str = msg.data
  
        match input_str:
            case'forward': # up
                # self.tripod_robot_walk()
                self.tripod_bezier_walk(direction="forward")
            case 'back': # down
                self.tripod_bezier_walk(direction="back")
            case 'rotate_right': # right  
                self.tripod_bezier_walk(direction="rotate_right")
            case 'rotate_left': # left
                self.tripod_bezier_walk(direction="rotate_left")
            case 'sit': # 1
                self.sit()
            case 'stand':  # 2
                self.stand()
            case 'neutral_standing':   #3
                self.neutral_standing_position()
            case 'stop':  # 0
                self.stop()
        self.get_logger().info(input_str)


    def hexapod_controller_callback(self, msg):
        # Process the received keyboard input
        input_str = msg.data
  
        match input_str:
            case'forward': # up
                # self.tripod_robot_walk()
                self.tripod_bezier_walk(direction="forward")
            case 'back': # down
                self.tripod_bezier_walk(direction="back")
            case 'rotate_right': # right  
                self.tripod_bezier_walk(direction="rotate_right")
            case 'rotate_left': # left
                self.tripod_bezier_walk(direction="rotate_left")
            case 'sit': # 1
                self.sit()
            case 'stand':  # 2
                self.stand()
            case 'neutral_standing':   #3
                self.neutral_standing_position()
            case 'stop':  # 0
                self.stop()
        self.get_logger().info(input_str)



    def set_speed(self, leg_id, speed=None, acceleration=None):
        """
        Set speed and acceleration
        """
        leg_channels = self.legs[leg_id]['channels']

        for i in leg_channels:
            if speed is not None:
                self.servo.setSpeed(i, speed)
            if acceleration is not None:
                self.servo.setAccel(i, acceleration)

    """
    =================
    Transitions and miscellaneous
    =================
    """ 

    def transition(self, wait=1):
        """
        Transition state - legs up so can move to another state without getting "stuck".
        """
        # Coxas centred, femurs retracted and tibias raised - can safley move to other positions from this
        for leg in range(6):
            # middle legs
            if leg in [0, 3]:
                self.move_leg_to([150, 0, 50], leg_id=leg)
            # back legs
            elif leg in [1, 2]:
                self.move_leg_to([106, -106, 50], leg_id=leg)
            # # front legs
            elif leg in [4, 5]:
                self.move_leg_to([106, 106, 50], leg_id=leg)

        # wait before performing next task
        time.sleep(wait)

        # Publish a message to indicate that the gait has been performed
        msg = String()
        msg.data = 'Hexapod transitioning ...'
        self.hexapod_movement_publisher_.publish(msg)    


    def stand(self, wide_stance=False):
        """
        Default stand is with width of 105 mm. Wide stance has all servos in neutral position (148mm width stance). This is at x =  148, y = 0 and z = -70
        """
        # transition before standing
        self.transition()

        # Stand up so all servos centred
        if wide_stance:
            for leg in range(6):
                # middle legs
                if leg in [0, 3]:
                    self.move_leg_to([148, 0, -70], leg_id=leg)
                # back legs
                elif leg in [1, 2]:
                    self.move_leg_to([104, -104, -70], leg_id=leg)
                # front legs
                elif leg in [4, 5]:
                    self.move_leg_to([104, 104, -70], leg_id=leg)
        else:
            for leg in range(6):
                # middle legs
                if leg in [0, 3]:
                    self.move_leg_to([105, 0, -50], leg_id=leg)
                # back legs
                elif leg in [1, 2]:
                    self.move_leg_to([74, -74, -50], leg_id=leg)
                # front legs
                elif leg in [4, 5]:
                    self.move_leg_to([74, 74, -50], leg_id=leg)

        # Publish a message to indicate that the gait has been performed
        msg = String()
        msg.data = 'Hexapod stood up successfully'
        self.hexapod_movement_publisher_.publish(msg)
        self.is_standing = True
        self.is_walk_setup = False
        time.sleep(1)


    def sit(self):
        """
        Sits hexapod down
        """
        # transition before standing
        self.transition()

        # Sit down so coxa servos are parallel
        for i in range(6):
            self.move_leg_to([65, 0, -50], leg_id=i)

        # Publish a message to indicate that the gait has been performed
        msg = String()
        msg.data = 'Hexapod sat down successfully'
        self.hexapod_movement_publisher_.publish(msg)
        self.is_standing = False
        self.is_walk_setup = False
        time.sleep(1)


    def stop(self):
        """
        Hexapod stops at current position - same x and y, z set to leg with lowest z value
        """
        self.can_walk = False
        z_values = []
        positions = []
        for leg in range(6):
            result = self.get_leg_position(leg)
            z_values.append(result[2])
            positions.append(result)

        z_min = min(z_values)

        # Same x and y but place all feet on ground (lowest z value)
        for leg in range(6):
            pos = list(positions[leg])
            pos[2] = z_min
            # print(f"Stop position: {pos}")
            self.move_leg_to(pos, leg_id=leg)

        # Publish a message to indicate that the gait has been performed
        msg = String()
        msg.data = 'Hexapod stopped and planted feet'
        self.hexapod_movement_publisher_.publish(msg)
        self.is_standing = False
        time.sleep(1)


    def walk_setup(self, coordinates, transition_time=0.6):
        """
        Move each leg to its starting walking position
        """
        transition_pos = [[150, 0, 50], [106, -106, 50], [106, -106, 50], [150, 0, 50], [106, 106, 50], [106, 106, 50]]
        # raise leg and move to desired coordinates
        for leg in self.RIGHT_TRIPOD_LEGS:
            self.move_leg_to(transition_pos[leg], leg_id=leg)
        
        time.sleep(transition_time)
        
        for leg in self.RIGHT_TRIPOD_LEGS:     
            self.move_leg_to(coordinates[leg], leg_id=leg)
        
        time.sleep(transition_time)
        
        for leg in self.LEFT_TRIPOD_LEGS:
            self.move_leg_to(transition_pos[leg], leg_id=leg)
        
        time.sleep(transition_time)
        
        for leg in self.LEFT_TRIPOD_LEGS:     
            self.move_leg_to(coordinates[leg], leg_id=leg)
        
        self.is_walk_setup = True


    def neutral_standing_position(self, transition_time=0.5):
        """
        Move each leg to the default standing poisiton - raises one leg at a time to get there
        """
        # raise leg and move to desired coordinates
        for leg in range(6):
            # middle legs
            if leg in [0, 3]:
                self.move_leg_to([150, 0, 50], leg_id=leg)
                time.sleep(transition_time)
                self.move_leg_to([148, 0, -70], leg_id=leg)
            # back legs
            elif leg in [1, 2]:
                self.move_leg_to([106, -106, 50], leg_id=leg)
                time.sleep(transition_time)
                self.move_leg_to([104, -104, -70], leg_id=leg)
            # # front legs
            elif leg in [4, 5]:
                self.move_leg_to([106, 106, 50], leg_id=leg)
                time.sleep(transition_time)
                self.move_leg_to([104, 104, -70], leg_id=leg)
        time.sleep(transition_time)
        self.is_walk_setup = True


    """
    =================
    Gaits
    =================
    """
    def sidewinder_bezier_walk(self, duration=1, stride=30, swing_angle=60, speed=80, acceleration=40, forward=True):
        """
        Sidewinder gait using bezier system
        """
        forward_control_points = []
        backward_control_points = []

        # +150 and +40 are offsets because when interpolating the bezier curve, it folds in on itself in 3d
        for leg in range(6):
            # tmp = self.calculate_step_points(leg, stride, swing_angle, self.SIDEWINDER[leg], sidewinder=True)
            # offset = (tmp[1][0] + 150, tmp[1][1]+40, tmp[1][2])
            # points = [tmp[0], offset, tmp[2], tmp[3]]
            # forward_control_points.append(points[:3])
            # backward_control_points.append([points[-2], points[-1]])
            points = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.SIDEWINDER[leg], forward=forward, bezier=True, type="SIDEWINDER")
            forward_control_points.append(points)
            backward_control_points.append([points[-1], points[0]])

        # sidewinder walks at just 3cm/s!
        for leg in range(6):
            self.set_speed(leg, speed, acceleration)

        self.bezier_tripod_system(forward_control_points, backward_control_points, duration)

    
    def tripod_bezier_walk(self, direction="forward", duration=1, stride=80, swing_angle=60, speed=100, acceleration=40):
        """
        Tripod gait using bezier system.
        """
        # tripod walks at max 85 cm/stride. Any more and legs will collide
        for leg in range(6):
            self.set_speed(leg, speed, acceleration)

        match direction:
            case "forward":
                self.bezier_tripod_system(self.TRIPOD_BEZIER_COORDINATES["forward"]["forward_control_points"], self.TRIPOD_BEZIER_COORDINATES["forward"]["backward_control_points"], duration=duration)
            case "back":
                self.bezier_tripod_system(self.TRIPOD_BEZIER_COORDINATES["back"]["forward_control_points"], self.TRIPOD_BEZIER_COORDINATES["back"]["backward_control_points"], duration=duration)
            case "rotate_left":
                self.bezier_tripod_system(self.TRIPOD_BEZIER_COORDINATES["rotate_left"]["forward_control_points"], self.TRIPOD_BEZIER_COORDINATES["rotate_left"]["backward_control_points"], duration=duration)
            case "rotate_right":
                self.bezier_tripod_system(self.TRIPOD_BEZIER_COORDINATES["rotate_right"]["forward_control_points"], self.TRIPOD_BEZIER_COORDINATES["rotate_right"]["backward_control_points"], duration=duration)

        
        # Publish a message to indicate that the gait has been performed
        msg = String()
        msg.data = 'bezier_tripod'
        self.hexapod_movement_publisher_.publish(msg)
        # self.is_standing = False
        # self.is_walk_setup = False
        # time.sleep(1)
    
    def tripod_robot_walk(self, direction="forward", speed=80, acceleration=20, duration=0.5):
        """
        Tripod walk using robot system.
        """
        # fastest duration = 0.32

        match direction:
            case "forward":
                self.robot_tripod_system(self.TRIPOD_ROBOT_COORDINATES["forward"]["forward_control_points"], self.TRIPOD_ROBOT_COORDINATES["forward"]["backward_control_points"], speed=speed, acceleration=acceleration, duration=duration)
            case "back":
                self.robot_tripod_system(self.TRIPOD_ROBOT_COORDINATES["back"]["forward_control_points"], self.TRIPOD_ROBOT_COORDINATES["back"]["backward_control_points"], speed=speed, acceleration=acceleration, duration=duration)
            case "rotate_left":
                self.robot_tripod_system(self.TRIPOD_ROBOT_COORDINATES["rotate_left"]["forward_control_points"], self.TRIPOD_ROBOT_COORDINATES["rotate_left"]["backward_control_points"], speed=speed, acceleration=acceleration, duration=duration)
            case "rotate_right":
                self.robot_tripod_system(self.TRIPOD_ROBOT_COORDINATES["rotate_right"]["forward_control_points"], self.TRIPOD_ROBOT_COORDINATES["rotate_right"]["backward_control_points"], speed=speed, acceleration=acceleration, duration=duration)


    """
    =================
    Walk Systems - works with tripod gaits only
    =================
    """

    def bezier_tripod_system(self, right_tripod_control_points, left_tripod_control_points, duration=0.5):
        """
        Bezier walk cycle with interpolation for tripod gaits i.e. move hree legs at a time
        """
        # Define the tripod control points
        # sidewinder forward_control_points = [(120, -6, -60), (250, 10, 10), (120, 25, -60)]
        # sidewinder backward_control_points = [(120, 25, -60), (120, -6, -60)]
        # tripod forward_control_points = [(148, -10, -70), (200, 35, -10), (148, 60, -70)]
        # tripod backward_control_points = [(148, 60, -70), (148, -10, -70)]

        # setup legs if not in walk stance
        if not self.is_walk_setup:
            self.stand()
            coordinates = []
            for leg in range(6):
                if leg in self.RIGHT_TRIPOD_LEGS:
                    coordinates.append(right_tripod_control_points[leg][0])
                else:
                    coordinates.append(left_tripod_control_points[leg][0])
            self.walk_setup(coordinates)

        # for _ in range(5):
        self.three_step_pair(duration, self.RIGHT_TRIPOD_LEGS, self.LEFT_TRIPOD_LEGS, right_tripod_control_points, left_tripod_control_points)
        self.three_step_pair(duration, self.LEFT_TRIPOD_LEGS, self.RIGHT_TRIPOD_LEGS, right_tripod_control_points, left_tripod_control_points)
        # self.three_step_pair(duration, [], [5], forward_control_points, backward_control_points, 0.1)
        # self.three_step_pair(duration, [5], [], forward_control_points, backward_control_points, 0.1)

        # Publish a message to indicate that the gait has been performed
        msg = String()
        msg.data = 'Hexapod walking with bezier tripod system.'
        self.hexapod_movement_publisher_.publish(msg)


    def robot_tripod_system(self, right_tripod_control_points, left_tripod_control_points, speed=80, acceleration=20, duration=0.5):
        """
        Robot walk cycle with no interpolation for tripod gaits i.e. move three legs at a time
        """
        # move leg faster on forward step
        speed_scale_factor = euclidean_distance(right_tripod_control_points[0][0], right_tripod_control_points[0][1]) * 2 / euclidean_distance(right_tripod_control_points[0][2], right_tripod_control_points[0][3]) * 1.1
        for i in range(6):
            self.set_speed(i, acceleration=acceleration)

        # setup legs into right positions
        if not self.is_walk_setup:
            self.stand()
            coordinates = []
            for leg in range(6):
                if leg in self.RIGHT_TRIPOD_LEGS:
                    # print(forward_control_points)
                    coordinates.append(right_tripod_control_points[(leg-1)//2][0])
                else:
                    coordinates.append(left_tripod_control_points[leg//2][0])
            self.walk_setup(coordinates)

        # start_time = time.time()
        # loop though all positions
        # print(len(right_tripod_control_points[0]), len(left_tripod_control_points[0]))
        for pos in range(len(right_tripod_control_points[0])+1):
            pos = pos % len(right_tripod_control_points[0])
            # right tripod legs
            for leg in self.RIGHT_TRIPOD_LEGS:
                # adjust speed based on control point
                if pos == 3:
                    self.set_speed(leg_id=leg, speed=speed)
                else:
                    self.set_speed(leg_id=leg, speed=int(speed * speed_scale_factor))

                self.move_leg_to(right_tripod_control_points[(leg-1)//2][pos], leg)

            # Update outside the loop!
            # pos = (pos + 2) % 4         
            # print(f"Right: {right_tripod_control_points}, pos: {pos}")
            # left tripod legs
            for leg in self.LEFT_TRIPOD_LEGS:
                # adjust speed based on control point
                if pos == 0:
                    self.set_speed(leg_id=leg, speed=speed)
                else:
                    self.set_speed(leg_id=leg, speed=int(speed * speed_scale_factor))
                self.move_leg_to(left_tripod_control_points[leg//2][pos], leg)
            
            # print(f"Left: {left_tripod_control_points}, pos: {pos}")
            time.sleep(duration)           

        # print(f"Step time: {time.time() - start_time}")

        # Publish a message to indicate that the gait has been performed
        msg = String()
        msg.data = 'Hexapod walking with robot tripod system'
        self.hexapod_movement_publisher_.publish(msg)  


    def set_tripod_robot_coordinates(self, stride=80, swing_angle=50):
        """
        Sets the coordinates in lookup table for robot system
        """

        # forward
        for leg in range(6):
            coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=False, type="TRIPOD")
            if leg in self.RIGHT_TRIPOD_LEGS:
                self.TRIPOD_ROBOT_COORDINATES["forward"]["forward_control_points"].append(coordinates)
            else:
                tmp = coordinates[2:] + coordinates[:2]
                self.TRIPOD_ROBOT_COORDINATES["forward"]["backward_control_points"].append(tmp)
    

        # back
        for leg in range(6):
            coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=False, type="TRIPOD")
            if leg in self.RIGHT_TRIPOD_LEGS:
                self.TRIPOD_ROBOT_COORDINATES["back"]["forward_control_points"].append(coordinates)
            else:
                tmp = coordinates[2:] + coordinates[:2]
                self.TRIPOD_ROBOT_COORDINATES["back"]["backward_control_points"].append(tmp)

        # rotate left
        for leg in range(6):
            if leg in self.RIGHT_TRIPOD_LEGS:
                if leg in self.MIDDLE_LEGS:
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=False, type="TRIPOD")     
                else:
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=False, type="TRIPOD")
                self.TRIPOD_ROBOT_COORDINATES["rotate_left"]["forward_control_points"].append(coordinates)    
            else:
                if leg in self.MIDDLE_LEGS:
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=False, type="TRIPOD")
                    tmp = coordinates[2:] + coordinates[:2]
                else: 
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=False, type="TRIPOD")
                    tmp = coordinates[2:] + coordinates[:2]
                self.TRIPOD_ROBOT_COORDINATES["rotate_left"]["backward_control_points"].append(tmp)

        # rotate right
        for leg in range(6):
            if leg in self.RIGHT_TRIPOD_LEGS:
                if leg in self.MIDDLE_LEGS:
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=False, type="TRIPOD")
                else:
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=False, type="TRIPOD")
                self.TRIPOD_ROBOT_COORDINATES["rotate_right"]["forward_control_points"].append(coordinates)
            else:
                if leg in self.MIDDLE_LEGS:
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=False, type="TRIPOD")
                    tmp = coordinates[2:] + coordinates[:2]
                else: 
                    coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=False, type="TRIPOD")
                    tmp = coordinates[2:] + coordinates[:2]
                self.TRIPOD_ROBOT_COORDINATES["rotate_right"]["backward_control_points"].append(tmp)  

    
    def set_tripod_bezier_coordinates(self, stride=80, swing_angle=50):
        """
        Sets the coordinates in lookup table for robot system
        """

        # forward
        for leg in range(6):
            coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=True, type="TRIPOD")
            self.TRIPOD_BEZIER_COORDINATES["forward"]["forward_control_points"].append(coordinates)
            self.TRIPOD_BEZIER_COORDINATES["forward"]["backward_control_points"].append([coordinates[-1], coordinates[0]])

        # back
        for leg in range(6):
            coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=True, type="TRIPOD")
            self.TRIPOD_BEZIER_COORDINATES["back"]["forward_control_points"].append(coordinates)
            self.TRIPOD_BEZIER_COORDINATES["back"]["backward_control_points"].append([coordinates[-1], coordinates[0]])

        # rotate left
        for leg in range(6):
            if leg in self.RIGHT_LEGS:
                coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=True, type="TRIPOD")
            else: 
                coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=True, type="TRIPOD")
            self.TRIPOD_BEZIER_COORDINATES["rotate_left"]["forward_control_points"].append(coordinates)
            self.TRIPOD_BEZIER_COORDINATES["rotate_left"]["backward_control_points"].append([coordinates[-1], coordinates[0]])

        # rotate right
        for leg in range(6):
            if leg in self.LEFT_LEGS:
                coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=True, bezier=True, type="TRIPOD")
            else: 
                coordinates = self.calculate_control_points(leg=leg, stride=stride, swing_angle=swing_angle, point=self.TRIPOD_DEFAULT[leg], forward=False, bezier=True, type="TRIPOD")
            self.TRIPOD_BEZIER_COORDINATES["rotate_right"]["forward_control_points"].append(coordinates)
            self.TRIPOD_BEZIER_COORDINATES["rotate_right"]["backward_control_points"].append([coordinates[-1], coordinates[0]])


    """
    =================
    IK, Leg movements and Step Patterns
    =================
    """

    def inverse_kinematics(self, x, y, z, leg_id):
        """
        Return the IK of the three servos in quarter microseconds.
        """

        def offset_coxa(x, y):
            """
            Helper funtion to adjust for coxa offset
            """
            theta = math.atan2(y, x)
            x -= self.COXA_LEN * math.cos(theta)
            y -= self.COXA_LEN * math.sin(theta)

            return x, y

        # Calculate coxa angle before offsetting
        coxa_angle = math.degrees(math.atan2(y, x)) if x != 0 else 90
        coxa_angle += self.COXA_OFFSET_ANGLES[leg_id]

        if coxa_angle > 180:
            coxa_angle -= 360
    
        # femur and tibia calculations use an offset
        x, y = offset_coxa(x, y)
        
        # set up the equations
        P = math.sqrt(x**2 + y**2)          # x position in the x-z plane
        L = math.sqrt(P**2 + z**2)          # femur-target position distance in x-z plane

        if self.FEMUR_LEN + L < self.TIBIA_LEN or self.TIBIA_LEN + self.FEMUR_LEN < L: 
            return None

        # intermediary angles
        alpha = math.degrees(math.atan2(z, P))
        beta = math.degrees(math.acos((self.FEMUR_LEN**2 + L**2 - self.TIBIA_LEN**2)/(2 * self.FEMUR_LEN * L)))
        gamma = math.degrees(math.acos((self.FEMUR_LEN**2 + self.TIBIA_LEN**2 - L**2)/(2 * self.FEMUR_LEN * self.TIBIA_LEN)))
        # print(f"alpha={alpha}, beta={beta}, gamma={gamma}")
        
        # final angles
        femur_angle = beta + alpha + self.OFFSET_COXA           # alpha is negative when z < 0 and positive when z > 0, The femur is 8 degrees above coxa so acount for this angle
        tibia_angle = -(180 - gamma)                            # -ve because the rotation is clockwise
        # print(f"coxa: {coxa_angle}, femur: {femur_angle}, tibia: {tibia_angle}")
        
        # offset angles so servos point towards +ve x axis
        femur_angle += (self.OFFSET_FEMUR)
        tibia_angle += (self.OFFSET_TIBIA)
        # print(femur_angle, tibia_angle)
        
        # convert angles to quarter microseconds
        converted_coxa_angle = -coxa_angle * 4 * (self.servo_calibration[self.legs[leg_id]['servos'][0]][0] - self.servo_calibration[self.legs[leg_id]['servos'][0]][1]) / 90
        
        # because servos are on the 'same' side of the legs (how the hexapod was designed), they need to be flipped for the other side in the code
        if leg_id in self.LEFT_LEGS:
            # LEFT SIDE
            converted_femur_angle = -femur_angle * 4 * (self.servo_calibration[self.legs[leg_id]['servos'][1]][0] - self.servo_calibration[self.legs[leg_id]['servos'][1]][1]) / 90
            converted_tibia_angle = tibia_angle * 4 * (self.servo_calibration[self.legs[leg_id]['servos'][2]][0] - self.servo_calibration[self.legs[leg_id]['servos'][2]][1]) / 90
        else:
            # RIGHT SIDE
            converted_femur_angle = femur_angle * 4 * (self.servo_calibration[self.legs[leg_id]['servos'][1]][0] - self.servo_calibration[self.legs[leg_id]['servos'][1]][1]) / 90
            converted_tibia_angle = -tibia_angle * 4 * (self.servo_calibration[self.legs[leg_id]['servos'][2]][0] - self.servo_calibration[self.legs[leg_id]['servos'][2]][1]) / 90


        move_servos = [converted_coxa_angle, converted_femur_angle, converted_tibia_angle]
        # print(f"Move amount: {move_servos}")
        # limit servo to ~90 degrees either side
        for servo in move_servos:
            if servo > 3600 or servo < -3600:
                return None

        return move_servos


    def move_leg_to(self, coordinates, leg_id):
        """
        Moves a leg to (x, y, z) position if possible.
        """
        # get move position
        move_amount = self.inverse_kinematics(coordinates[0], coordinates[1], coordinates[2], leg_id)        
        
        # will hold final servo coordinates
        move_to = [None, None, None]
        
        if move_amount is not None:
            for i in range(3):
                # flip coxa movement for left side
                if leg_id in [2, 3, 4] and i == 0:
                    move_to[i] = self.legs[leg_id]['start_pos'][i] - move_amount[i]              
                else:
                    move_to[i] = self.legs[leg_id]['start_pos'][i] + move_amount[i]

            for i, channel in enumerate(self.legs[leg_id]['channels']):
                # self.get_logger().info(f"{channel}: {move_to[i]/4}")
                self.servo.setTarget(channel, int(move_to[i]))
        
        else:
            self.get_logger().error(f"The {self.legs[leg_id]['name']} leg cannot reach ({coordinates[0]}, {coordinates[1]}, {coordinates[2]})!")


    # step patterns
    def three_step_pair(self, duration, forward_legs, backward_legs, control_points_1, control_points_2, delay_between_steps = 0.1):
        """
        Moves a both sets of three legs either backwards or forwards to the desired control points
        """
        # start time
        start_time = time.time()
        elapsed_time = 0        
        
        while elapsed_time <= duration:
            # for i in range(len(forward_arc_points)):
            elapsed_time = time.time() - start_time

            # move to end of cycle if time over and break loop
            if elapsed_time >= duration:
                for leg in forward_legs:
                    if len(forward_legs) > 0:
                        self.move_leg_to(control_points_1[leg][-1], leg)
                for leg in backward_legs:
                    if len(backward_legs) > 0:
                        self.move_leg_to(control_points_2[leg][-1], leg)
                break                

            t = sine_ease_in_out(elapsed_time / duration)

            for leg in forward_legs:
                if len(forward_legs) > 0:
                    forward_pos = bezier_interpolation(control_points_1[leg], t)           # bezier interpolation for forward step
                    self.move_leg_to(forward_pos, leg)
            for leg in backward_legs:
                if len(backward_legs) > 0:
                    backward_pos = linear_interpolation(control_points_2[leg], t)         # linear interpolation for backward step                 
                    self.move_leg_to(backward_pos, leg)
            
            time.sleep(0.01)

        time.sleep(delay_between_steps)


    def get_leg_position(self, leg_id):
        """
        Gets the (x, y, z) coordinates for the desired leg
        """
        leg = self.legs[leg_id]
        channels = leg['channels']
        coxa_channel, femur_channel, tibia_channel = channels
        coxa_servo, femur_servo, tibia_servo = leg['servos']                       
        
        # degree modifier and centers for each servo         
        coxa_mod = (self.servo_calibration[coxa_servo][0] - self.servo_calibration[coxa_servo][1]) / 90
        femur_mod = (self.servo_calibration[femur_servo][0] - self.servo_calibration[femur_servo][1]) / 90
        tibia_mod = (self.servo_calibration[tibia_servo][0] - self.servo_calibration[tibia_servo][1]) / 90
        coxa_centre = servo_centre(self.servo_calibration[coxa_servo])
        femur_centre = servo_centre(self.servo_calibration[femur_servo])
        tibia_centre = servo_centre(self.servo_calibration[tibia_servo])

        coxa_angle = -(self.servo.getPosition(coxa_channel) - coxa_centre) / (coxa_mod * 4)

        if leg_id in [2, 3, 4]:
            femur_angle = -(self.servo.getPosition(femur_channel) - femur_centre) / (femur_mod * 4) - (self.OFFSET_FEMUR + self.OFFSET_COXA)
            tibia_angle = (self.servo.getPosition(tibia_channel) - tibia_centre) / (tibia_mod * 4) - self.OFFSET_TIBIA
            coxa_angle = -coxa_angle
        else:
            femur_angle = (self.servo.getPosition(femur_channel) - femur_centre) / (femur_mod * 4) - (self.OFFSET_FEMUR + self.OFFSET_COXA)
            tibia_angle = -(self.servo.getPosition(tibia_channel) - tibia_centre) / (tibia_mod * 4) - self.OFFSET_TIBIA
        
        # side angles and lengths calculations
        gamma = 180 + tibia_angle
        L = math.sqrt(self.FEMUR_LEN**2 + self.TIBIA_LEN**2 - 2 * self.FEMUR_LEN * self.TIBIA_LEN * math.cos(math.radians(gamma)))
        beta = math.degrees(math.acos((self.FEMUR_LEN**2 + L**2 - self.TIBIA_LEN**2)/(2 * self.FEMUR_LEN * L)))
        alpha = femur_angle - beta
        z = math.sin(math.radians(alpha)) * L
        P = math.sqrt(L**2 - z**2)

        # top calculations 
        x = math.cos(math.radians(coxa_angle - self.COXA_OFFSET_ANGLES[leg_id])) * (P + self.COXA_LEN)
        y = math.sin(math.radians(coxa_angle - self.COXA_OFFSET_ANGLES[leg_id])) * (P + self.COXA_LEN)

        # print(x, y, z)
        return x, y, z


    def calculate_control_points(self, leg, stride, swing_angle, point, forward=True, bezier=True, type="TRIPOD", bezier_x_offset=100, bezier_z_offset=40):
        """
        Calculates the control points for a single leg. Forward_point parameter defines if it is the forward arc or backward drag. Bezier => no start point at end, robotic system stays at start point for two motions.
        """

        # Error check that correct parameters have been passed
        if (swing_angle > 60 or swing_angle < 20):
            self.get_logger().error("Swing angle must be between 20 and 60 degrees")
            return None
        if (stride < 0 or stride > 100):
            self.get_logger().error("Maximum stride length is 100mm")
            return None

        def calculate_mid_point(start_point, end_point):
            x = (start_point[0] + end_point[0]) / 2
            y = (start_point[1] + end_point[1]) / 2
            mid_point_distance = math.sqrt((start_point[0] - x)**2 + (start_point[1] - y)**2)
            z = abs(mid_point_distance) * math.tan(math.radians(swing_angle)) + start_point[2]

            # to prevent curve folding in on itself
            if bezier:
                x += bezier_x_offset
                z += bezier_z_offset

            return x, y, z

        # start or end point
        match leg:
            case 0 | 3: # middle legs
                if type.upper() == "TRIPOD" or type.upper() == "SIDEWINDER":
                    start_point = point if forward else (point[0], -point[1], point[2])
                    end_point = (point[0], point[1] + stride, point[2]) if forward else (point[0], -point[1] - stride, point[2])
            case 4 | 5: # front legs
                if type.upper() == "TRIPOD":
                    start_point= (point[0], point[1] - stride, point[2]) if forward else point                                    
                    end_point = point if forward else (point[0], point[1] - stride, point[2])
                elif type.upper() == "SIDEWINDER":
                    start_point = point if forward else (point[0], -point[1], point[2])
                    end_point = (point[0], point[1] + stride, point[2]) if forward else (point[0], -point[1] - stride, point[2])
            case 1 | 2: # back legs
                if type.upper() == "TRIPOD" or type.upper() == "SIDEWINDER":
                    start_point = point if forward else (point[0], point[1] + stride, point[2])
                    end_point = (point[0], point[1] + stride, point[2]) if forward else point
                elif type.upper() == "SIDEWINDER":
                    start_point = point if forward else (point[0], -point[1], point[2])
                    end_point = (point[0], point[1] + stride, point[2]) if forward else (point[0], -point[1] - stride, point[2])

        # mid point
        mid_point = calculate_mid_point(start_point, end_point)


        # bezier system does not need duplicate of start point
        return [start_point, mid_point, end_point] if bezier else [start_point, mid_point, end_point, start_point]


    """
    =================
    Clean Up Code
    =================
    """
    def __del__(self):
        # Close the connection to the Maestro controller
        self.servo.close()


    """
    =================
    Legacy Code - Functions that I don't use any more
    =================
    """

    # def step(self, legs, control_points_1, control_points_2, duration=1, speed=None, acceleration=None):
    #     """
    #     Sets step for tripod gait. No time based functions. Let servos move along path and have a delay so both forward and backward motion take exactly the set duration.
    #     """
    #     if len(control_points_1) > len(control_points_2):
    #         s1, a1, s2, a2, d1, d2, d3, d4 = [80, 40, 40, 20, 0.01, 0.02, 0.1, 0]
    #     else:
    #         s1, a1, s2, a2, d1, d2, d3, d4 = [40, 20, 80, 40, 0.02, 0.01, 0, 0.1]

    #     # self.set_speed(s1, a1)
    #     start_time = time.time()
        
    #     # forward step
    #     for point in control_points_1:
    #         for leg in legs:
    #             self.move_leg_to(point, leg)
    #             # print(self.get_leg_position(leg))
    #         time.sleep(d1)
    #     else:
    #         time.sleep(0.1)
    #         self.move_leg_to(control_points_1[-1], leg)
    #         time.sleep(d3)

    #     # print(f"Forward Step: {time.time() - start_time}")

    #     # backward step
    #     # self.set_speed(s2, a2)
    #     for point in control_points_2:
    #         for leg in legs:
    #             self.move_leg_to(point, leg)
    #         time.sleep(d2)
    #     else:
    #         time.sleep(0.1)
    #         self.move_leg_to(control_points_2[-1], leg)
    #         time.sleep(d4)               

    #     sleep_time = duration - (time.time() - start_time)
        
    #     # if sleep_time > 0:
    #     #     time.sleep(sleep_time)

    #     print(f"Total Step: {time.time() - start_time}")


    # def tripod_gait_old(self, duration=1, direction="F"):
    #     """
    #     Tripod walking gait with legs [5, 3, 1] as one set and [4, 2, 0] as the other. Normal version.
    #     """
    #     # match direction:
    #     #     case "F":
    #     #         stride = 

    #     # Define the tripod control points
    #     middle_control_points_front = bezier_curve([(150, -10, -70), (150, 10, 0), (150, 30, 0), (150, 50, -70)], 40)
    #     middle_control_points_back = linear_interpolate_line((150, 50, -70), (150, -6, -70), 10)

    #     front_control_points_front = bezier_curve([(130, 44, -70), (120, 64, 0), (110, 84, 0), (104, 104, -70)], 46)
    #     front_control_points_back = linear_interpolate_line((104, 104, -70), (130, 44, -70), 10)

    #     rear_control_points_front = bezier_curve([(104, -104, -70), (110, -84, 0), (120, -64, 0), (130, -44, -70)], 46)
    #     rear_control_points_back = linear_interpolate_line((130, -44, -70), (104, -104, -70), 10)


    #     p1 = multiprocessing.Process(target=self.step, args=([3], middle_control_points_front, middle_control_points_back))
    #     p2 = multiprocessing.Process(target=self.step, args=([0], middle_control_points_back, middle_control_points_front))

    #     p1.start()
    #     p2.start()

    #     p1.join()
    #     p2.join()

    #     # self.step([3], middle_control_points_front, middle_control_points_back)
    #     # self.step([0], middle_control_points_back, middle_control_points_front)
    #     # self.step([4], front_control_points_front, front_control_points_back, speed=80, acceleration=40)
    #     # self.step([1], rear_control_points_front, rear_control_points_back, speed=80, acceleration=40)

    #     # self.step(self.MIDDLE_LEGS, middle_control_points_front, middle_control_points_back, speed=80, acceleration=40)
    #     # self.step(self.FRONT_LEGS, front_control_points_front, front_control_points_back, speed=80, acceleration=40)
    #     # self.step(self.BACK_LEGS, rear_control_points_front, rear_control_points_back, speed=80, acceleration=40)

    #     # Publish a message to indicate that the gait has been performed
    #     msg = String()
    #     msg.data = 'Hexapod walking with tripod gait.'
    #     self.publisher_.publish(msg)    


    # def tripod_gait_sidewinder_no_timer(self, duration=0.4, time_step=0.01, speed=80, acceleration=40):
    #     """
    #     Tripod walking gait with legs [5, 3, 1] as one set and [4, 2, 0] as the other. Sidewinder version.
    #     """
    #     # Define the tripod control points
    #     forward_control_points = [(120, -6, -60), (250, 10, 10), (120, 25, -60)]
    #     backward_control_points = [(120, 25, -60), (120, -6, -60)]

    #     # walks at just 3.5cm/s !
    #     for i in range(6):
    #         self.set_speed(i, speed, acceleration)

    #     self.three_step_pair_no_timer(duration, time_step, self.RIGHT_TRIPOD_LEGS, self.LEFT_TRIPOD_LEGS, forward_control_points, backward_control_points)
    #     self.three_step_pair_no_timer(duration, time_step, self.LEFT_TRIPOD_LEGS, self.RIGHT_TRIPOD_LEGS, forward_control_points, backward_control_points)
    #     # self.three_step_no_timer(duration, time_step, [5], [], forward_control_points, backward_control_points)
    #     # self.three_step_no_timer(duration, time_step, [], [5], forward_control_points, backward_control_points)

    #     # Publish a message to indicate that the gait has been performed
    #     msg = String()
    #     msg.data = 'Hexapod walking with sidewinder tripod gait.'
    #     self.publisher_.publish(msg)


    # def three_step_pair_no_timer(self, duration, time_step, forward_legs, backward_legs, control_points_1, control_points_2, delay_between_steps = 0.1):
    #     """
    #     Moves both sets of three legs either backwards or forwards to the desired control points
    #     """
    #     num_steps = int(duration / time_step)
        
    #     for i in range(num_steps + 1):
    #         t = i / num_steps
            
    #         forward_pos = bezier_interpolation(control_points_1, t)           # bezier interpolation for forward step
    #         backward_pos = bezier_interpolation(control_points_2, t)         # linear interpolation for backward step

    #         for leg in forward_legs:
    #             if len(forward_legs) > 0:               
    #                 self.move_leg_to(forward_pos, leg)
    #         for leg in backward_legs:
    #             if len(backward_legs) > 0:
    #                 self.move_leg_to(backward_pos, leg)
        
    #         time.sleep(time_step)

    #     time.sleep(delay_between_steps)


    # def calculate_step_points(self, leg, stride, swing_angle, point, sidewinder = False):
    #     """
    #     Calculates the start, mid and end points for a step
    #     """

    #     # Error check that correct parameters have been passed
    #     if (swing_angle > 60 or swing_angle < 20):
    #         self.get_logger().error("Swing angle must be between 20 and 60 degrees")
    #         return None
    #     if (stride < 0 or stride > 100):
    #         self.get_logger().error("Maximum stride length is 100mm")
    #         return None

    #     def calculate_mid_point(start_point, end_point):
    #         x = (start_point[0] + end_point[0]) / 2
    #         y = (start_point[1] + end_point[1]) / 2
    #         mid_point_distance = math.sqrt((start_point[0] - x)**2 + (start_point[1] - y)**2)
    #         z = abs(mid_point_distance) * math.tan(math.radians(swing_angle)) + start_point[2]

    #         return x, y, z
        
    #     # start or end point
    #     match leg:
    #         case 0 | 1 | 2 | 3: # middle and back legs
    #             start_point = point
    #             end_point = (point[0], point[1] + stride, point[2])
    #         case 4 | 5: # front legs
    #             end_point = (point[0], point[1] + stride, point[2]) if sidewinder else point
    #             start_point= point if sidewinder else (point[0], point[1] - stride, point[2])

    #     # mid point
    #     mid_point = calculate_mid_point(start_point, end_point)


    #     return [start_point, mid_point, end_point, start_point]
    

def main(args=None):
    rclpy.init(args=args)
    node = HexapodMovement(device=18)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()