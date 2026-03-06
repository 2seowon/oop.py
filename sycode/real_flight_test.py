import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode


class FlightTest(Node):
    def __init__(self):
        super().__init__('flight_test')

        self.offboard_enabled = False
        self.armed = VehicleStatus.ARMING_STATE_DISARMED  # 1, armed == 2, msg.arming_state

        # waypoints for the drone to target
        self.target = [
            [0, 0, -10],
            [5, 0, -10],
            [5, 5, -10],
            [5, -5, -10],
            [5, 0, -10],
            [0, 0, -10]
        ]

        # target coordinate is given as self.target[self.target_index]
        self.target_index = 0

        # counter to check how long the aircraft has stayed at the target
        self.reached_target_counter = 0

        # distance threshold to decide if the aircraft has reached the target
        # needed since the aircraft is very unlikely to stay exactly at the target coordinate without any error
        self.distance_threshold = 2.0

        # number of iterations after reaching the target before moving on to the next target
        self.hold_threshold = 30

        # variables for setting topic pub/sub qos(quality of servic)
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        # subscribers for needed topics
        # subscriber = self.create_subscription(topic type, topic name, callback function, qos)
        # callback function runs everytime the topic is received
        # to update a variable to the newest value of a topic, it should be done by the callback function
        self.subscriber_odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                                            self.callback_odometry, qos_profile_sub)
        self.subscriber_vehicle_status = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                                                  self.callback_vehicle_status, qos_profile_sub)
        self.subscriber_vehicle_control_mode = self.create_subscription(VehicleControlMode,
                                                                        '/fmu/out/vehicle_control_mode',
                                                                        self.callback_vehicle_control_mode,
                                                                        qos_profile_sub)

        # publishers for needed topics
        # publisher = self.create_publisher(topic type, topic name, qos)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',
                                                             qos_profile_pub)
        self.publisher_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                                                   qos_profile_pub)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command',
                                                               qos_profile_pub)

        timer_period = 0.01  # sec, 10Hz should be >2Hz

        # timer is run every iteration
        # timer = self.create_timer(timer period, callback function
        # timer's callback function can be thought of as the main function of a code
        self.timer = self.create_timer(timer_period, self.callback_cmdloop)

    # callback function of /fmu/out/vehicle_odometry
    # updates position
    def callback_odometry(self, msg):
        self.position = msg.position

    # callback function of /fmu/out/vehicle_status_v1
    # checks if the aircraft is armed
    def callback_vehicle_status(self, msg):
        self.armed = msg.arming_state

    # callback function of /fmu/out/vehicle_control_mode
    # checks if aircraft is in offboard mode
    def callback_vehicle_control_mode(self, msg):
        self.offboard_enabled = msg.flag_control_offboard_enabled

    # tells fc which field will be controlled
    # also acts as a heartbeat, fc should receive the message >2Hz or offboard mode is aborted
    def publish_offboard_mode(self):
        msg = OffboardControlMode()

        # timestamp to tell fc when the message was sent
        # all messeages should include timestamp
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # set True for the field to control
        # other fields are set to False
        # fields have priority starting from position to direct actuator
        # only the first field to be set to True is controlled in multirotors
        msg.position = True  # position based control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.publisher_offboard_mode.publish(msg)

    # various commands can be sent by assinging command parameter
    # set param1, param2 to match the command's intentions
    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    def publish_arm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        print("Arm command send")

    def publish_disarm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        print("Disarm command send")

    # publish target coordinate
    def publish_trajectory_setpoint(self, p):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = p
        self.publisher_trajectory_setpoint.publish(msg)

    # check if aircraft has reached the target
    def close_enough(self):
        sc = self.setpoint_counter
        distance_to_target = math.dist(self.position, self.target[sc])
        if distance_to_target < self.distance_threshold:
            return True
        else:
            return False

    def callback_cmdloop(self):
        if self.armed == VehicleStatus.ARMING_STATE_DISARMED:
            self.publish_arm_command()
            # sets vehicle to offboard mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # should always be sent every loop
        # if fc receives the message <2Hz, offboard mode is aborted
        self.publish_offboard_mode()

        # publish target position
        self.publish_trajectory_setpoint(self.[self.target_index])

        if self.close_enough():
            self.reached_target_counter += 1
            # if stayed at the target for long enough
            if self.reached_target_counter > self.hold_threshold:
                self.target_index += 1  # move to next target
                self.reached_target_counter = 0  # reset reached_target_counter


# runs the node
def main(args=None):
    rclpy.init(args=args)

    flight_test = FlightTest()
    rclpy.spin(flight_test)

    flight_test.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
