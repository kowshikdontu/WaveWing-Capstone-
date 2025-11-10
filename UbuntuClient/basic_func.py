from simple_pid import PID
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import numpy as np
import math

class MotionTracker:
    def __init__(self, vehicle, dt):
        self.vehicle = vehicle
        self.dt = dt
        self.position = np.zeros(3)  # [x, y, z]
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)  # [roll, pitch, yaw]`
        self.last_update = time.time()

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        self.velocity = np.array(self.vehicle.velocity)
        self.orientation = np.array([self.vehicle.attitude.roll,
                                     self.vehicle.attitude.pitch,
                                     self.vehicle.attitude.yaw])
        self.position += self.velocity * dt

    def reset(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity
    def get_orientation(self):
        return self.orientation


class DroneController:
    def __init__(self, MotionTracker, vehicle):
        self.vehicle = vehicle
        self.MotionTracker = MotionTracker
        self.pid_x = PID(0.8, 0.05, 0.02, setpoint=0)
        self.pid_y = PID(0.8, 0.05, 0.02, setpoint=0)
        self.pid_z = PID(0.8, 0.05, 0.02, setpoint=0)
        self.pid_yaw = PID(0.5, 0.05, 0.02, setpoint=0)
        self.pid_yaw.output_limits = (-2, 2)  # Reduced yaw rate for smoother rotation
        self.pid_x.output_limits = (-1.0, 1.0)
        self.pid_y.output_limits = (-1.0, 1.0)
        self.pid_z.output_limits = (-1.0, 1.0)

    def arm_and_takeoff(self, target_altitude):
        print("Arming motors...")
        # print(id(vehicle))
        while not self.vehicle.is_armable:
             print(" Waiting for vehicle to become armable...")
             time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)

        # while True:
        #     print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
        #     if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        #         print("Reached target altitude")
        #         break
        #     time.sleep(1)

    def send_velocity(self, vx, vy, vz, duration=0,yaw_rate=0):
        # Use MAV_FRAME_LOCAL_NED to ensure movement in the global frame
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Bitmask to control velocity (vx, vy, vz) and yaw_rate
            0, 0, 0,  # No need to set position
            vx, vy, vz,  # Desired velocity in NED frame
            0, 0, 0,  # No acceleration control
            yaw_rate, 1 # Yaw rate control
        )
        self.vehicle.send_mavlink(msg)
        for _ in range(int(duration * 10)):
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)

        self.vehicle.flush()

    def send_rc_override(self, yaw_value):

        rc_override = [0] * 8
        rc_override[3] = yaw_value  # Channel 4 controls yaw (1500 is neutral)
        self.vehicle.channels.overrides = {
            '4': yaw_value
        }

    def yaw_to_angle(self, target_yaw):

        self.MotionTracker.reset()
        self.pid_yaw.setpoint = target_yaw  # Target yaw angle
        while True:
            # Update motion tracking
            self.MotionTracker.update()

            # Get current yaw
            current_yaw = self.MotionTracker.get_orientation()[2]

            # Calculate yaw control output from PID controller
            yaw_correction = self.pid_yaw(current_yaw)

            # Send yaw control signal via RC override
            yaw_override_value = int(1500 + yaw_correction)  # Centered at 1500
            self.send_rc_override(yaw_override_value)

            print(f"Current Yaw: {current_yaw:.2f}, Target: {target_yaw}, RC Yaw: {yaw_override_value}")

            # Stop yawing if within tolerance (2 degrees)
            if abs(target_yaw - current_yaw) < 2:
                break

            time.sleep(0.1)

        # Stop the yaw motion by sending neutral yaw (1500)
        self.send_rc_override(1500)
        print("Yaw completed.")
    def goto_position_target_local_ned(self,north, east, down):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()



    def move_forward(self, distance):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[0] + distance
        while abs(target_dist -self.MotionTracker.position[0]) > 0.05:
            self.MotionTracker.update()

            velocity_x = self.pid_x(-target_dist +self.MotionTracker.position[0])
            print(self.MotionTracker.position, velocity_x)
            self.send_velocity(velocity_x, 0, 0)
            time.sleep(0.1)
        self.send_velocity(0, 0, 0)  # Stop the drone after moving
        print("Moved forward")

    def move_backward(self, distance):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[0] - distance
        while abs(self.MotionTracker.position[0] - target_dist) > 0.05:
            self.MotionTracker.update()

            velocity_x = self.pid_x(-target_dist +self.MotionTracker.position[0])
            print(self.MotionTracker.position,velocity_x)
            self.send_velocity(velocity_x, 0, 0)
            time.sleep(0.1)
        self.send_velocity(0, 0, 0)  # Stop the drone after moving
        print("Moved backward")

    def move_left(self, distance):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[1] - distance
        while abs(self.MotionTracker.position[1] - target_dist) > 0.05:
            self.MotionTracker.update()
            velocity_y = self.pid_y(target_dist - self.MotionTracker.position[1])
            self.send_velocity(0, -velocity_y, 0)
            time.sleep(0.1)
        self.send_velocity(0, 0, 0)  # Stop the drone after moving
        print("Moved left")

    def move_right(self, distance):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[1] + distance
        while abs(target_dist - self.MotionTracker.position[1]) > 0.05:
            self.MotionTracker.update()

            velocity_y = self.pid_y(-target_dist +self.MotionTracker.position[1])
            print(self.MotionTracker.position,velocity_y)
            self.send_velocity(0, velocity_y, 0)
            time.sleep(0.1)
        self.send_velocity(0, 0, 0)  # Stop the drone after moving
        print("Moved right")

    def move_up(self, altitude):
        self.MotionTracker.reset()
        target_altitude = self.MotionTracker.position[2] + altitude
        while target_altitude + self.MotionTracker.position[2] > 0.05:
            self.MotionTracker.update()
            velocity_z = self.pid_z(target_altitude - self.MotionTracker.position[2])
            print(self.MotionTracker.position,velocity_z)
            self.send_velocity(0, 0, velocity_z)
            time.sleep(0.1)
        self.send_velocity(0, 0, 0)  # Stop the drone after moving
        print("Moved up")

    def move_down(self, altitude):
        self.MotionTracker.reset()
        target_altitude = self.MotionTracker.position[2] - altitude
        if abs(target_altitude + self.MotionTracker.position[2]) > 0.05:
            self.MotionTracker.update()
            velocity_z = self.pid_z(target_altitude + self.MotionTracker.position[2])
            self.send_velocity(0, 0, velocity_z)
            time.sleep(0.1)
        self.send_velocity(0, 0, 0)  # Stop the drone after moving
        print("Moved down")



    def condition_yaw(self,heading, relative=False):
        if relative:
            is_relative = 1
        else:
            is_relative = 0

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            10,
            1,
            is_relative,
            0, 0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def revolve_around_point_ned(self,radius, revolutions=1, speed=2):
        num_steps = 36
        total_steps = num_steps * revolutions
        angle_step = 360 / num_steps
        angular_velocity = speed / radius

        for step in range(total_steps):
            angle = math.radians(step * angle_step)

            velocity_x = -radius * angular_velocity * math.sin(angle)  # North component
            velocity_y = radius * angular_velocity * math.cos(angle)  # East component
            self.send_velocity(velocity_x, velocity_y, 0, 1)  # Move for 1 second at a time
            target_yaw = math.degrees(math.atan2(-velocity_y, -velocity_x))  # Yaw to face inward
            self.condition_yaw(target_yaw)

    def move_in_clock_direction(self,clock_direction):
        clock_yaw_map = {
            12: 0,  # Straight ahead (12 o'clock = 0 degrees)
            1: 30,  # 1 o'clock = 30 degrees
            2: 60,  # 2 o'clock = 60 degrees
            3: 90,  # 3 o'clock = 90 degrees (right)
            4: 120,  # 4 o'clock = 120 degrees
            5: 150,  # 5 o'clock = 150 degrees
            6: 180,  # 6 o'clock = 180 degrees (backward)
            7: 210,  # 7 o'clock = 210 degrees
            8: 240,  # 8 o'clock = 240 degrees
            9: 270,  # 9 o'clock = 270 degrees (left)
            10: 300,  # 10 o'clock = 300 degrees
            11: 330,  # 11 o'clock = 330 degrees
        }
        yaw_angle = clock_yaw_map.get(clock_direction, 0)

        self.condition_yaw(yaw_angle)


    def stop(self):
        self.send_velocity(0, 0, 0, 0)
        print("Stopped")


# Connect to the vehicle
# vehicle = connect('127.0.0.1:14550', wait_ready=True)
# # vehicle = connect('/dev/ttyACM0',baud=256000, wait_ready=True)
# # print(dir(vehicle.message_factory))
#
# motion_tracker = MotionTracker(vehicle, 0.1)
# controller = DroneController(motion_tracker, vehicle)
# vehicle.mode = VehicleMode("STABILIZE")
# vehicle.parameters['LOG_DISARMED'] = 1
#
# vehicle.parameters['ARMING_CHECK'] = 1
# vehicle.parameters['WP_YAW_BEHAVIOR'] = 1
#
# print(id(vehicle))

# controller.arm_and_takeoff(1)
# controller.arm_and_takeoff(1)
# controller.arm_and_takeoff(1)
# controller.arm_and_takeoff(1)
# # Test the functions
# controller.move_forward(100)
# controller.move_right(2)
# controller.move_backward(4)
# controller.move_left(2)
# controller.send_velocity(0,0,0)
# controller.move_up(3)
# controller.move_down(3)
# controller.send_velocity(90)
# controller.yaw_to_angle(180)
# controller.condition_yaw(30,True)
# controller.stop()
