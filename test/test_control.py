from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

connection_string = '127.0.0.1:14550'
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, baud=921600, wait_ready=True, timeout=180)


class image_converter:
    # Send camera view back from drone for SLAM
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/roscam/cam/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def arm_and_takeoff(aTargetAltitude):
  # Wait for drone to arm and takeoff for given height
  print("Basic checks")
  while not vehicle.is_armable:
    print("Initializing...")
    time.sleep(1)
        
  print("Arming motors")
  vehicle.mode = VehicleMode("GUIDED")
  vehicle.armed = True

  print("Arming...")
  while not vehicle.armed:
    print(vehicle.armed)
    time.sleep(1)

  print("Taking off...")
  vehicle.simple_takeoff(aTargetAltitude+1)

  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)    
    if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.8: 
      print("Reached target altitude")
      break
    time.sleep(1)

# Not in use
def wait_for_position(target_location):
    # Wait for drone to arrive the point
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, target_location)
        print("Distance to target: ", distance)
        if distance < 0.1:  # 0.1 meters tolerance
            print("Reached target location")
            break
        time.sleep(1)

def get_distance_metres(location1, location2):
    # Get the distance of target and drone
    dlat = location2.lat - location1.lat
    dlon = location2.lon - location1.lon
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5

def goto(lon, lat, wait_time):
    # Let the drone go to specific point using lon and lat,
    # giving wait time to wait until next instruction
    print(f"Moving to {lon}, {lat}...")
    location = LocationGlobalRelative(lon,
                                      lat,
                                      vehicle.location.global_relative_frame.alt)
    vehicle.simple_goto(location)
    time.sleep(wait_time)

# This one not working
def height_adjust(height):
    # Adjust the height of the drone
    print("Drone moving...alt adding", height)
    print(vehicle.location.global_relative_frame.alt)
    location = LocationGlobalRelative(vehicle.location.global_relative_frame.lon,
                                      vehicle.location.global_relative_frame.lat,
                                      (vehicle.location.global_relative_frame.alt + height))
    vehicle.simple_goto(location)
    time.sleep(3)
    print(vehicle.location.global_relative_frame.alt)

def reset_position():
    # Let the drone go back to simulation world's origin (approximately)
    print("Reseting drone to origin position...")
    location = LocationGlobalRelative(-35.3632631,
                                      149.1652375,
                                      vehicle.location.global_relative_frame.alt)
    vehicle.simple_goto(location)
    time.sleep(3)
    print(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)

def lon_plus():
    # Move the drone's lon relatively one meter (code will then change it to lon and lat)
    print("Lon plus")
    location = LocationGlobalRelative(vehicle.location.global_frame.lat,
                                      vehicle.location.global_frame.lon + (1 / 111319.5),
                                      vehicle.location.global_relative_frame.alt)
    vehicle.simple_goto(location)
    time.sleep(1.5)
    # wait_for_position(location)

def lat_plus():
    # Move the drone's lat relatively one meter (code will then change it to lon and lat)
    print("Lat plus")
    location = LocationGlobalRelative(vehicle.location.global_frame.lat + (1 / 111319.5),
                                      vehicle.location.global_frame.lon,
                                      vehicle.location.global_relative_frame.alt)
    vehicle.simple_goto(location)
    time.sleep(1.5)
    # wait_for_position(location)

def lon_minus():
    # Move the drone's lon relatively minus one meter (code will then change it to lon and lat)
    print("Lon minus")
    location = LocationGlobalRelative(vehicle.location.global_frame.lat,
                                      vehicle.location.global_frame.lon - (1 / 111319.5),
                                      vehicle.location.global_relative_frame.alt)
    vehicle.simple_goto(location)
    time.sleep(1.5)
    # wait_for_position(location)

def lat_minus():
    # Move the drone's lat relatively minus one meter (code will then change it to lon and lat)
    print("Lat minus")
    location = LocationGlobalRelative(vehicle.location.global_frame.lat - (1 / 111319.5),
                                      vehicle.location.global_frame.lon,
                                      vehicle.location.global_relative_frame.alt)
    vehicle.simple_goto(location)
    time.sleep(1.5)
    # wait_for_position(location)

def navigation():
    # Use to let drone make a retangle movement in sim world
    reset_position()

    goto(-35.3632038, 149.165281, 5) # Top left corner

    # height_adjust(-2) # Going down 2 meter

    # goto(-35.3632631, 149.165281) # Go →
    goto(-35.3633214, 149.165281, 5) # Go →

    # goto(-35.3633214, 149.1652375) # Go ↓
    goto(-35.3633214, 149.1651860, 4) # Go ↓

    # goto(-35.3632631, 149.1651900) # Go ←
    goto(-35.3632038, 149.1651860, 5) # Go ←

    # goto(-35.3632038, 149.1652375) # Go ↑
    goto(-35.3632038, 149.165281, 4) # Go ↑

    # height_adjust(2) # Going up 2 meter

    reset_position()

    vehicle.mode = VehicleMode("LAND")
    print("Drone landing...")
    vehicle.close()
    print("Navigation finished.")
    exit(0)

def main(args):

    arm_and_takeoff(4)
    print("Finished take off")

    ic = image_converter()
    print("Initializing ROS node...")

    rospy.init_node('image_converter', anonymous=True)
    print("ROS node initialized.")

    time.sleep(3)
    navigation()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

    vehicle.mode = VehicleMode("LAND")
    print("Drone landing...")
    vehicle.close()
    print("Drone landed.")

if __name__ == '__main__':
    main(sys.argv)
