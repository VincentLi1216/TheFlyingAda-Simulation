from dronekit import connect, VehicleMode
import time

connection_string = '127.0.0.1:14550'
# Connect to the Vehicleprint('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, baud=921600, wait_ready=True, timeout=180)

def arm_and_takeoff(aTargetAltitude):

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
    
arm_and_takeoff(3)
print("Finished take off")
time.sleep(3)
vehicle.mode = VehicleMode("LAND")
vehicle.close()
