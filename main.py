from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse 
import cv2
import numpy as np
import keyboard

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550') # --connect /dev/ttyTHS1
args = parser.parse_args()

print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

def send_velocity_command(velocity_x, velocity_y, velocity_z):
    
    #velocity_x positive = forward. negative = backwards
    #velocity_y positive = right. negative = left
    #velocity_z positive = down. negative = up

    print("Sending XYZ movement command with v_x(forward/backward): %f v_y(right/left): %f " % (velocity_x,velocity_y,velocity_z))

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,    
        mavutil.mavlink.MAV_FRAME_BODY_NED,  #relative to drone heading pos relative to EKF origin
        0b0000111111100011, #ignore velocity z and other pos arguments
        0, 0, 0,
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)    

    vehicle.send_mavlink(msg)

# arm and take off
def arm_and_takeoff(aTargetAltitude):
  print ("Basic pre-arm checks")

  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")

  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) 

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

# Initialize the takeoff sequence to 5m
arm_and_takeoff(5)

print("Take off complete")

# Hover for 10 seconds
time.sleep(10)

print("searching for a circle tag")

cap=cv2.VideoCapture(1)
font=cv2.FONT_HERSHEY_SIMPLEX

# center point of the drone / camera drone
def draw_center_point(frame):
    height, width, _ = frame.shape
    global center_x, center_y
    center_x = int(width / 2)
    center_y = int(height / 2)
    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 255), -1)

# landing command
def landing():
  print("Now let's land")
  vehicle.mode = VehicleMode("LAND")
  vehicle.close()

# main program for searching circle tag
land_flag = False
while True:
    # emergency landing 
    if keyboard.is_pressed('l'):  # if key 'l' is pressed vehicle manually landing 
            print("Closing due to manual interruption")
            landing()
            break
    
    if land_flag:
       break
    
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, minDist=100,
                              param1=50, param2=30, minRadius=50, maxRadius=100)
    draw_center_point(frame)

    circle = False
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        circle = True
        
        for (x, y, r) in circles:
            tolerance = 20 
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), tolerance, (255, 0, 0), 2)
            cv2.line(frame, (center_x, center_y), (x, y), (0, 255, 255), 2)
            
            diff_x = center_x - x
            diff_y = y - center_y
            
            ratio_tolerance = 1  

            ratio_x = (diff_x + ratio_tolerance) / frame.shape[1]
            ratio_y = (diff_y + ratio_tolerance) / frame.shape[0]

            # ratio between drone and circle
            cv2.putText(frame, f'Ratio: {ratio_x:.2f}, {ratio_y:.2f}', (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # max velocity 1 m/s
            max_velocity = 1 
            velocity_x = ratio_x * max_velocity
            velocity_y = ratio_y * max_velocity

            send_velocity_command(velocity_x, velocity_y, 0)

            # when the drone is in the middle of the circle, then landing.
            if -0.01 < ratio_x < 0.01 and -0.01 < ratio_y < 0.01:
                print("the drone is already in the middle of the circle")
                landing()
                land_flag = True
            
    cv2.putText(frame, "circle " + str(circle), (5, 25), font, 0.8, (0 , 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("deteksi lingkaran",frame)
    if cv2.waitKey(3) & 0xFF ==ord('q'):
        break


cap.release()
cv2.destroyAllWindows()