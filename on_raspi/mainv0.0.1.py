# coding: UTF-8
import time
import os
import math
import csv

import apriltag
import cv2

from dronekit import connect
import dronekit
from pymavlink import mavutil

import RPi.GPIO as GPIO
from datetime import date

##setup device
pix = connect('/dev/ttyAMA0', wait_ready=True,baud=500000,timeout = 40)

##setup led pin
LED_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN,GPIO.OUT)

#setup dir
today = date.today()
timer = time.time()
D = today.strftime("%d_%m_%Y")
try:
    os.stat(D)
except:
    os.mkdir(D)
mission_dir = os.listdir("./%s"%D)
this_mission_dir = "0"
for i in range(1,100,1):
    if not str(i) in mission_dir:
        this_mission_dir = str(i)
        break
dir = "%s/%s"%(D,this_mission_dir)
os.mkdir(dir)

#setup camera
cap = cv2.VideoCapture(0)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH )/2)
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT )/2)
size = (cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
video_file = "mission_video.mp4"
video_fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = cap.get(cv2.CAP_PROP_FPS)
W= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out = cv2.VideoWriter('output.mp4',video_fourcc,fps,(W, H))

blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)

#apriltag detector
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9') )

#map
with open('map.csv', newline='') as csvfile
map = csv.DictReader(csvfile)

'''--mission variable--'''
##drone frame control variable
mavlink_eslaped = 0.1 #mavlink send pause timer
send_timer = 0

#mission part
chan_state = []
mission_start = False
mission_code = 0
mission_complete = False

##take_off part
DEFAULT_TAKEOFF_THRUST = 0.7
SMOOTH_TAKEOFF_THRUST = 0.6

takeoff_step = 0
takeoff_done = False
take_off_tag = 0

#take photo part
pre_tag = 0
photo_index = []

'''---function---'''
#offcial command
'''
# Uncomment the lines below for testing roll angle and yaw rate.
# Make sure that there is enough space for testing this.

# set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
# set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
'''
def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    '''
    while not pix.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    '''

    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    pix.mode = VehicleMode("GUIDED_NOGPS")
    pix.armed = True

    while not pix.armed:
        print(" Waiting for arming...")
        pix.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        #current_altitude = pix.location.global_relative_frame.alt
        current_altitude = pix.rangefinder.distance
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)
def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = pix.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = pix.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    pix.send_mavlink(msg)
def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    '''
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    '''
    global send_timer
    if time.time() - send_timer < duration:
        if time.time() - send_timer % 0.1 == 0 :
            print("mav_send")
            send_attitude_target(roll_angle, pitch_angle,
                                 yaw_angle, yaw_rate, False,
                                 thrust)
    # Reset attitude, or it will persist for 1s more due to the timeout
    else:
        send_attitude_target(0, 0,
                             0, 0, True,
                             thrust)
        send_timer = time.time()
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

#mavlink
def send_ned_velocity(velocity_x, velocity_y, velocity_z,heading):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = pix.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, heading)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # Set up velocity mappings
        # velocity_x > 0 => fly North
        # velocity_x < 0 => fly South
        # velocity_y > 0 => fly East
        # velocity_y < 0 => fly West
        # velocity_z < 0 => ascend
        # velocity_z > 0 => descend
    pix.send_mavlink(msg)

#takeoff
def takeoff_tag_detect():
    cv2.drawMarker(frame, (width, height), (0, 0, 255), cv2.MARKER_CROSS, 10, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)

    detect_id = 1000
    first_detect = 0
    pre_dis = 1000

    return_data = [0, 0, 0, 0, False]  # angle,distance,dx,dy

    global timer,take_off_tag
    tag_take_ready = False

    if (time.time() - timer > 2):
        timer = time.time()
        tag_take_ready = True

    GPIO.output(LED_PIN, GPIO.LOW)
    for tag in tags:
        color = (255,255,255)

        if (tag.tag_id == take_off_tag):
            GPIO.output(LED_PIN, GPIO.HIGH)

            cv2.line(frame, (width, height), tuple(tag.center.astype(int)), (255, 0, 0), 3)
            color = green
            tag_1 = int(
                math.sqrt((tag.corners[0][0] - tag.corners[1][0]) ** 2 + (tag.corners[0][1] - tag.corners[1][1]) ** 2))
            tag_2 = int(
                math.sqrt((tag.corners[1][0] - tag.corners[2][0]) ** 2 + (tag.corners[1][1] - tag.corners[2][1]) ** 2))
            tag_3 = int(
                math.sqrt((tag.corners[2][0] - tag.corners[3][0]) ** 2 + (tag.corners[2][1] - tag.corners[3][1]) ** 2))
            tag_4 = int(
                math.sqrt((tag.corners[3][0] - tag.corners[0][0]) ** 2 + (tag.corners[3][1] - tag.corners[0][1]) ** 2))

            mid_1 = (int((tag.corners[0][0] + tag.corners[1][0]) / 2), int((tag.corners[0][1] + tag.corners[1][1]) / 2))
            mid_2 = (int((tag.corners[1][0] + tag.corners[2][0]) / 2), int((tag.corners[1][1] + tag.corners[2][1]) / 2))
            mid_3 = (int((tag.corners[2][0] + tag.corners[3][0]) / 2), int((tag.corners[2][1] + tag.corners[3][1]) / 2))
            mid_4 = (int((tag.corners[3][0] + tag.corners[0][0]) / 2), int((tag.corners[3][1] + tag.corners[0][1]) / 2))

            cv2.putText(frame, str(tag_1), mid_1, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, str(tag_2), mid_2, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, str(tag_3), mid_3, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, str(tag_4), mid_4, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)

            angle_x = 300 * (tag_4 / tag_2 - 1)
            len = (tag_2 + tag_4) / 2
            distance = -0.000226 * (len ** 3) + 0.057933 * (len ** 2) - 5.2449 * (len) + 198.14

            return_data[0] = angle_x
            return_data[1] = distance
            return_data[2] = tag.center[0] - width
            return_data[3] = tag.center[0] - height
            return_data[4] = True

            cv2.putText(frame, "ang: " + str(angle_x), (50, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(frame, "dis: " + str(distance), (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 1,
                        cv2.LINE_AA)
        else:
            timer = time.time()

        cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, color, 2)  # left-top
        cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, color, 2)  # right-top
        cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, color, 2)  # right-bottom
        cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, color, 2)  # left-bottom
        text = str(tag.tag_id)
        text = str(tag.center.astype(int))
        cv2.putText(frame, text, tuple(tag.center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1,
                    cv2.LINE_AA)
    return tag_take_ready
def takeoff(aTargetAltitude):
    global takeoff_step,takeoff_done,TIME
    if(takeoff_step == 0):
        if(takeoff_tag_detect()):
            takeoff_step = 1
        else:
            print("Non-detected takeoff_tag")

    elif(takeoff_step == 1):
        pix.mode = dronekit.VehicleMode("GUIDED_NOGPS")
        if(not pix.is_armable):
            print("Wait drone initialize")
        else:
            print("ok")
            takeoff_step  = 2
            pix.armed = True
        takeoff_step  = 2
        pix.armed = True
            
    elif(takeoff_step == 2):
        if not pix.armed:
            print("wait for arm..")
        else:
            print("take_off")
            takeoff_step = 3
     
    elif(takeoff_step == 3):
        print("----takeoff----")
        current_altitude = pix.rangefinder.distance
        print(" Altitude: %f  Desired: %f" % (current_altitude, aTargetAltitude))

        thrust = DEFAULT_TAKEOFF_THRUST
        if current_altitude >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            takeoff_step = 4
            TIME = time.time()
        elif current_altitude >= aTargetAltitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)

    elif(takeoff_step == 4):
        print("takeoff done!")
        set_attitude()
        if(time.time() - TIME > 0.5):
            takeoff_done = True

#camera
def get_frame():
    global ret, frame
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
def photo_by_tag(INDEX):

    cv2.drawMarker(frame, (width, height), (0, 0, 255), cv2.MARKER_CROSS, 10, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)

    detect_id = 1000
    first_detect = 0
    pre_dis = 1000
    
    return_data = [0,0,0,0,False] #angle,distance,dx,dy
    
    for tag in tags:
        if tag.tag_id not in INDEX:
            dx = tag.center[0] - width
            dy = tag.center[1] - height
            tag_dis = math.sqrt(dx**2 + dy**2)
            
            if( tag_dis < pre_dis):
                pre_dis = tag_dis
                detect_id = tag.tag_id
                
    global pre_tag,timer
    if(detect_id != pre_tag or pre_dis >= 30 ):
        pre_tag = detect_id
        timer = time.time()
        
    if(time.time() - timer > 2):
        cv2.imwrite(dir + '/%s.jpg'%detect_id,frame)
        INDEX.append(detect_id)
        timer = time.time()
        
    GPIO.output(LED_PIN, GPIO.LOW)
    for tag in tags:
            
        color = red
        
        if( tag.tag_id == detect_id):
            GPIO.output(LED_PIN, GPIO.HIGH)
            if(pre_dis > 30):
                cv2.line(frame,(width,height),tuple(tag.center.astype(int)),(255,255,255),3)
            else:
                cv2.line(frame,(width,height),tuple(tag.center.astype(int)),(255,0,0),3)
            color = green
            tag_1 = int(math.sqrt((tag.corners[0][0] - tag.corners[1][0])**2 + (tag.corners[0][1] - tag.corners[1][1])**2))
            tag_2 = int(math.sqrt((tag.corners[1][0] - tag.corners[2][0])**2 + (tag.corners[1][1] - tag.corners[2][1])**2))
            tag_3 = int(math.sqrt((tag.corners[2][0] - tag.corners[3][0])**2 + (tag.corners[2][1] - tag.corners[3][1])**2))
            tag_4 = int(math.sqrt((tag.corners[3][0] - tag.corners[0][0])**2 + (tag.corners[3][1] - tag.corners[0][1])**2))
            
            mid_1 = (int((tag.corners[0][0] + tag.corners[1][0])/2) , int((tag.corners[0][1] + tag.corners[1][1])/2))
            mid_2 = (int((tag.corners[1][0] + tag.corners[2][0])/2) , int((tag.corners[1][1] + tag.corners[2][1])/2))
            mid_3 = (int((tag.corners[2][0] + tag.corners[3][0])/2) , int((tag.corners[2][1] + tag.corners[3][1])/2))
            mid_4 = (int((tag.corners[3][0] + tag.corners[0][0])/2) , int((tag.corners[3][1] + tag.corners[0][1])/2))
            
            cv2.putText(frame, str(tag_1), mid_1, cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, str(tag_2), mid_2, cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, str(tag_3), mid_3, cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, str(tag_4), mid_4, cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 255), 1, cv2.LINE_AA)
            
            angle_x = 300*(tag_4/tag_2 - 1)
            len = (tag_2 + tag_4)/2
            distance = -0.000226*(len**3)+0.057933*(len**2)-5.2449*(len)+198.14
            
            return_data[0] = angle_x
            return_data[1] = distance
            return_data[2] = tag.center[0] - width
            return_data[3] = tag.center[0] - height
            return_data[4] = True
            
            cv2.putText(frame, "ang: "+str(angle_x),(50,430), cv2.FONT_HERSHEY_SIMPLEX,0.6, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, "dis: "+str(distance), (50,400), cv2.FONT_HERSHEY_SIMPLEX,0.6, (255, 0, 255), 1, cv2.LINE_AA)
                
        if tag.tag_id in INDEX:
            color = (255,255,255)
            
        cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, color, 2) # left-top
        cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, color, 2) # right-top
        cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, color, 2) # right-bottom
        cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, color, 2) # left-bottom
        text = str(tag.tag_id)
        text = str(tag.center.astype(int))
        cv2.putText(frame, text, tuple(tag.center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
    
    return return_data
def recording():
        out.write(frame)
def camera_release():
        cap.release()
        out.release()

#map nav
def map_pos(tagID):
    if tagID not in map['ID']:
        return(0,0)

    index = map['ID'].index(tagID)
    posX = map['posX'][index]
    posY = map['posY'][index]

    return(posX,posY)

#body control
def focus_on_tag(tag):
    dx = tag.center[0] - width
    dy = tag.center[1] - height
    tag_dis = math.sqrt(dx ** 2 + dy ** 2)

    if(tag_dis > 15):
        if(dx > 0):
            roll = 3
        elif(dx < 0):
            roll = -3

        if(dy > 0):
            pitch = 3
        elif(dy < 0):
            roll = -3
    ang = tag_direction(tag)
    return pitch,roll,ang

def status_check():
    error_code = 0
    if(pix.battery.voltage < 11.3):
        print("----battery LOW----")
        error_code += 0b00000001
        
    if(pix.mode != dronekit.VehicleMode("GUIDED")):
        print("----mode wrong----")
        error_code += 0b00000010
    
    if(not mission_start):
        chan_state = pix.channels.copy()
    else:
        if(chan_state != pix.channels ):
            print("----manual interrupt----")
    return error_code
def tag_direction(tag):
    LT = tag.corners[0]
    RT = tag.corners[1]
    RB = tag.corners[2]
    LB = tag.corners[3]

    heading_vector = (LT[0] - LB[0], LT[1] - LB[1])

    if(heading_vector[0]>=0 ):
            Direction = 90 + math.degrees(math.atan(heading_vector[1] / heading_vector[0]))
    elif(heading_vector[0]<0):
            Direction = -90 + math.degrees(math.atan(heading_vector[1] / heading_vector[0]))

    return Direction
def tag_to_tag(TAG,OLDpos,NEWpos,ANG):
    distanceX = NEWpos[0] - OLDpos[0]
    distanceY = NEWpos[1] - OLDpos[1]

    pitch,roll,ang = focus_on_tag(TAG)

    if(distanceX > 0):
        roll = 3
    elif(distanceX < 0):
        roll = -3
    if(distanceX == 0):
        if(distanceY > 0):
            pitch = 3
        elif(distanceY < 0):
            pitch = -3

    if(ANG > 15):
        yaw = 15
    elif(ANG < -15)
        yaw = -15

    set_attitude(roll_angle=roll,pitch_angle=pitch,yaw_rate=yaw,thrust = 0.5)
def tag_nav(target_id):
    global MAIN_TAG
    cv2.drawMarker(frame, (width, height), (0, 0, 255), cv2.MARKER_CROSS, 10, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)

    detect_id = 1000
    first_detect = 0
    pre_dis = 1000

    GPIO.output(LED_PIN, GPIO.LOW)

    for tag in tags:
        if tag.tag_id not in INDEX:
            dx = tag.center[0] - width
            dy = tag.center[1] - height
            tag_dis = math.sqrt(dx ** 2 + dy ** 2)

            if (tag_dis < pre_dis):
                pre_dis = tag_dis
                detect_id = tag.tag_id

    for tag in tags:
        color = red
        if (tag.tag_id == detect_id):
            GPIO.output(LED_PIN, GPIO.HIGH)
            if (pre_dis > 30):
                cv2.line(frame, (width, height), tuple(tag.center.astype(int)), (255, 255, 255), 3)
            else:
                cv2.line(frame, (width, height), tuple(tag.center.astype(int)), (255, 0, 0), 3)
            color = green
            MAIN_TAG = tag

        cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, color, 2)  # left-top
        cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, color, 2)  # right-top
        cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, color, 2)  # right-bottom
        cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, color, 2)  # left-bottom
        text = str(tag.tag_id)
        text = str(tag.center.astype(int))
        cv2.putText(frame, text, tuple(tag.center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1,
                    cv2.LINE_AA)

    tag1 = map_pos(detect_id)
    tag2 = map_pos(target_id)
    ang = tag_direction(MAIN_TAG)

    tag_to_tag(MAIN_TAG,tag1,tag2,ang)

    return tag1


#main program
print("system startup")
while(0):
    get_frame()
    
    cv2.imshow('cap',frame)
    out.write(frame)
    K = cv2.waitKey(1)    
    if(K == 27):
        break
    
while(1):
    get_frame()
    if(pix.mode == dronekit.VehicleMode("GUIDED_NOGPS")):
        mission_start = True
    else:
        mission_start = False

    DIR = 0

    #mission detect
    if(not takeoff_done):
        mission_code = 1
    elif(takeoff_done):
        mission_code = 2
    elif(mission_complete):
        mission_code = 3

    #mission start
    if(mission_start):
        if(mission_code == 0):
            print('ok')
        elif(mission_code == 1):
            takeoff(1)
        elif(mission_code == 2):
            LOCATION = tag_nav(DIR)
            if(LOCATION == DIR):
                print("GET POINT")
        elif(mission_code == 3):
            print("2")
        elif(mission_code == 4):
            print("3")
        elif(mission_code == 5):
            print("4")
        else:
            pix.mode = dronekit.VehicleMode("LAND")
            print("system error")

    #print("RangeFinder: "+ str(pix.rangefinder.distance) + " m")

    cv2.imshow('cap',frame)
    K = cv2.waitKey(1)    
    if(K == 27):
        break

#safety mode change
pix.mode = dronekit.VehicleMode("LAND")

#system release
camera_release()
#pix.flush()
cv2.destroyAllWindows()
GPIO.cleanup()