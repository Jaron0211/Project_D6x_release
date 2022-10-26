# coding: UTF-8
#dev
test_mode = True

import time
import os
import math
import csv
import socket

import apriltag
import cv2
import numpy as np

from dronekit import connect
import dronekit
from pymavlink import mavutil

import RPi.GPIO as GPIO
from datetime import date
##socket
#HOST = '192.168.50.157'
HOST = '192.168.50.157'
PORT = 8000


#setup UDP
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client.connect((HOST, PORT))


##global log
log_msg = ''
log_timer = 0
log_file = open('log.txt','w+')

##setup device
pix = connect('/dev/ttyAMA0', wait_ready=False,baud=500000,timeout = 40)

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

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE,1.2)

blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)

#external camera
camera = cv2.VideoCapture(1)

#apriltag detector
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9') )

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

takeoff_timer = 0
arm_timer = 0

##land part
landing_timer = 0

#take photo part
pre_tag = 0
photo_index = []

#GLOBAL NAV
REF_HOME = (0,0,0)
XV, YV, ZV, AV = 0,0,0,0
body_x,body_y = 0,0

#task manager
task_start = False
task_ID = 0
TASK_TIMER = 0
TASK_FINISH = False
task_all_done = False

#Failsafe
failsafe_channels = ()
FAILSAFE = False
FAILSAFE_CHANNEL_STATE = False

'''---function---'''
#offcial command
'''
# Uncomment the lines below for testing roll angle and yaw rate.
# Make sure that there is enough space for testing this.

# set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
# set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
'''
def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = True,
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
    '''
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    '''
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
def takeoff_old(aTargetAltitude):
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
def takeoff(aTargetAltitude):
    global takeoff_step,takeoff_timer,arm_timer,log_msg

    takeoff_done = False
    current_altitude = pix.rangefinder.distance
    thrust = 0

    if(pix.armed):
        if(takeoff_step == 0):
            log_msg += '\n----takeoff_prepare----'
            thrust = 0.2
            if(time.time() - takeoff_timer >= 5):
                takeoff_step += 1
                takeoff_timer = time.time()
        elif(takeoff_step == 1):
            thrust = 0.4
            if (time.time() - takeoff_timer >= 3):
                takeoff_step += 1
                takeoff_timer = time.time()
        elif(takeoff_step == 2):

            if(current_altitude < aTargetAltitude * 0.8):
                thrust = 0.6
                takeoff_timer = time.time()
            elif(current_altitude > aTargetAltitude * 1.2):
                thrust = 0.4
                takeoff_timer = time.time()
            else:
                thrust = 0.5

            if(time.time() - takeoff_timer > 2):
                log_msg += '\n--Reached target altitude'
                takeoff_step+=1

        elif(takeoff_step == 3):
            thrust = 0.5
            if (time.time() - takeoff_timer > 5):
                takeoff_done = True
    else:
        if(time.time() - arm_timer>2):
            pix.armed = True
            log_msg += '\n--Arm!'
            arm_timer = time.time()
            takeoff_timer = time.time()

    log_msg += ('\n--------takeoff' +
                '\nARMED: ' + str(pix.armed) +
                '\nTAKEOFF_DONE: ' + str(takeoff_done) +
                '\nAltitude: %f  Desired: %f' % (current_altitude, aTargetAltitude) +
                '\nSTEP: ' + str(takeoff_step) +
                '\n--------')

    return takeoff_done,thrust
def land():
    current_altitude = pix.rangefinder.distance
    thrust = 0.4
    land_finish = False
    global landing_timer

    if(current_altitude >= 0.1):
        landing_timer = time.time()

    if(current_altitude<=0.1 and time.time() - landing_timer > 5):
        land_finish = True

    return land_finish,thrust

#camera
def get_frame():
    global ret, frame
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    '''
    h,  w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    '''
    
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

#tag locolization
def get_tag_position(tag):
    ID = tag.tag_id
    return_data = (0,0)
    ret = False
    PX,PY,PZ = 0,0,0

    csvfile = open('map.csv', newline='')
    POINTS = csv.DictReader(csvfile)

    for POINT in POINTS:
        if int(POINT['ID']) == ID:
            PX = int(POINT['posX'])
            PY = int(POINT['posY'])
            PZ = int(POINT['posZ'])
            ret = True

    return ret,PX,PY,PZ
def tag_direction(tag):
    LT = tag.corners[0]
    RT = tag.corners[1]
    RB = tag.corners[2]
    LB = tag.corners[3]
    Direction = 0

    heading_vector = (LT[0] - LB[0], LT[1] - LB[1])
    #print(heading_vector)
    if(heading_vector[0]>=0 ):
            Direction = 90+math.degrees(math.atan(heading_vector[1] / heading_vector[0]))
    elif(heading_vector[0]<0):
            Direction = -90+math.degrees(math.atan(heading_vector[1] / heading_vector[0]))

    return Direction
def get_rel_position(tag):

    global body_x,body_y
    body_y = math.degrees(pix.attitude.pitch)
    body_x = math.degrees(pix.attitude.roll)

    angle_x = 62.2 * (tag.center[0]/W - 0.5) + (body_x)
    angle_y = 48.8 * (tag.center[1]/H - 0.5) - (body_y)

    tag_1 = int(
        math.sqrt((tag.corners[0][0] - tag.corners[1][0]) ** 2 + (tag.corners[0][1] - tag.corners[1][1]) ** 2))
    tag_2 = int(
        math.sqrt((tag.corners[1][0] - tag.corners[2][0]) ** 2 + (tag.corners[1][1] - tag.corners[2][1]) ** 2))
    tag_3 = int(
        math.sqrt((tag.corners[2][0] - tag.corners[3][0]) ** 2 + (tag.corners[2][1] - tag.corners[3][1]) ** 2))
    tag_4 = int(
        math.sqrt((tag.corners[3][0] - tag.corners[0][0]) ** 2 + (tag.corners[3][1] - tag.corners[0][1]) ** 2))

    tag_lengh = (tag_1 + tag_2 + tag_3 + tag_4)/4
    ZL = 2454.6*tag_lengh**(-0.971)

    angle_z = tag_direction(tag)

    TAG_X = (math.cos(math.radians(angle_z))*math.tan(math.radians(angle_x)) +
             math.sin(math.radians(angle_z))*math.tan(math.radians(angle_y)))* ZL
    TAG_Y = (-math.sin(math.radians(angle_z))*math.tan(math.radians(angle_x)) +
             math.cos(math.radians(angle_z))*math.tan(math.radians(angle_y)))* ZL
    TAG_Z = ZL

    return TAG_X,TAG_Y,TAG_Z
def get_abs_position(tag):

    TAG_X,TAG_Y,TAG_Z = get_rel_position(tag)
    ret,GLOBAL_X,GLOBAL_Y,GLOBAL_Z = get_tag_position(tag)
    facing = tag_direction(tag)

    abs_X = GLOBAL_X - TAG_X
    abs_Y = GLOBAL_Y + TAG_Y
    abs_Z = GLOBAL_Z - TAG_Z

    return abs_X,abs_Y,abs_Z,facing

# body control
def mean_position(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)

    XV_TUPLE = []
    YV_TUPLE = []
    ZV_TUPLE = []
    ANGV_TUPLE = []
    ret = False

    for tag in tags:
        XV, YV, ZV, ANGV = get_abs_position(tag)
        text = str(int(float(XV))) +','+ str(int(float(YV)))
        cv2.putText(frame, text, tuple(tag.corners[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1,
                    cv2.LINE_AA)

        XV_TUPLE.append(XV)
        YV_TUPLE.append(YV)
        ZV_TUPLE.append(ZV)
        ANGV_TUPLE.append(ANGV)
        ret = True

    XV_MEAN,YV_MEAN,ZV_MEAN,ANGV_MEAN = 0,0,0,0

    if(ret):

        for i in XV_TUPLE:
            XV_MEAN += i
        XV_MEAN = XV_MEAN / len(XV_TUPLE)

        for i in YV_TUPLE:
            YV_MEAN += i
        YV_MEAN = YV_MEAN / len(YV_TUPLE)

        for i in ZV_TUPLE:
            ZV_MEAN += i
        ZV_MEAN = ZV_MEAN / len(ZV_TUPLE)

        for i in ANGV_TUPLE:
            ANGV_MEAN += i
        ANGV_MEAN = ANGV_MEAN / len(ANGV_TUPLE)

    log = '\n' + str(XV_MEAN)+ ',' + str(YV_MEAN) + ','+ str(ZV_MEAN)+','+str(ANGV_MEAN)
    log_file.write(log)
    return ret,XV_MEAN,YV_MEAN,ZV_MEAN,ANGV_MEAN
def pos_control(XW,YW,ZW,AW,XV, YV, ZV, AV):

    status = False
    dx = int(XW) - XV
    dy = int(YW) - YV
    dz = int(ZW) - ZV
    da = int(AW) - AV

    global log_msg
    log_msg += ('\n----pos-control' +
                '\n %f %f %f %f ' %(dx,dy,dz,da))

    if(dz > 10):
        thrust = 0.6
    elif(dz < -10):
        thrust= 0.4
    else:
        thrust = 0.5

    if(da>5):
        yaw_rate = -5
    elif(da<-5):
        yaw_rate = 5
    else:
        yaw_rate = 0

    if(dx<-10):
        roll_rate = 2
    elif(dx>10):
        roll_rate = -2
    else:
        roll_rate = 0

    if(dy<-10):
        pitch_rate = 2
    elif(dy > 10):
        pitch_rate = -2
    else:
        pitch_rate = 0


    if(thrust == 0.5 and yaw_rate==0 and roll_rate==0 and pitch_rate==0 ):
        status = True

    return status,pitch_rate,roll_rate,yaw_rate,thrust

#task manager
def task_manager(frame):

    global task_start,task_ID,REF_HOME,TASK_TIMER,TASK_FINISH,send_timer
    pitch_angle, roll_angle, yaw_rate, thrust = 0,0,0,0.5

    target,TYPE,TASK,DURATION = None,None,None,None
    csvfile = open('task.csv', newline='')
    WAYPOINTS = list(csv.DictReader(csvfile))

    if(task_start == False):
        TASK_TIMER = time.time()

    global XV,YV,ZV,AV
    tag_avaliable, XV_N, YV_N, ZV_N, AV_N = mean_position(frame)
    if(tag_avaliable):
        XV = XV*0.8 + XV_N*0.2
        YV = YV*0.8 + YV_N*0.2
        ZV = ZV*0.8 + ZV_N*0.2
        AV = AV*0.8 + AV_N*0.2
    
    XW, YW, ZW, AW = XV, YV, ZV, AV

    for WAYPOINT in WAYPOINTS:
        if int(WAYPOINT['ID']) == task_ID:
            XW,YW,ZW,AW = WAYPOINT['XW'], WAYPOINT['YW'], WAYPOINT['ZW'], WAYPOINT['AW']
            TYPE = WAYPOINT['TYPE']
            TASK = WAYPOINT['TASK']
            DURATION = WAYPOINT['DURATION']
            task_start = True

    global log_msg
    log_msg += ('\n----task_manager----' +
                '\ntask_start: ' + str(task_start) +
                '\nTYPE: ' + str(TYPE) +
                '\nTASK: ' + str(TASK) +
                '\nTIMER: ' + str(time.time() - TASK_TIMER) +
                '\nDURATION: ' + str(DURATION) +
                '\nID: ' + str(task_ID) +
                '\nTarget: ' + XW + ' , '+ YW + ' , '+ ZW + ' , ' + AW +
                '\nPOSITION: ' + str(round(XV,2)) + ' , '+ str(round(YV,2)) + ' , '+ str(round(ZV,2))+ ' , ' + str(round(AV,2)) +
                '\nTag_avaliable: ' + str(tag_avaliable) +
                '\n----task_detail----' )

    if(task_start):
        if TYPE == 'HOME':

            if (TASK == 'SETHOME'):
                TASK_FINISH = True

            elif (TASK == 'TAKEOFF'):

                _, pitch_angle, roll_angle, _, _ = pos_control(XW, YW, ZW, AW, XV, YV, ZV, AV)
                takeoff_alt = float(int(ZW) / 100)
                ret,thrust = takeoff(takeoff_alt)

                if(ret):
                    TASK_FINISH = True

        elif TYPE == 'WAYPOINT':
            ret,pitch_angle, roll_angle, yaw_rate, thrust = pos_control(XW,YW,ZW,AW,XV,YV,ZV,AV)
            if(ret):
                if (TASK == 'TAKE_PHOTO'):
                    if (not TASK_FINISH):
                        _,pic = camera.read()
                        cv2.imwrite('test.png',pic)
                        TASK_FINISH = True

        elif TYPE == 'LANDPOINT':

            ret, pitch_angle, roll_angle, yaw_rate, thrust = pos_control(XW,YW,ZW,AW,XV,YV,ZV,AV)

            if (ret):
                land_finish ,thrust = land()
                if land_finish:
                    TASK_FINISH = True
                    pix.armed = False

        if time.time() - TASK_TIMER >= int(DURATION) and TASK_FINISH:
            task_ID += 1
            TASK_TIMER = time.time()
            TASK_FINISH = False

    else:
        task_ID = 0
        TASK_FINISH = False
        TASK_TIMER = time.time()

    log_msg += "\npitch: %f roll: %f  yaw: %f thrust: %f " % (pitch_angle, roll_angle, yaw_rate, thrust)

    if time.time()-send_timer >= 0.1:
        send_attitude_target(roll_angle=roll_angle,pitch_angle=pitch_angle,yaw_rate=yaw_rate,thrust=thrust)
        send_timer = time.time()
        #pix.flush()

    return task_all_done

#main program
print("system startup")
while(1):
    get_frame()
    log_msg = ''
    log_msg += '------loop------'

    if(not test_mode):
        if(pix.mode == dronekit.VehicleMode("GUIDED")):
            mission_start = True
        else:
            mission_start = False
        log_msg += '\nmission_start: '+str(mission_start)
    else:
        log_msg += '\nmission_start: ' + 'test_mode'
        mission_start = True
    DIR = 0

    #mission start
    if(mission_start):
        log_msg += '\nControlMode: Auto'
        task_manager(frame)
        '''
        try:
            for i in range(1,9,1):
                if abs(failsafe_channels[str(i)] - pix.channels[str(i)])>10:
                    FAILSAFE = True
        except Exception as err:
            pass
        '''
    else:
        #failsafe_channels = pix.channels.copy()
        log_msg += '\nControlMode: Manual'

    ##global log
    if(time.time() - log_timer > 0.1):
        print(log_msg)
        try:
            #setup UDP
            client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            client.sendto(log_msg.encode(),(HOST,PORT))
        except Exception as err:
            log_msg += '\n----Host connection lost----'

        log_timer = time.time()
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(fps)
    cv2.imshow('FRAME',frame)
    K = cv2.waitKey(1)
    if(K == 27):
        break

#safety mode change
#pix.mode = dronekit.VehicleMode("LAND")

#system release
camera_release()
pix.flush()
cv2.destroyAllWindows()
GPIO.cleanup()
log_file.close()