#pymavlink to communicate with pixhawk
from pymavlink import mavutil
#timing
import time
#math operations
import math

def set_parameters(param,value):
    master.mav.param_set_send(
    master.target_system,
    master.target_component,
    param,
    value
    )

def set_mode(modep):
    mode = modep
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system, #target system
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, #custom_mode is the mode we are setting
        mode_id) #mode_id is the mode we are setting it to (previous code)
    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")

def manualControl(x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # pitch for sub, back/forward on joystick [-1000,1000], respectively
        y,  # roll for sub, left/right on joystick [-1000,1000], respectively
        z,  # thrust for sub, slider on joystick [0,1000]
        r,  # yaw for sub, clockwise/counterclockwise on joystick [-1000,1000], respectively
        0)  # buttons


#check this for sure
# def getDepth():
#     #set initial depth variable = 0
#     depth = 0
#     #infinite loop
#     while True:
#         #recv_match() is for capturing messages with particular names or field values
#         #without arguments, it will just look for the next message without specifications
#         #so msg is the variable used to store messages
#         msg = master.recv_match()
#         #if there is not a message,
#         if not msg:
#             # continue to the next iteration of loop, which means check if there is a new message
#             continue
#         #If the message type in message is VFR_HUD,
#         #https://mavlink.io/en/messages/common.html#VFR_HUD
#         #Basically if the message is a metric typically displayed on a HUD for the sub,
#         if msg.get_type() == 'VFR_HUD':
#             #set a variable data to the string version of the msg
#             data = str(msg)
#             #data will come as smth like 11:12:13:14:15:16:17:18:19:20
#             try:
#                 #data will be split into a list, so [11,12,13,14,15,16,17,18,19,20]
#                 data = data.split(":")
#                 #6th item of list will be split by commas so.. [1,6] and then the first item will be returned so 1
#                 #depth = 1 in this example  
#                 depth = data[5].split(",")[0]
#                 print("success1")
#             #when depth can't be split into all of the above, returns as empty
#             except:
#                 print('flop2')
#             #print the depth out
#             print("Current Depth: ", depth)
#         #as soon as the depth is detected, and isn't 0 (which is what the fucntion sets it to)
#         if not depth == 0:
#             print("flop3")
#             #break the infinite loop
#             break
#     #return the detected depth
#     return float(depth)

#check for velocity in Qgroundcontrol
def get_velocity():
    velocity = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                speed = data[2].split(",")[0]
            except:
                print('')

            velocity = float(speed)
        if not velocity == 0:
            break

    return velocity

#use SCALED_PRESSURE2 to read pressure 
def getPressure():
    while True:
        pressure = None
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'SCALED_PRESSURE2':
            data = str(msg)
            try:
                data = data.split(":")
                print(data, "pressure")
                pressure = data[3].split(",")[0]
            except:
                print('')
        return pressure

def getDepth(initial_pressure):
    pressure = initial_pressure - getPressure()
    P = pressure * 100
    g = 9.80665
    p = 1023.6
    depth = P/(p*g) *(-1)
    return depth
    
def maintainDepth(initial_depth):
    angle = initial_depth - getDepth()
    
#check for heading in Qgroundcontrol
def get_heading():
    heading = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                heading = data[3].split(",")[0]
            except:
                print('')

            heading = float(heading)
            break

    return heading

#dependent on getDepth()
def goDepth(depth):
    #set the desired operating mode
    #why STABILIZE?
    #https://ardupilot.org/dev/docs/apmcopter-programming-advanced.html
    mode = 'STABILIZE'
    # get the mode id from the mode_mapping dictionary
    # The code master.mode_mapping()[mode] is accessing a dictionary of mode mappings 
    # stored in the "master" object and returning the value associated with the key mode.
    #https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
    mode_id = master.mode_mapping()[mode]
    #set the mode
    #https://mavlink.io/en/messages/common.html#SET_MODE
    master.mav.set_mode_send(
        master.target_system,#target system
        #https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,#custom_mode is the mode we are setting
        mode_id)#mode_id is the mode we are setting it to (previous code)
    
    #set the depth you are currently at
    current_depth = abs(getDepth())

    #if the current depth is greater than the desired depth
    if current_depth > depth:
        #run this 1000000 times (because manual control is one small shift at a time)
        for i in range(1000000):
            #set thrust to 700 (upward)
            manualControl(0,0,700, 0)
            #get the current depth at each small shift
            current_depth = abs(getDepth())
            #if the current depth is less than the desired depth (within 5%)
            if current_depth < depth *0.95:
                #break the loop and stop
                break
        #print the depth reached
        print("REACHED DESIRED Depth: ", getDepth())

        #get mode to DEPTH_HOLD
        mode = 'ALT_HOLD'
        #get the mode id
        mode_id = master.mode_mapping()[mode]
        #set the mode via message
        master.mav.set_mode_send(
            master.target_system,#target
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,#set mode to the one specified
            mode_id) #mode_id that we want to set

    #if the current depth is less than the desired depth
    else:
        for i in range(1000000):
            #set thrust to 300 (downward) *wait why is it 300? and not -300? (diff for z)
            manualControl(0,0,300, 0)
            #rest is self-explanatory
            current_depth = abs(getDepth())

            if current_depth > depth *0.95:
                break
        print("REACHED DESIRED Depth: ", getDepth())
        #self-explanatory
        mode = 'ALT_HOLD'
        mode_id = master.mode_mapping()[mode]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
    
#dependent on get_velocity()
def travel_in_x(xThrottle, distanceTravel):
    #set mode to STABILIZE (prevents too much external movement) KEEPS PITCH AND ROLL STABLE SO WHEN CHANGING XTHROTTLE, MOVES FORWARD!! 
    #would probably work for roll as well  (vertical movement)
    #self-explanatory
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    #print mode
    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>> travel_in_x")
    #start timer
    start = time.time()
    #create an array to store velocity values
    velocity_array = []
    #distance is initially 0
    distance = 0
    for i in range(10000000):
        #set pitch to xThrottle (forward), no thrust (0-1000) because xthrottle is forward due to STABILIZE (no changes in pitch)
        #notice no iteration
        manualControl(xThrottle, 0, 500, 0)
        #get the elapsed time
        elapsed = time.time() - start
        #get the velocity at each iteration
        velocity_array.append(get_velocity())
        #get the average velocity
        average_velocity = sum(velocity_array) / len(velocity_array)
        #calculate the distance traveled
        distance = elapsed * average_velocity
        #print the distance
        print("RECORDED DISTANCE: ", distance)
        #if the distance traveled is greater than the desired distance(within 5%)
        if distance > 0.95*distanceTravel:
            break
    #print the distance reached
    print("REACHED DESIRED DISTANCE: ", distance)
    #self-explanatory, hold altitude
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

#dependent on get_heading()
#dependent on get_heading()
def rotateClockwise(degrees):
    #hold altitude and send message
    set_mode("MANUAL")
    print("start heading: " + str(get_heading()))
    
    #the original heading of the vehicle.
    #what is heading? the compass direction in which the craft's nose is pointing
    #https://mavlink.io/en/messages/common.html#ATTITUDE
    start_heading = get_heading()
    new_heading = (start_heading + degrees) % 360
    #run this 10000000 times
    for i in range(10000000):
        print(i) # does it only run this loop once?
        #get current heading, usually after small shift in yaw
        current_heading = get_heading()
        #calculate the difference in rotation by degrees
        #rotate clockwise, no thrust
        manualControl(0, 0, 500, 200)
        print("current heading: " + str(get_heading()))
        # if the desired degrees rotated
        # is greater than the desired rotation (within 4%), stop
        if current_heading > 0.9 * new_heading and  current_heading < 1.1 * new_heading:
            break
    #print the rotation reached
        print("ROTATED: ", current_heading)
    #hold altitude
    manualControl(0, 0, 500, 0)
    mode = 'MANUAL'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    
def rotateCounterClockwise(degrees):
    set_mode("MANUAL")
    print("start heading: " + str(get_heading()))

    #the original heading of the vehicle.
    #what is heading? the compass direction in which the craft's nose is pointing
    #https://mavlink.io/en/messages/common.html#ATTITUDE
    start_heading = get_heading()
    new_heading = (start_heading - degrees) % 360
    #run this 10000000 times
    for i in range(10000000):
        print(i) # does it only run this loop once?
        #get current heading, usually after small shift in yaw
        current_heading = get_heading()
        #calculate the difference in rotation by degrees
        #rotate clockwise, no thrust
        manualControl(0, 0, 500, -200)
        print("current heading: " + str(get_heading()))
        # if the desired degrees rotated
        # is greater than the desired rotation (within 4%), stop
        if current_heading > 0.9 * new_heading and current_heading < 1.1 * new_heading:
            break
    #print the rotation reached
        print("ROTATED: ", current_heading)
    #hold altitude
    manualControl(0, 0, 500, 0)
    mode = 'MANUAL'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    
def maintainHeading(heading):
    #hold altitude
    mode = 'MANUAL'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    #get the current heading
    start_heading = get_heading()
    #calculate the difference in headings
    angle = float(start_heading - heading) #derek you put abs bruh it cant be negative we changed it to float were lowkey goated
    #if the angle is negative, rotate clockwise (increase heading)
    if angle < 0: 
        rotateClockwise(abs(angle))
    #if the angle is positive, rotate counter-clockwise (decrease heading)
    else: 
        rotateCounterClockwise(abs(angle))
    
        
# master = mavutil.mavlink_connection('192.168.2.2', baud=57600) #establish connection with pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600) #establish connection with pixhawk 


print("<<<<<<WAITING FOR CONNECTION>>>>>>")
print("Hi")
master.wait_heartbeat() #ensure connection is valid
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")

# set_parameters("RC_OVERRIDE_TIME", 0.2)

master.arducopter_arm()

# for i in range(0,40):
#     print(getDepth(1007))
#     time.sleep(0.5)

# time.sleep(40)

beginning_heading = get_heading()
print ("beginning heading: " + beginning_heading)

print("DESCENDINGGGGG")

manualControl(0,0,800,0)
time.sleep(2.5)

manualControl(0,0,500,0)

maintainHeading(beginning_heading)

print("going forward")

manualControl(750,0,500,0)


time.sleep(10)
manualControl(0,0,500,0)



#set_parameters(RC_OVERRIDE_TIME, 0.1)


# print("rotating")
# rotateClockwise(180)
# time.sleep(1.0)
# rotateClockwise(180)
# time.sleep(1.0)
# rotateClockwise(180)
# time.sleep(1.0)
# rotateClockwise(180)
# time.sleep(1.0)

# print("DESCENDINGGGGGGGGG") # also this goes straight on rc_override_time = 2.0
# for i in range(0,15): 
#     manualControl(0,0,900,0)

# for i in range(0,20): #7 is good for gate, with rc_override=0.2
#     for i in range(0,5):
#         manualControl(750,0,500,0)
    
    #print(getDepth(1007))

# print("DESCENDINGGGGGGGGG AGAIN")
# for i in range(0,25): 
#     manualControl(0,0,850,0)

# for i in range(0,9):
#     for i in range(0,5):
#         manualControl(750,0,500,0)
#     time.sleep(2.0)
#     print(getDepth())

# # for i in range(0,40):
# #     manualControl(0,0,700,0)
