"""nelsonA4_controller controller."""

#Name: Nelson Lopez
#PID: 730157511

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor

MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
TIME_STEP = 64
Min_dist = 0.50
Max_dist = 5.0

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)


sensors = []
sensorNames = [
    'so0', 'so1', 'so2', 'so3', 'so4', 'so5', 'so6', 'so7',
    'so8', 'so9', 'so10', 'so11', 'so12', 'so13', 'so14', 'so15' 
]

for i in range(MAX_SENSOR_NUMBER):
    sensors.append(robot.getDistanceSensor(sensorNames[i]))
    sensors[i].enable(timestep)

leftMotor = robot.getMotor('left wheel')
rightMotor = robot.getMotor('right wheel')

# leftMotor.setPosition(float(100.0))
# rightMotor.setPosition(float(100.0))

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


endTime = False
isTurning = False
isFollowing = False
postTurn = False
endFlag = False
hasFollowed = False

stepCount = 0
step2 = 0
endCount = 0
step3 = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
 # Read the sensors:
    sensor_values = []
    distance_values = []
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i in range(MAX_SENSOR_NUMBER):
        sensor_values.append(sensors[i].getValue())
        sensorVal = sensor_values[i] / 1024
        distance = 5.0 * (1.0 - (sensorVal))
        distance_values.append(distance)#MAX_SENSOR_VALUE=1024
        #print("Sensor " + str(sensorNames[i] + ": " + str(sensorVal)))
        print("Distance " + str(sensorNames[i]) + ": "+ str(distance_values[i]))


    # Process sensor data here.
    # print(sensor_values[0])
    # print(distance_values[0])
    # Enter here functions to send actuator commands, like:
    left_front_obstacle = distance_values[0] < Min_dist or distance_values[1] < Min_dist or distance_values[2] < Min_dist
    right__front_obstacle = distance_values[5] < Min_dist or distance_values[6] < Min_dist or distance_values[7] < Min_dist

    right_rear_obstacle = distance_values[13] < Min_dist or distance_values[14] < Min_dist or distance_values[15] < Min_dist
    left_rear_obstacle = distance_values[8] < Min_dist or distance_values[9] < Min_dist or distance_values[10] < Min_dist



    front_obstacle = distance_values[3] < Min_dist or distance_values[4] < Min_dist
    rear_obstacle = distance_values[11] < Min_dist or distance_values[12] < Min_dist

    #if s0 and s15 are the same, the robot turned 90 degrees
    # if 7 and 8 are the same, the robot turned the wrong way
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    #degrees90 = False
    #if (distance_values[0] + distance_values[15])/2 < Min_dist:
        #degrees90 = True
    if endFlag:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break


    #print("isTurning: " + str(isTurning))
    print("isFollowing: " + str(isFollowing))
    #print("postTurn: " + str(postTurn))

    if front_obstacle and isTurning == False or distance_values[1] < Min_dist and isTurning == False:
        isTurning = True

    if isTurning:
        if distance_values[3] <= Min_dist or distance_values[4] <= Min_dist:
            leftSpeed  += 0.5 * MAX_SPEED
            rightSpeed -= 0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            if hasFollowed != True:
                stepCount+=1
            continue
        if distance_values[1] <= Min_dist or distance_values[2] <= Min_dist:
            leftSpeed  += 0.5 * MAX_SPEED
            rightSpeed -= 0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            if hasFollowed != True:
                stepCount+=1
            continue
            
        else:
            isTurning = False
            isFollowing = True
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        if hasFollowed != True:
            stepCount+=1
        continue


    elif isFollowing:
        hasFollowed = True
        if distance_values[15] > distance_values[0] or distance_values[0] < Min_dist and distance_values[1] < Min_dist: #straight
            leftSpeed  += 0.2 * MAX_SPEED
            rightSpeed -= 0.2 * MAX_SPEED
        else: 
            leftSpeed  -= 0.2 * MAX_SPEED
            rightSpeed += 0.2 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

        #or distance_values[0] > Max_dist and distance_values[15]
        if right_rear_obstacle != True:
            isFollowing = False
            postTurn = True

        continue

    
    elif postTurn:
        if left_front_obstacle:
            postTurn = False
            isFollowing = True
            continue

        if step2 <= stepCount:
            if step3 > 20:
                leftSpeed  -= 0.5 * MAX_SPEED
                rightSpeed += 0.5 * MAX_SPEED
                step2+=1
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
                continue
            else: 
                step3+=1
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
                continue
        else:
            postTurn = False
            endTime = True
        continue

    elif endTime:
        if endCount == 258:
            endFlag = True
        endCount+= 1
        #print(endCount)


    """if front_obstacle:
        leftSpeed  += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED

    if distance_values[1] < Min_dist or distance_values[14] < Min_dist:
        leftSpeed  += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
    elif distance_values[0] < Min_dist or distance_values[15] < Min_dist:
        if degrees90 != True:
            leftSpeed  += 0.5 * MAX_SPEED
            rightSpeed -= 0.5 * MAX_SPEED
        else:
            if distance_values[15] > distance_values[0]:#Perpendi
                leftSpeed  += 0.2 * MAX_SPEED
                rightSpeed -= 0.2 * MAX_SPEED
            elif distance_values[15] < distance_values[0]:
                leftSpeed  -= 0.2 * MAX_SPEED
                rightSpeed += 0.2 * MAX_SPEED
            elif distance_values[15] == Max_dist or distance_values[0] == Max_dist:
                leftSpeed  -= 0.2 * MAX_SPEED
                rightSpeed += 0.2 * MAX_SPEED
            else:
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED """


        
    # modify speeds according to obstacles
    #if left_obstacle:
        # turn right
        #leftSpeed  += 0.5 * MAX_SPEED
        #rightSpeed -= 0.5 * MAX_SPEED
    #elif right_obstacle:
        # turn left
        #leftSpeed  -= 0.5 * MAX_SPEED
        #rightSpeed += 0.5 * MAX_SPEED
    # write actuators inputs
    #leftMotor.setVelocity(leftSpeed)
    #rightMotor.setVelocity(rightSpeed)
    #  motor.setPosition(10.0)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass

# Enter here exit cleanup code.
