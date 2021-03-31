"""a6_controller controller."""

# Name: Nelson Lopez
# PID: 730157511

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, PositionSensor, GPS
import math


MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
TIME_STEP = 64
Min_dist = 0.50
Max_dist = 5.0
x2 = 2
y2 = -2

# Flags for Finite State
initTurnCount = 0
hasTurned = False
hit = False


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

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

lastX = 0
lastY = 0
lastAng = 0;
while robot.step(timestep) != -1:
    # Read the sensors:
    sensor_values = []
    distance_values = []
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i in range(MAX_SENSOR_NUMBER):
        sensor_values.append(sensors[i].getValue())
        # MAX_SENSOR_VALUE=1024
        distance_values.append(5.0 * (1.0 - (sensor_values[i] / 1024)))
        #print("Distance " + str(sensorNames[i]) + ": " + str(distance_values[i]))

    left_front_obstacle = distance_values[0] < Min_dist or distance_values[1] < Min_dist or distance_values[2] < Min_dist
    right__front_obstacle = distance_values[5] < Min_dist or distance_values[6] < Min_dist or distance_values[7] < Min_dist

    right_rear_obstacle = distance_values[13] < Min_dist or distance_values[14] < Min_dist or distance_values[15] < Min_dist
    left_rear_obstacle = distance_values[8] < Min_dist or distance_values[9] < Min_dist or distance_values[10] < Min_dist

    front_obstacle = distance_values[3] < Min_dist and distance_values[4] < Min_dist
    rear_obstacle = distance_values[11] < Min_dist or distance_values[12] < Min_dist

    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    gps_sensor = robot.getGPS('gps')
    gps_sensor.enable(timestep)
    # print("gps", gps_sensor.getValues())

    val0 = gps_sensor.getValues()[0]
    val2 = gps_sensor.getValues()[2]
    # ('gps', [1.923787815212497, 0.0943611335373209, -2.0002095009366525])
    # ('gps', [1.923458038293817, 0.09436612018097251, -2.0083787455590287])
    # Destination Location
    # ('X: ', -2.98, 'Y: ', 2.989) start location
    # 2, 0.1, -2 is approx on each map
    # M-Line Angle is arctan(5,5) = 45 Degree Line

    x1 = round(val0, 3)
    y1 = round(val2, 3)
    # x1, y2 are current coords

    y = y2 - y1
    x = x2 - x1
    slope = y/x
    # x and  y is difference
    # print("Slope:", slope)
    #print("X: ", x, "Y: ", y)
    rawAng = abs(math.degrees(math.atan2(y, x)))
    ang = round(rawAng, 3)
    # print("Angle: ", ang) #Angle

    if hasTurned is not True:
        #print("iTurn")
        leftSpeed += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
        if initTurnCount > 18:
            hasTurned = True
        initTurnCount += 1
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        pass

    if front_obstacle or left_front_obstacle:
        #print("front or left front Turn")
        leftSpeed += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        continue

    if front_obstacle and right__front_obstacle:
        #print("Turn Front Right")
        leftSpeed -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        continue



    if right__front_obstacle:
        #print("RTurn")
        leftSpeed -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        continue




# Keep on M Line
    if hasTurned and not front_obstacle:
        #print("MLINE")
        if ang > 45:
            # left
            leftSpeed -= 0.2 * MAX_SPEED
            rightSpeed += 0.2 * MAX_SPEED
        else:
            # right
            leftSpeed += 0.2 * MAX_SPEED
            rightSpeed -= 0.2 * MAX_SPEED


    if (abs(x) < 0.25 and abs(y) < 0.25) or (abs(x) < 0.10 or abs(y) < 0.10):  # At Endpoint
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break

    lastX = x
    lastY = y
    lastAng = ang
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)


# Enter here exit cleanup code.
