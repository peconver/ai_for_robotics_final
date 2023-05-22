import serial
import time
#create turtlebot3 velocity
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# read Odometry and move in a Square based on distance travelled.

import rospy
from nav_msgs.msg import Odometry  # nav_msgs/Odometry.msg
from geometry_msgs.msg import Twist   # geometry_msgs/Twist.msg
from tf.transformations import euler_from_quaternion
from math import radians, pi, sqrt
import numpy as np

count = 0
move = turn = 0.0
yawadd = 0.0
lastyaw = 1.0
data = np.zeros(5)

def get_odom(msg):                  # at ~ 30 Hz
    global count, data
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    turn = msg.twist.twist.angular.z
    data = np.copy((yaw, x, y, turn, msg.header.seq))
    count = count +1

def fixyaw(yaw, turn):    # if yaw is negative, add 2*pi ... if turn + = anti-clockwise
    global yawadd, lastyaw
    if yaw < 0 and lastyaw > 0 and turn >= 0:  # add 2*pi if gone past 180 degrees anti-clockwise: or started with - yaw
        yawadd = yawadd + 2 * pi
        print("Added 2Pi to Yawadd... @ yaw now = %.3f " % (yaw))
    if yaw > 0 and lastyaw < 0 and turn < 0:  # - 2*pi if gone past 180 degrees, clockwise
        yawadd = yawadd - 2 * pi
        print("Subtracted 2Pi from Yawadd... @ yaw now = %.3f " % (yaw))
    lastyaw = yaw
    yaw = yaw + yawadd
    return yaw

rospy.init_node('straight')
subodom = rospy.Subscriber('/odom', Odometry, get_odom, queue_size=1)    # nav_msgs/Odometry
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
r = rospy.Rate(30)
wait = rospy.Rate(3)
move_cmd = Twist()
print("Waiting for odom call back to start...")
while data[4] == 0:         # wait for odom call back to start...
    rospy.sleep(0.1)

# get initial position data ....
print("Starting ......... ")
Startyaw = fixyaw(data[0], data[3])     # (yaw and turn)
Startx = data[1]
Starty = data[2]
print("Yaw %.3f , x %.3f , y %.3f" % (Startyaw, Startx, Starty))
Stepyaw = Startyaw
Stepx = Startx
Stepy = Starty


print("Going Straight")
move_cmd.linear.x = 0.3  # start at .1 m/s ...
move_cmd.angular.z = 0.0;
dist = 0.0
Dgoal = 0.2  # turn 90 degrees
decelD = 0.02  # distance to decelerate
minD = 0.005
maxD = 0.1
    # while dist <= Dgoal:  # move for 0.2 metres...
    #     cmd_vel.publish(move_cmd)
    #     dist = sqrt((data[1] - Stepx)**2 + (data[2] - Stepy)**2)
    #     r.sleep()
    # move_cmd.linear.x = 0.0
    # cmd_vel.publish(move_cmd)   # fully stop
    # wait.sleep()
    # dist = sqrt((data[1] - Stepx) ** 2 + (data[2] - Stepy) ** 2)
    # Stepx = data[1]
    # Stepy = data[2]
    # print("Yaw %.3f , x %.3f , y %.3f , dist %.3f" % (Stepyaw, Stepx, Stepy, dist))

    # print("Turning")
    # move_cmd.linear.x = 0.0
    # move_cmd.angular.z = radians(45);  # 45 deg/s in radians/s
    # rotate = 0.0
    # Rgoal = radians(90)  # turn 90 degrees
    # decelR = radians(15)  # distance to decelerate
    # minR = radians(1)
    # maxR = radians(90)
    # while rotate <= Rgoal:
    #     cmd_vel.publish(move_cmd)
    #     yaw = fixyaw(data[0], data[3])
    #     rotate = yaw - Stepyaw
    #     if rotate > (Rgoal-decelR):    # start decelerating:
    #         speed = 0.7*maxR*(Rgoal - rotate) / Rgoal  # e.g. ~ 10% of maxR or less
    #         if speed > maxR:
    #             speed = maxR
    #         if speed < minR:
    #             speed = minR
    #         move_cmd.angular.z = speed
    #     r.sleep()
    # move_cmd.angular.z = 0.0
    # cmd_vel.publish(move_cmd)   # fully stop
    # wait.sleep()
    # yaw = fixyaw(data[0], data[3])
    # rotate = (yaw - Stepyaw)*180/pi
    # Stepyaw = yaw
    # print("Yaw %.3f , x %.3f , y %.3f , rotate %.2f degrees" % (Stepyaw, Stepx, Stepy, rotate))
    # steps = steps + 1
print("TurtleBot stopped: %d " % (count))

# get final position data ....
wait.sleep()
Endyaw = fixyaw(data[0], data[3])
Endx = data[1]
Endy = data[2]
print("End: Yaw %.3f , x %.3f , y %.3f" % (Endyaw, Endx, Endy))
Difyaw = 180.0*(Endyaw - Startyaw)/pi  - 360.0
Difdist = sqrt((Endx - Startx)**2 + (Endy - Starty)**2)
print("Error in Yaw = %.1f degrees, and in Dist = %.3f metres " % (Difyaw, Difdist))
print("Finished")
# end of program ...




ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
# connect another serial port
ser2 = serial.Serial('/dev/ttyACM1', 115200, timeout=0)






def start_serial():
    ser.write(b'\r\r')
    time.sleep(1)
    ser.flushInput()


def read_msg(msg):
    bl = 0
    ser.flushOutput()
    ser.flushInput()
    msg = msg + "\r" 
    msg=msg.encode()
    ser.write(msg)
    time.sleep(5)
    data_lst = []  # 儲存數據的串列


    

#collect datafrom tag1
    while bl < 3:
        response = ser.readline()
        if response == "".encode():
            bl = bl+1
        time.sleep(0.1)
        if response != "dwm> ".encode() and response.decode().strip() != msg.decode().strip() and response != "".encode():
            # do whatever you want with this response
  

            data = response.decode().strip()
            # delete the blank line
            if data == "":
                continue
            # delete data contains "dwm>"
            if "dwm>" in data:
                continue
            data_lst.append(data)
            # every 10 data save to file
            # if len(data_lst) == 100 :
                
            #     #if data up to 100 then stop

            #     # use for loop to save data to file and delete the blank line
            #     for data in data_lst:
            #         print(data)
            #         with open("data.csv", "a") as f:
            #             f.write(data+ "\n")
            #     data_lst = []
            #     break



            # # use for loop to save data to file and delete the blank line
            for data in data_lst:

                print(data)
                with open("data1.csv", "a") as f:

                    
                    f.write(data+ "\n")

            print(data)  # 在螢幕上輸出數據


            
              

            # print(response.decode().strip())
    ser.flushInput()
# save data to file


#collect datafrom tag2
    while bl < 3:
        response = ser2.readline()
        if response == "".encode():
            bl = bl+1
        time.sleep(0.1)
        if response != "dwm> ".encode() and response.decode().strip() != msg.decode().strip() and response != "".encode():
            # do whatever you want with this response

            data = response.decode().strip()
            # delete the blank line
            if data == "":
                continue
            # delete data contains "dwm>"
            if "dwm>" in data:
                continue
            data_lst.append(data)
            # every 10 data save to file
            if len(data_lst) == 100 :   
                    #if data up to 100 then stop
                    # use for loop to save data to file and delete the blank line
                    for data in data_lst:
                        print(data)
                        with open("data2.csv", "a") as f:
                            f.write(data+ "\n")
                    data_lst = []
                    break
                    
            # use for loop to save data to file and delete the blank line
    ser2.flushInput()
# save data to file






# Start serial and send double \r to enter shell
start_serial()
# read anchors list
# print(start_time)
k=read_msg("lep")
# while True:
#     read_msg("lep")
#     if start_time>1:
#         break
# read unit modes
# print(read_msg("nmg"))
# read accelerometer values
# read_msg("av")
