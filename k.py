import serial
import time
#create turtlebot3 velocity




ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
# connect another serial port
ser2 = serial.Serial('/dev/ttyACM1', 115200, timeout=0)

# start serial
def start_serial():
    ser.write(b'\r\r')
    time.sleep(1)
    ser.flushInput()

# read ser ser2 msg
def read_msg(msg):
    







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
