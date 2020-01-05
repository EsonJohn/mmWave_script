import serial, time, sys

CLIport = serial.Serial('COM6', 115200)
Dataport = serial.Serial('COM5', 921600)
CLIport.close()
CLIport.open()
Dataport.close()
Dataport.open()

str_tmp = "sensorStop"
CLIport.write(str_tmp.encode())
print(str_tmp + " sent")
time.sleep(1)
a = CLIport.readlines()
print(a)



# config = [line.rstrip('\r\n') for line in open('./profile.cfg')]
# for i in config:
#     print(i)
#     CLIport.write(i.encode())
#     time.sleep(1)
#     a = CLIport.read(size=64)
#     print(a)
# time.sleep(3)
# str_stop = "sensorStop"
# CLIport.write(str_stop.encode())