import serial, time

CLIport = serial.Serial('COM6', 115200, timeout=1)

if CLIport.is_open:
    str_tmp = "sensorStop"
    CLIport.write((str_tmp+'\n').encode())
    print(str_tmp + " sent")
    time.sleep(0.1)
    