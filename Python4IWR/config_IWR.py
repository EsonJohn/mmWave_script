import serial, time

CLIport = serial.Serial('COM6', 115200, timeout=1)

if CLIport.is_open:
    config = [line.rstrip('\r\n') for line in open('cfg/profile.cfg')]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i + ' sent...')
        time.sleep(0.01)