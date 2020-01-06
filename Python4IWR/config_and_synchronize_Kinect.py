import serial, time, socket

# Synchronize with Kinect on socket
addr_ip = '127.0.0.1'
addr_port = 5555
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_Device:
    s_Device.bind((addr_ip, addr_port))
    s_Device.listen()
    print('\nListening on ' + addr_ip + ':' + str(addr_port))
    conn_Device, addr_Kinect = s_Device.accept()
    s_Device.close()
    print('Device connected on ' + str(addr_Kinect))
    device_say = conn_Device.recv(1024)
    print('Device ID: ' + device_say.decode())

# Configure IWR1642 by serial port
serial_port_CLI = 'COM6'
file_cfg = 'cfg/profile.cfg'
print('\nSending ' + file_cfg + ' to IWR1642 on ' + serial_port_CLI)
CLIport = serial.Serial(serial_port_CLI, 115200, timeout=1)
if CLIport.is_open:
    config = [line.rstrip('\r\n') for line in open(file_cfg)]
    for i in config:
        # Skip empty line
        if(i == ''):
            continue
        # Skip comment line
        if(i[0] == '%'):
            continue
        # Stop on sensorStart command
        if (i == 'sensorStart'):
            break
        CLIport.write((i+'\n').encode())
        print('>>> ' + i)
        time.sleep(0.01)

# Wait key to toggle IWRframe/Kinect
sending = False
initial_frame_sent = False
while True:
    if sending:
        print('\nFrame ' + 'sending' + ', press Enter to ' + 'stop')
        key_input = input('<<')
        # Stop Kinect
        conn_Device.send('stop'.encode())
        # Stop IWRframe
        CLIport.write(('sensorStop\n').encode())
        print('>>> sensorStop')
        time.sleep(0.01)
        sending = False
    else:
        print('\nFrame ' + 'stopped' + ', press Enter to ' + 'send')
        key_input = input('<<')
        # Start Kinect
        conn_Device.send('start'.encode())
        # Start IWRframe
        if initial_frame_sent:
            start_cmd = 'sensorStart 0'
        else:
            start_cmd = 'sensorStart'
        CLIport.write((start_cmd + '\n').encode())
        print('>>> ' + start_cmd)
        time.sleep(0.01)
        sending = True
        initial_frame_sent = True

CLIport.close()
conn_Device.close()