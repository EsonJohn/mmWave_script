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

# Wait key to toggle IWRframe/Kinect
sending = False
while True:
    if sending:
        print('\nFrame ' + 'sending' + ', press Enter to ' + 'stop')
        key_input = input('<<')
        # Stop Kinect
        conn_Device.send('stop'.encode())
        print('>>> sensorStop')
        time.sleep(0.01)
        sending = False
    else:
        print('\nFrame ' + 'stopped' + ', press Enter to ' + 'send')
        key_input = input('<<')
        # Start Kinect
        conn_Device.send('start'.encode())
        print('>>> sensorStart')
        time.sleep(0.01)
        sending = True

CLIport.close()
conn_Device.close()