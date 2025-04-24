import serial
import time

if __name__ == '__main__':
    port = '/dev/ttyUSB0'
    baudrate = 921600
    ser = serial.Serial('/dev/ttyUSB0', baudrate=baudrate)
    ser.close()
    ser.open()

    i = 0
    while True:
        if ser.in_waiting > 0:
            # print(f'{i}, {time.time()}')
            data = ser.read(ser.in_waiting).decode()
            print(f'{data}')
            i+=1