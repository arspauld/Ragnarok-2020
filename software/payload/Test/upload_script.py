import serial
import time
import pprint

# Configure the serial port to be used for upload
ser = serial.Serial()
ser.timeout = 0.02
ser.baudrate = 9600
ser.port = '/dev/cu.usbserial-AD0JLP8I' # os.environ['PLATFORMIO_UPLOAD_PORT']
# ser.open()

# Open the file to be parsed and uploaded
upload_bytes = []
with open('.pio/build/nucleo_l476rg/firmware.bin', 'rb') as file:
    byte = file.read(256)
    while byte:
        upload_bytes.append(byte)
        byte = file.read(256)

pprint.pprint(upload_bytes)   



# print(ser)
# ls = [0xff, 0x00, 0x00, 0xff, 0xff]
# for x in ls:
#     ser.write(0b10000000000011)

# ser.close()

# print(ser)
