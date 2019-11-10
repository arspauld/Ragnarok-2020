import serial
import pprint
import time

# Configure the serial port to be used for upload
ser = serial.Serial()
ser.timeout = 0.02
ser.baudrate = 115200
ser.port = '/dev/cu.usbserial-AD0JLP8I' # os.environ['PLATFORMIO_UPLOAD_PORT']
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_EVEN
ser.stopbits = serial.STOPBITS_ONE
ser.open()

# Open and parse the binary file
upload_bytes = []
checksum_bytes = []
with open('.pio/build/nucleo_l476rg/firmware.bin', 'rb') as file:
    byte = file.read(256)
    while byte:
        upload_bytes.append(byte)
        byte = file.read(256)


# for x in upload_bytes:
#     checksum_bytes.append(~x)
# pprint.pprint(upload_bytes)   



# print(ser)
# ls = [0xff, 0x00, 0x00, 0xff, 0xff]
# for x in ls:
#     ser.write(0b10000000000011)

# Get Bootloader version
print(bytes([0x00,0xff]))
print(ser.write(bytes([0x00])))
print(ser.write(bytes([0xff])))

# while not ser.in_waiting:
#     time.sleep(.1)
time.sleep(3)
print(ser.in_waiting)
ack = ser.read(1)
if ack == 0x79:
    print("Command recieved: Get")
elif ack == 0x1F:
    print("Command not recieved -- aborting")
    exit(0)
else:
    print("Unknown")
    exit(0)

ser.close()

# print(ser)
