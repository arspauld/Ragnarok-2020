import serial
import pprint
import time
from tqdm import tqdm

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
# upload_bytes = []
# checksum_bytes = []
# with open('.pio/build/nucleo_l476rg/firmware.bin', 'rb') as file:
#     byte = file.read(256)
#     while byte:
#         upload_bytes.append(byte)
#         byte = file.read(256)

### Initialize bootloader in USARTx
ser.write(bytes([0x7f]))                                    # Send initialization byte
while not ser.in_waiting:                                   # Wait for response from device
    time.sleep(0.01)
ack = int.from_bytes(ser.read(1), byteorder="big")          # Confirm acknowledge from device
if ack == 0x79:
    print("\nBootloader Initialized to USARTx\n--------------------------------\n")
else:
    print("Still not there:", ack)
    # exit(0)

### Get Bootloader version
ser.write(bytes([0x00, 0xff]))                              # Send GET command
while not ser.in_waiting:                                   # Wait until the device has sent data
    time.sleep(.1)

# Confirm that the device has properly recieved the command
ack = int.from_bytes(ser.read(1), byteorder='big')
if ack == 0x79:
    print("Command recieved: Get")
elif ack == 0x1F:
    print("ERROR: Command not recieved -- aborting")
    exit(0)
else:
    print("Unknown")
    exit(0)
print("Waiting to recieve", ser.in_waiting, "bytes")

# Read in all bytes
bootloader_version = ser.read(14)

# for i in tqdm(range(ser.in_waiting), desc="Reading", unit='bytes'):
#     bootloader_version.append(int.from_bytes(ser.read(1), byteorder='big'))

# Confirm bootloader version and that it contains the WRITE command
print("Bootloader version: %x.%x" % (bootloader_version[1]//16, bootloader_version[1] % 16))
# Confirm that bootloader contains WRITE command
if 0x31 in bootloader_version:
    print("Bootloader contains write command")
else:
    print("Write command not found -- aborting")
    exit(0)

### Jump to user code after upload

ser.close()
