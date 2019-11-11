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
while not ser.is_open:
    ser.open()

######## Acknowledge
def acknowledge(ack_text, nack_text, cmd=0x00, chk=0xff, ack=0x79, nack=0x1f):
    if cmd == 0x7f: 
        ser.write(bytes([cmd]))
    else: 
        ser.write(bytes([cmd, chk]))
    while not ser.in_waiting:
        time.sleep(0.01)
    acknowledge = int.from_bytes(ser.read(1), byteorder='big')
    if acknowledge == ack and ack_text != '':
        print(ack_text)
    elif acknowledge == nack:
        print(nack_text)
        exit(0)
    else:
        print("Unknown -- aborting")
        exit(0)

######## Write
def write(data, start_address=0x08000000):
    acknowledge(cmd=0x31, chk=0xCE, ack_text='', nack_text="ERROR: Command not found-- aborting")
    addr = [start_address >> 6 & 0xFF, start_address >> 4 & 0xFF, start_address >> 2 & 0xFF, start_address & 0xFF]
    addr.append(checksum(addr))
    data.append(checksum(data))

    # Send address and wait for ack
    ser.write(addr)
    while not ser.in_waiting:
        time.sleep(0.01)
    ack = int.from_bytes(ser.read(1), byteorder='big')
    if ack != 0x79:
        print('ERROR: Address invalid')
        exit(0)
    
    # Send number of bytes, data, and checksum
    data = [len(data)-1] + data
    data.append(checksum(data))
    ser.write(data)

    # Await acknowledge at end of write
    while not ser.in_waiting:
        time.sleep(0.01)
    ack = int.from_bytes(ser.read(1), byteorder='big')
    if ack != 0x79:
        print('ERROR: Invalid write -- aborting')
        exit(0)
    else:
        return start_address + len(data)

    

### List Checksum
def checksum(data):
    chk = 0
    for x in data:
        chk ^= x
    return chk

# Open and parse the binary file
upload_bytes = []
with open('.pio/build/nucleo_l476rg/firmware.bin', 'rb') as file:
    bytes_list = file.read(256)
    while bytes_list:
        x=[]
        for byte in bytes_list:
            x.append(byte)
        upload_bytes.append(x)
        bytes_list = file.read(256)
    print(upload_bytes)

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
acknowledge(ack_text="Command recieved: Get",nack_text="ERROR: Command not recieved -- aborting")
print("Waiting to recieve", ser.in_waiting, "bytes")

# Read in all bytes
bootloader_version = ser.read(14)

# Confirm bootloader version and that it contains the WRITE command
print("Bootloader version: %x.%x" % (bootloader_version[1]//16, bootloader_version[1] % 16))

# Confirm that bootloader contains WRITE command
if 0x31 in bootloader_version:
    print("Bootloader contains write command")
else:
    print("Write command not found -- aborting")
    exit(0)

### Write new code to FLASH

### Jump to user code after upload
acknowledge(cmd = 0x21, chk=0xDE, ack_text='Command recieved: Go', nack_text="Read protection enabled -- aborting")

# Write the address to jump to
addrs = [0x08, 0x00, 0x00, 0x00]
addrs.append(checksum(addrs))
ser.write(addrs)

# Check to see ack from address
while not ser.in_waiting:
    time.sleep(0.01)
    # print(ser.in_waiting)
ack = int.from_bytes(ser.read(1), byteorder='big')
if ack == 0x79:
    print('Jumping to user code\n--------------------------------\n')
elif ack == 0x1F:
    print('ERROR: NACK recieved -- aborting')
    exit(0)
else:
    print('ERROR: Unknown -- aborting')
    exit(0)

# Close the serial port before exiting the program
ser.close()
