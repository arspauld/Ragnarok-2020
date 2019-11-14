import serial
#import pprint
import time
#from tqdm import tqdm

# Configure the serial port to be used for upload
ser = serial.Serial()
ser.timeout = 0.02
ser.baudrate = 115200
# ser.port = '/dev/cu.usbserial-AD0JLP8I' # os.environ['PLATFORMIO_UPLOAD_PORT']
ser.port = 'COM13'
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
        pass
    elif acknowledge:
        print(ack_text)
    elif acknowledge == nack:
        print(nack_text)
        exit(0)
    else:
        print("Unknown -- aborting")
        exit(0)

######## Write
def write(data, start_address=0x08000000):
    acknowledge(cmd=0x31, chk=0xCE, ack_text='', nack_text="ERROR: Command not found -- aborting")
    addr = [start_address >> 24 & 0xFF, start_address >> 16 & 0xFF, start_address >> 8 & 0xFF, start_address & 0xFF]
    addr.append(checksum(addr))
    data = [len(data)-1] + data
    data.append(checksum(data))
    # print(data)

    # Send address and wait for ack
    # print(bytes(addr))
    ser.write(bytes(addr))
    while not ser.in_waiting:
        time.sleep(0.01)
        # print('sleeping')
    ack = int.from_bytes(ser.read(1), byteorder='big')
    if ack != 0x79:
        print('ERROR: Invalid address -- aborting')
        exit(0)
    
    # Send number of bytes, data, and checksum
    # print(len(data))
    # print(data)
    # Error is likely in this area, needs to be [0-255]
    ser.write(bytes(data[0:256]))
    ser.write(bytes(data[256:258]))

    # Await acknowledge at end of write
    while not ser.in_waiting:
        time.sleep(0.01)
    ack = int.from_bytes(ser.read(1), byteorder='big')
    if ack != 0x79:
        print('ERROR: Invalid write -- aborting')
        exit(0)
    else:
        return start_address + len(data) - 2
        

######## Read
def read(start_address = 0x08000000):
    # Recieve initial command ack
    acknowledge(cmd=0x11, chk=0xEE, ack_text='', nack_text='ERROR: Command not found -- aborting')

    # Parse the start address into four bytes
    addr = [start_address >> 24 & 0xFF, start_address >> 16 & 0xFF, start_address >> 8 & 0xFF, start_address & 0xFF]
    addr.append(checksum(addr))

    # Write the desired address and await ack
    ser.write(bytes(addr))
    while not ser.in_waiting:
        time.sleep(0.1)
    ack = int.from_bytes(ser.read(1), byteorder='big')
    if ack != 0x79:
        print('ERROR: Invalid address -- aborting')
        exit(0)
    
    # Write the number of bytes to recieve and await ack
    ser.write(bytes([255, 0]))
    while not ser.in_waiting:
        time.sleep(0.1)
    ack = int.from_bytes(ser.read(1), byteorder='big')
    if ack != 0x79:
        print('ERROR: Invalid number of bytes -- aborting')
        exit(0)

    # Read in the bytes as a list of ints and return
    dat = []
    for i in range(256):
        dat.append(int.from_bytes(ser.read(1), byteorder='big'))
    
    return [dat, start_address+len(dat)]

    

### List Checksum
def checksum(data):
    chk = 0
    for x in data:
        chk ^= x
    return chk

# Open and parse the binary file
upload_bytes = []
with open('.pio/build/nucleo_l476rg/firmware.bin', 'rb') as file:
    # file.read(0x00010000)
    bytes_list = file.read(256)
    while bytes_list:
        x=[]
        for byte in bytes_list:
            x.append(byte)
        upload_bytes.append(x)
        bytes_list = file.read(256)

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

### Erase previous memory
print('\nErasing flash\n--------------------------------')
acknowledge(cmd=0x44, chk=0xBB, ack_text='Command recieved: Erase', nack_text='ERROR: Command not recieved -- aborting')

# Send global erase
ser.write(bytes([0xff, 0xff, 0x00]))
while not ser.in_waiting:
    time.sleep(0.1)
ack = int.from_bytes(ser.read(1), byteorder='big')
if ack == 0x79:
    print('\n\nFlash erased\n--------------------------------')
elif ack == 0x1F:
    print('\nERROR: NACK recieved -- aborting')
    exit(0)
else:
    print('ERROR: Unknown -- aborting')
    exit(0)

### Write new code to FLASH
print('\nUploading binary\n--------------------------------')
start_address = 0x08000000
i=0
for x in upload_bytes:#tqdm(upload_bytes, desc='Write', unit='Pages'):
    # print(hex(start_address))
    start_address = write(data=x, start_address=start_address)
    i+=1
    print("\rProgress: %.2f%%, %d/%d writes" % (i/len(upload_bytes)*100, i, len(upload_bytes)), end="")

### Verify code on the FLASH
print('\n\nVerifying upload\n--------------------------------')
start_address = 0x08000000
read_data = []
for i in range(len(upload_bytes)):
    temp = read(start_address)
    read_data.append(temp[0])
    start_address = temp[1]
    print("\rProgress: %.2f%%, %d/%d reads" % ((i+1)/len(upload_bytes)*100, i+1, len(upload_bytes)), end="")
print()

if read_data[-2] != upload_bytes[-2]:
    print('-- Unknown Error: Dumping --')
    for i in range(len(upload_bytes)):
        print(read_data[i])
        print(upload_bytes[i])
        print()
else:
    print('-- Flash verified --')

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
    print('\n\nJumping to user code\n--------------------------------')
elif ack == 0x1F:
    print('\nERROR: NACK recieved -- aborting')
    exit(0)
else:
    print('ERROR: Unknown -- aborting')
    exit(0)

# Close the serial port before exiting the program
ser.close()
