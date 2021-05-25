
import serial, time

ser = serial.Serial('/dev/rfcomm0', 115200)
print(f'Opened serial connection: {ser}')
'''
print('Waiting for handshake (\"Finished!\")')
lastWord = list(ser.read(9).decode('ascii'))
while True:
	lastWord[0] = lastWord[1]
	lastWord[1] = lastWord[2]
	lastWord[2] = lastWord[3]
	lastWord[3] = lastWord[4]
	lastWord[4] = lastWord[5]
	lastWord[5] = lastWord[6]
	lastWord[6] = lastWord[7]
	lastWord[7] = lastWord[8]
	lastWord[8] = ser.read(1).decode('ascii')
	print("".join(lastWord))
	if("".join(lastWord) == "Finished!"):
		break
'''
ser.write(b'kbalance')
time.sleep(3)
ser.write(b'kzero')
time.sleep(3)
ser.write(b'kbalance')
time.sleep(1)
ser.close()

'''
# read from Arduino
print('start...')
time.sleep(5)
print('zeroing...')
ser.write(b'kbalance')
print('zat...')
time.sleep(3)
print('resting...')
ser.write(b'krest')
print('rat.')
'''
