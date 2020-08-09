import serial
import time
from threading import Thread
from threading import Lock

serial_mutex = Lock()	# This mutex guards the left and right track speeds acessed by sepperate threads
leftTrackVel = 0		#speed of left track [-1, 1]
rightTrackVel = 0		#speed of right track [-1, 1]
driverReturnValue = 0	#Return value from arduino, used for debug
stop = False


# Sends track speeds to arduino driver, prints recieved values
def arduino_serial(ser):
	while not stop:
		time.sleep(0.1)
		while ser.in_waiting:
			driverReturnValue = ord(ser.read())
			print(driverReturnValue)
		serial_mutex.acquire()
		left_value = int(round(191 + leftTrackVel*63))
		ser.write(chr(left_value))
		print("Left value sent:", left_value)
		ser.write(b'%c'%left_value)
		time.sleep(0.04)
		right_value = int(round(63 + rightTrackVel*63))
		ser.write(b'%c'%right_value)
		time.sleep(0.04)
		serial_mutex.release()
		

if __name__ == '__main__':
	# If running this on windows, 
	# you will need to change the port to COM3 or similar, use device manager
	ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
	time.sleep(4)
	
	try:
		thread = Thread(target=arduino_serial, args=(ser,))
		thread.start()
        
		# Hold position for 1 second
		serial_mutex.acquire()
		leftTrackVel = 0
		rightTrackVel = 0
		serial_mutex.release()
		time.sleep(1)

		# Move tracks forward at full speed
		serial_mutex.acquire()
		leftTrackVel = 0
		rightTrackVel = 1
		serial_mutex.release()
		time.sleep(2)

		# Hold position
		serial_mutex.acquire()
		leftTrackVel = 0
		rightTrackVel = 0
		serial_mutex.release()
		time.sleep(1)
	
	finally:
		print("Finishing")
		stop = True
		thread.join()
		ser.close()
