import time
from picamera import PiCamera
from gpiozero import OutputDevice, MotionSensor, LightSensor
import serial, csv
import threading
import adafruit_icm20x



# camera script

# figure out how to get accelerometer data from both pi and flight computers

# init gpio, serial and i2c
smoke = OutputDevice(17)
ser = serial.Serial(
    port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
i2c = board.I2C()
icm = adafruit_icm20x.ICM20649(i2c)


# init constants 
BUFF_DURATION = 15 # number of seconds between buffers
SAMPLE_INTERVAL = .01 # how often we sample the sensors in seconds
buffer_counter = BUFF_DURATION / SAMPLE_INTERVAL # how many samples before we make a new buffer
BUFF_COUNT = 2
file_number = 1 # suffix of file name

# init camera
camera = PiCamera()
camera.resolution = (1024, 768) # (2592, 1944)
camera.framerate = 60

camera.start_recording('%d.h264'%file_number.zfill(4))

#start recording first buffer to file ("Recording" + file_number)
#camera.start_recording('/home/pi/Desktop/video'+file_number.zfill(4)+'.h264')
file_number+=1

while(not(IMU_read() and quark_read())): # if either returns true then break loop
    buffer_counter-=1
    if(buffer_counter <= 0):
        # stop recording newest buffer and save file to ("Recording" + file_number)
        camera.split_recording('%d.h264' % file_number.zfill(4))
    
        if(file_number > BUFF_COUNT):
            # TODO: delete file named "Recording" + (file_number - BUFF_COUNT)
            pass
        file_number+=1
        buffer_counter = BUFF_DURATION / SAMPLE_INTERVAL
    time.sleep(SAMPLE_INTERVAL)
    
# boost detected
    
# launch smoke flare
smoke.on()
    
# wait 15 minutes
camera.wait_recording(15*60)
# stop recording the final file and save to (flightRecording)
camera.stop_recording()


# thermistor script

# implement lookup table

SAMPLE_INTERVAL = .1 # how often data is sampled
sampleTimeAverage = 10 # number of seconds of data to average
upperTempLimit = ? # temp at which the heater should be turned off
lowerTempLimit = ? # temp at which the heater should be turned on
altitudeToDereefAt = 1500 # altitude to start melting the wire

numberOfSamplesToAverage = sampleTimeAverage / SAMPLE_INTERVAL
sampleData[numberOfSamplesToAverage] = {apogeeAltitude} # set all altitude samples to the apogee altitude initially
oldestDataIndex = 0

# on pad
while(not IMU_read() or not quark_read()):
  time.sleep(sampleTimeDelay)

# going up
hasDetectedApogee = 0
apogeeDetectPin = TBD
while(!get(apogeeDetectPin)):
  time.sleep(SAMPLE_INTERVAL)


# going down but not disreefing yet
while(currentAltitude > altitudeToDereefAt):
  sampleData[oldestDataIndex] = current altitude from sensors
  oldestDataIndex+=1
  currentAltitude = 0
  for i in range(1,numberOfSamples):
    currentAltitude += altitudeData[i] / numberOfSamples

  voltage = sample(thermistor)
  temp = convert voltage to temp via lookup table

  if(temp < lowerTempLimit):
    turn on thermister
  if(temp > upperTempLimit):
    turn off thermister
  time.sleep(SAMPLE_INTERVAL)

# disreefing
turn on thermister
time.sleep(60)
turn off thermister

def quark_read():
    # todo fix code
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            if line == "1":
                smoke.on()
            elif line == "0":
                smoke.off()
                
def IMU_read() -> bool:
    a = icm.acceleration
    mag = sqrt(a[0]^2+a[1]^2+a[2]^2)
    return (mag > 5)