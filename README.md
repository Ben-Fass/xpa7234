# camera script

//figure out how to get accelerometer data from both pi and flight computers

{
bufferTimer = 15 //number of seconds between buffers
sampleTimeDelay = .01 // how often we sample the sensors in seconds
bufferCounter = bufferTimer / sampleTimeDelay //how many samples before we make a new buffer
numberOfBuffers = 2
fileNumber = 1

start recording first buffer to file ("Recording" + fileNumber)
fileNumber++

while((pi accelerometer magnitude greater than 5) or (eggtimer accelerometer magnitude greater than 5))
{
  bufferCounter--
  if(bufferCounter <= 0)
  {
    stop recording newest buffer and save file to ("Recording" + fileNumber)
    
    if(fileNumber > numberOfBuffers)
    {
      delete file named "Recording" + (fileNumber - numberOfBuffers)
    }
    
    fileNumber++
    start new recording to a file named ("Recording" + fileNumber)
    bufferCounter = bufferTimer / sampleTimeDelay
  }
  time.sleep(sampleTimeDelay)
}

    
wait 15 minutes
stop recording the final file and save to (flightRecording)
}


