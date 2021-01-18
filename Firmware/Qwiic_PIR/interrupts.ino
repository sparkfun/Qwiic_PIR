/******************************************************************************
  interrupts.ino
  Fischer Moseley @ SparkFun Electronics
  Original Creation Date: July 31, 2019

  This file contains the interrupt routines that are triggered upon an I2C write from
  master (receiveEvent), an I2C read (requestEvent), or a button state change
  (eventInterrupt). These ISRs modify the registerMap state variable, and sometimes
  set a flag (updateFlag) that updates things in the main loop.

  This code is beerware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given.
******************************************************************************/

//Turn on interrupts for the various pins
void setupInterrupts() {
  //Attach interrupt to switch
  attachPCINT(digitalPinToPCINT(pirPin), eventInterrupt, CHANGE);
}

//When Qwiic Button receives data bytes from Master, this function is called as an interrupt
void receiveEvent(int numberOfBytesReceived) {
  registerNumber = Wire.read(); //Get the memory map offset from the user

  //Begin recording the following incoming bytes to the temp memory map
  //starting at the registerNumber (the first byte received)
  for (uint8_t x = 0 ; x < numberOfBytesReceived - 1 ; x++) {
    uint8_t temp = Wire.read(); //We might record it, we might throw it away

    if ( (x + registerNumber) < sizeof(memoryMap)) {
      //Clense the incoming byte against the read only protected bits
      //Store the result into the register map
      *(registerPointer + registerNumber + x) &= ~*(protectionPointer + registerNumber + x); //Clear this register if needed
      *(registerPointer + registerNumber + x) |= temp & *(protectionPointer + registerNumber + x); //Or in the user's request (clensed against protection bits)
    }
  }

  //Update the PIREvents and ButtonClicked queues.
  
  //If the user has requested to pop the oldest event off the stack then do so!
  if (registerMap.detectQueueStatus.popRequest) {
    //Update the register with the next-oldest timestamp
    detectEvents.pop();
    registerMap.detectQueueBack = detectEvents.back();

    //Update the status register with the state of the detectEvents buffer
    registerMap.detectQueueStatus.isFull = detectEvents.isFull();
    registerMap.detectQueueStatus.isEmpty = detectEvents.isEmpty();

    //Clear the popRequest bit so we know the popping is done
    registerMap.detectQueueStatus.popRequest = false;
  }
  
  //If the user has requested to pop the oldest event off the stack then do so!
  if (registerMap.removedQueueStatus.popRequest) {
    //Update the register with the next-oldest timestamp
    removedEvents.pop();
    registerMap.removedQueueBack = removedEvents.back();

    //Update the status register with the state of the removedEvents buffer
    registerMap.removedQueueStatus.isFull = removedEvents.isFull();
    registerMap.removedQueueStatus.isEmpty = removedEvents.isEmpty();

    //Clear the popRequest bit so we know the popping is done
    registerMap.removedQueueStatus.popRequest = false;
  }

  updateFlag = true; //Update things like LED brightnesses in the main loop
}

//Respond to GET commands
//When Qwiic Button gets a request for data from the user, this function is called as an interrupt
//The interrupt will respond with bytes starting from the last byte the user sent to us
//While we are sending bytes we may have to do some calculations
void requestEvent() {
  registerMap.eventStatus.rawReading = digitalRead(pirPin);

  //Calculate time stamps before we start sending bytes via I2C
  registerMap.detectQueueBack = millis() - detectEvents.back();
  registerMap.detectQueueFront = millis() - detectEvents.front();
  
  registerMap.removedQueueBack = millis() - removedEvents.back();
  registerMap.removedQueueFront = millis() - removedEvents.front();

  //This will write the entire contents of the register map struct starting from
  //the register the user requested, and when it reaches the end the master
  //will read 0xFFs.

  Wire.write((registerPointer + registerNumber), sizeof(memoryMap) - registerNumber);
}

//Called any time the pin changes state
void eventInterrupt() {

  //Debounce
  if (millis() - lastEventTime < registerMap.eventDebounceTime)
    return;
  lastEventTime = millis();

  registerMap.eventStatus.eventAvailable = true;

  //Update the PIREvents queues and registerMap
  bool pinState = digitalRead(pirPin);
  switch (pinState)
  {
    case 0:
      registerMap.eventStatus.objectRemoved = true;
      removedEvents.push(lastEventTime);
      registerMap.removedQueueStatus.isEmpty = removedEvents.isEmpty();
      registerMap.removedQueueStatus.isFull = removedEvents.isFull();
      break;
    case 1:
      registerMap.eventStatus.objectDetected = true;
      detectEvents.push(lastEventTime);
      registerMap.detectQueueStatus.isEmpty = detectEvents.isEmpty();
      registerMap.detectQueueStatus.isFull = detectEvents.isFull();
      break;
  }
}
