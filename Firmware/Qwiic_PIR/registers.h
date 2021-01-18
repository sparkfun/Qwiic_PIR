/******************************************************************************
  registers.h
  Fischer Moseley @ SparkFun Electronics
  Original Creation Date: July 31, 2019

  This file defines the memoryMap struct, which acts as the pseudo register map
  of the Qwiic Button/Switch. It also serves as an easy way to access variables
  and manipulate the state of the device.

  During I2C transactions, the memoryMap object is wrapped as a collection of
  bytes. The byte that the user is interested in (either to read or write) is
  selected with a register pointer. For instance, if the user sets the pointer
  to 0x0e, they will be addressing the 4th uint8_t sized object in this struct.
  In this case, that would be the interruptConfig register!

  This code is beerware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given.
******************************************************************************/

typedef union {
  struct {
    bool rawReading : 1;
    bool eventAvailable : 1; //This is bit 0. User mutable, gets set to 1 when a new event occurs. User is expected to write 0 to clear the flag.
    bool objectRemoved : 1; //Defaults to zero on POR. Gets set to one when the button gets clicked. Must be cleared by the user.
    bool objectDetected : 1;  //Gets set to one if button is pushed.
    bool : 4;
  };
  uint8_t byteWrapped;
} statusRegisterBitField;

typedef union {
  struct {
    bool detectEnable : 1; //user mutable, set to 1 to enable an interrupt when the button is pressed. Defaults to 0.
    bool removeEnable : 1; //user mutable, set to 1 to enable an interrupt when the button is pressed. Defaults to 0.
    bool: 6;
  };
  uint8_t byteWrapped;
} interruptConfigBitField;

typedef union {
  struct {
    bool popRequest : 1; //This is bit 0. User mutable, user sets to 1 to pop from queue, we pop from queue and set the bit back to zero.
    bool isEmpty : 1; //user immutable, returns 1 or 0 depending on whether or not the queue is empty
    bool isFull : 1; //user immutable, returns 1 or 0 depending on whether or not the queue is full
    bool: 5;
  };
  uint8_t byteWrapped;
} queueStatusBitField;

typedef struct memoryMap {
  //Button Status/Configuration                       Register Address
  uint8_t id;                                             // 0x00
  uint8_t firmwareMinor;                                  // 0x01
  uint8_t firmwareMajor;                                  // 0x02

  statusRegisterBitField eventStatus;                    // 0x03

  //Interrupt Configuration
  interruptConfigBitField interruptConfigure;                // 0x04
  uint16_t eventDebounceTime;                            // 0x05

  //PIREvents queue manipulation and status functions
  queueStatusBitField detectQueueStatus;                 // 0x07
  unsigned long detectQueueFront;                        // 0x08
  unsigned long detectQueueBack;                         // 0x0C

  queueStatusBitField removedQueueStatus;                 // 0x10
  unsigned long removedQueueFront;                        // 0x11
  unsigned long removedQueueBack;                         // 0x15
  
  //Device Configuration
  uint8_t i2cAddress;                                     // 0x19
};
