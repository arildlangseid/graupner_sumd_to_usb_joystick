#include <Joystick.h>

// Uncomment for debug-printing
//#define PRINT_DEBUG


#define THROTTLE 0
#define AILERON 1
#define ELEVATOR 2
#define RUDDER 3
#define FLAP 4
#define DUALRATE 5
#define SMOKE 6
#define MODE 7
#define RESET 9

#define SUMD_MAXCHAN 16
#define SUMD_BUFFERSIZE SUMD_MAXCHAN*2+5

static uint8_t sumd[SUMD_BUFFERSIZE]={0};

//#ifdef PRINT_DEBUG
  static int channel[SUMD_MAXCHAN];
//#endif

// Create the Joystick
Joystick_ Joystick;

// Constant that maps the physical pin to the joystick button.
const int pinToButtonMap = 9;

void setup() {
#ifdef PRINT_DEBUG
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for native USB CDC to be ready
  }
#endif

	// Initialize Button Pins
	pinMode(pinToButtonMap, INPUT_PULLUP);

#define INPUT_LOW 1000
#define INPUT_HIGH 2000

	// Initialize Joystick Library
	Joystick.begin();
  Joystick.setAcceleratorRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setBrakeRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setRudderRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setRxAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setRyAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setRzAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setXAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setYAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setZAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setSteeringRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setAcceleratorRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setThrottleRange(INPUT_LOW, INPUT_HIGH);

  // SUMD serial params 115200 baud, 8 bit, none parity, 1 stop bit
  Serial1.begin(115200);
}

// Last state of the switches
int lastThrottle = 1500;
int lastAileron = 1500;
int lastElevator = 1500;
int lastRudder = 1500;
int lastFlap = 1500;

bool lastDualRate = false;
bool lastSmoke = false;
bool lastMode = false;

bool lastReset = false;

static uint8_t sumdIndex=0;
static uint8_t sumdSize=0;
static uint8_t channelCounter=0;
void loop() {
  if (Serial1.available()) {
    int val = Serial1.read();
    // 0xA8 - start of frame
    if (sumdIndex == 0 && val != 0xA8) { return; }
    // 0x00 - SUMH
    // 0x01 - SUMD
    if (sumdIndex == 1 && (val == 0x00 && val == 0x01)) { sumdIndex = 0; return; }
    // number of channels
    if (sumdIndex == 2) { sumdSize = val; }
    if (sumdIndex < SUMD_BUFFERSIZE ) { sumd[sumdIndex] = val; }

    if (sumdIndex > 2 && sumdIndex % 2 == 0 && channelCounter < sumdSize) {
      /*sumdIndex is even*/
      int rcValue = ((sumd[sumdIndex-1]<<8 | sumd[sumdIndex]))>>3;
      if (rcValue>750 && rcValue<2250) {
        channel[channelCounter] = rcValue;
        setJoystickValues(channelCounter);
        channelCounter++;
      }
    }
    sumdIndex++;

    if (sumdIndex == sumdSize*2+5) {
      sumdIndex = 0;
      channelCounter = 0;
#ifdef PRINT_DEBUG
      debug();
#endif
    }
  }


//	delay(50);
}

void setJoystickValues(uint8_t channelCounter) {
  if (channelCounter==THROTTLE) {
    int throttle = channel[THROTTLE];
    if (lastThrottle!=throttle) {
      Joystick.setThrottle(throttle); // Throttle ch10
      lastThrottle=throttle;
    }
  } else if (channelCounter==AILERON) {
    int aileron = channel[AILERON];
    if (lastAileron!=aileron) {
      Joystick.setRyAxis(aileron); // YRotation ch2
      lastAileron=aileron;
    }
  } else if (channelCounter==ELEVATOR) {
    int elevator=channel[ELEVATOR];
    if (lastElevator!=elevator) {
      Joystick.setRxAxis(elevator); // XRotation ch3
//    Joystick.setYAxis(elevator); // Accelerator ch9
//    Joystick.setRzAxis(elevator); // Rudder ch11

//    Joystick.setAccelerator(elevator);
//    Joystick.setBrake(elevator);
//    Joystick.setRudder(elevator);
//    Joystick.setXAxis(aileron);
//    Joystick.setSteering(elevator);
      lastElevator=elevator;
    }
  } else if (channelCounter==RUDDER) {
    int rudder = channel[RUDDER];
    if (lastRudder!=rudder) {
      Joystick.setZAxis(rudder); // ZAxis ch4
      lastRudder=rudder;
    }
  } else if (channelCounter==FLAP) {
    int flap = channel[FLAP];
    if (lastFlap!=flap) {
      Joystick.setYAxis(flap); // Accelerator ch9
      lastFlap=flap;
    }
  } else if (channelCounter==DUALRATE) {
    bool dualrate = (channel[DUALRATE] > 1500) ? true : false;
    if (lastDualRate != dualrate)
    {
      Joystick.setButton(0, dualrate);
      lastDualRate = dualrate;
    }
  } else if (channelCounter==SMOKE) {
    bool smoke = (channel[SMOKE] > 1500) ? true : false;
    if (lastSmoke != smoke)
    {
      Joystick.setButton(1, smoke);
      lastSmoke = smoke;
    }
  } else if (channelCounter==RESET) {
    bool reset = (channel[RESET] > 1500) ? true : false;
    if (lastReset != reset) {
      Joystick.setButton(2, reset);
      lastReset = reset;
    }
  } else if (channelCounter==MODE) {
    bool mode = (channel[MODE] > 1500) ? true : false;
    if (lastMode != mode)
    {
      Joystick.setButton(3, mode);
      lastMode = mode;
    }
  }


// Uprøvde
/*
  Joystick.setAcceleratorRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setBrakeRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setRudderRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setRzAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setXAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setYAxisRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setSteeringRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setAcceleratorRange(INPUT_LOW, INPUT_HIGH);
  Joystick.setThrottleRange(INPUT_LOW, INPUT_HIGH);
*/

}


#ifdef PRINT_DEBUG
void debug() {
  Serial.print(sumdSize);
  Serial.print("\t");
  Serial.print(channel[0]);
  for (uint8_t i=1;i<sumdSize;i++) {
    Serial.print("\t");
    Serial.print(channel[i]);
  }
  Serial.println("");
}
#endif