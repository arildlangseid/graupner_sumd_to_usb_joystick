#include <Joystick.h>

//
// Adapter to connect a Graupner SUMD receiver as a gamepad to USB on a PC
// Made to be used with RealFlight 8
//
// Connect HoTT receiver SUMD output to Serial-RX pin on Leonardo
//

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

#define BUTTON_MODE 0 
#define BUTTON_SMOKE 1
#define BUTTON_RESET 2
#define BUTTON_DUALRATE 3

#define RANGE_INPUT_LOW 1000
#define RANGE_INPUT_HIGH 2000

#define SUMD_MAXCHAN 16
#define SUMD_BUFFERSIZE SUMD_MAXCHAN*2+5

// Create the Joystick
Joystick_ Joystick;

static uint8_t sumd[SUMD_BUFFERSIZE]={0};
static int channel[SUMD_MAXCHAN];

// Last state of inputs and switches
static int lastThrottle = 1500;
static int lastAileron = 1500;
static int lastElevator = 1500;
static int lastRudder = 1500;
static int lastFlap = 1500;
static bool lastDualRate = false;
static bool lastSmoke = false;
static bool lastMode = false;
static bool lastReset = false;

// For parsing SUMD protocol
static uint8_t sumdIndex=0;
static uint8_t sumdSize=0;
static uint8_t channelCounter=0;


/*
* setup()
*/
void setup() {
#ifdef PRINT_DEBUG
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for native USB CDC to be ready
  }
#endif

	// Initialize Joystick Library
	Joystick.begin();
  Joystick.setAcceleratorRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH); // Accelerator will appear on RealFlight ch9
  Joystick.setBrakeRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH);
  Joystick.setRudderRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH); // Rudder will appear on RealFlight ch11
  Joystick.setRxAxisRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH); // XRotation will appear on RealFlight ch3
  Joystick.setRyAxisRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH); // YRotation will appear on RealFlight ch2
  Joystick.setRzAxisRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH);
  Joystick.setSteeringRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH);
  Joystick.setThrottleRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH); // Throttle will appear on RealFlight ch10
  Joystick.setXAxisRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH);
  Joystick.setYAxisRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH);
  Joystick.setZAxisRange(RANGE_INPUT_LOW, RANGE_INPUT_HIGH); // ZAxis will appear on RealFlight ch4

  // SUMD serial params 115200 baud, 8 bit, none parity, 1 stop bit
  Serial1.begin(115200);
}

/*
* loop()
*/
void loop() {
  // Read SUMD on serial-rx pin
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
}

/*
* setJoystickValues()
*/
void setJoystickValues(uint8_t channelCounter) {
  if (channelCounter==THROTTLE) {
    int throttle = channel[THROTTLE];
    if (lastThrottle!=throttle) {
      Joystick.setThrottle(throttle); // Throttle will appear on RealFlight ch10
      lastThrottle=throttle;
    }
  } else if (channelCounter==AILERON) {
    int aileron = channel[AILERON];
    if (lastAileron!=aileron) {
      Joystick.setRyAxis(aileron); // YRotation will appear on RealFlight ch2
      lastAileron=aileron;
    }
  } else if (channelCounter==ELEVATOR) {
    int elevator=channel[ELEVATOR];
    if (lastElevator!=elevator) {
      Joystick.setRxAxis(elevator); // XRotation will appear on RealFlight ch3
      lastElevator=elevator;
    }
  } else if (channelCounter==RUDDER) {
    int rudder = channel[RUDDER];
    if (lastRudder!=rudder) {
      Joystick.setZAxis(rudder); // ZAxis will appear on RealFlight ch4
      lastRudder=rudder;
    }
  } else if (channelCounter==FLAP) {
    int flap = channel[FLAP];
    if (lastFlap!=flap) {
      Joystick.setYAxis(flap); // Accelerator will appear on RealFlight ch9
      lastFlap=flap;
    }
  } else if (channelCounter==DUALRATE) {
    bool dualrate = (channel[DUALRATE] > 1500) ? true : false;
    if (lastDualRate != dualrate)
    {
      Joystick.setButton(BUTTON_DUALRATE, dualrate);
      lastDualRate = dualrate;
    }
  } else if (channelCounter==SMOKE) {
    bool smoke = (channel[SMOKE] > 1500) ? true : false;
    if (lastSmoke != smoke)
    {
      Joystick.setButton(BUTTON_SMOKE, smoke);
      lastSmoke = smoke;
    }
  } else if (channelCounter==RESET) {
    bool reset = (channel[RESET] > 1500) ? true : false;
    if (lastReset != reset) {
      Joystick.setButton(BUTTON_RESET, reset);
      lastReset = reset;
    }
  } else if (channelCounter==MODE) {
    bool mode = (channel[MODE] > 1500) ? true : false;
    if (lastMode != mode)
    {
      Joystick.setButton(BUTTON_MODE, mode);
      lastMode = mode;
    }
  }
}


#ifdef PRINT_DEBUG
/*
* debug()
*/
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