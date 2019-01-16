// Hacked from iBus2PPM v2 version 1.01
// https://github.com/povlhp/iBus2PPM
// so this project has license GPL 3.0 same as iBusPPM
// Arduino Nano code to read FlySky iBus and output PWM signals to 4 servos

#include <string.h>
#include <Servo.h>

#define IBUS_MAXCHANNELS 14
#define FAILSAFELIMIT 1020    // When all the 6 channels below this value assume failsafe
#define IBUS_BUFFSIZE 32    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)


static uint16_t rcFailsafe[IBUS_MAXCHANNELS] = {  1500, 1500, 950, 1500, 2000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
static uint16_t rcValue[IBUS_MAXCHANNELS];
static uint16_t rcValueSafe[IBUS_MAXCHANNELS]; // read by interrupt handler. Data copied here in cli/sei block
static boolean rxFrameDone;
static boolean failsafe = 0;
Servo servoLeftThigh;
Servo servoLeftShin;
Servo servoRightThigh;
Servo servoRightShin;


// Prototypes
void setupRx();
void readRx();


void setup() {
  setupRx();
  setupServos();
}

void loop() {
  for(int i = 0; i<4; i++)
  {
    readRx();  
    setServos(i);
  }
}

void setupServos()
{
  servoLeftThigh.attach(8);
  servoLeftShin.attach(9);
  servoRightThigh.attach(10);
  servoRightShin.attach(11);
}

void setServoHeight(bool right_not_left, bool thigh_not_shin, double height)
{
  // sets the servo to a height, 1.0 is total maximum, -1.0 is total minimum
  if(right_not_left)
  {
    if(thigh_not_shin)
      servoRightThigh.writeMicroseconds(1500 + height * 500);
    else
      servoRightShin.writeMicroseconds(1500 + height * 500); 
  }
  else
  {
    if(thigh_not_shin)
      servoLeftThigh.writeMicroseconds(1500 - height * 500);
    else
      servoLeftShin.writeMicroseconds(1500 - height * 500); 
  }
}


void setServos(int servo)
{
  double pitch = ((double)rcValue[1] - 1500)*0.002; // + is stick up, - is stick down, range from -1.0 to 1.0
  double roll = ((double)rcValue[0] - 1500)*0.002;; // + is right, - is left,  range from -1.0 to 1.0 
  double throttle = ((double)rcValue[2] - 1000)*0.001; // number from 0.0 to 1.0
  double yaw = ((double)rcValue[3] - 1500)*0.002;; // + is right, - is left,  range from -1.0 to 1.0 

  switch(servo){
    case 0:
      setServoHeight(false, true, (throttle - 0.5) * 0.5);
      break;

    case 1:
      setServoHeight(false, false, yaw * 0.25);
      break;
      
    case 2:
      setServoHeight(true, false, yaw * 0.25);
      break;
      
    case 3:
      setServoHeight(true, false, yaw * 0.25);
      break;

    default:
      break;
  }
}

static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};

void setupRx()
{
  uint8_t i;
  for (i = 0; i < IBUS_MAXCHANNELS; i++) { rcValue[i] = 1127; }
  Serial.begin(115200);

  // set up LED for serial error message
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);  // Checksum error - turn on error LED
}

void readRx()
{
  uint8_t i;
  uint16_t chksum, rxsum;

  rxFrameDone = false;

  uint8_t avail = Serial.available();
  
  if (avail)
  {
    digitalWrite(4, LOW);
    uint8_t val = Serial.read();
    // Look for 0x2040 as start of packet
    if (ibusIndex == 0 && val != 0x20) {
      return;
    }
    if (ibusIndex == 1 && val != 0x40) {
      ibusIndex = 0;
      return;
    }
 
    if (ibusIndex < IBUS_BUFFSIZE) ibus[ibusIndex] = val;
    ibusIndex++;

    if (ibusIndex == IBUS_BUFFSIZE)
    {
      ibusIndex = 0;
      chksum = 0xFFFF;
      for (i = 0; i < 30; i++)
        chksum -= ibus[i];

      rxsum = ibus[30] + (ibus[31] << 8);
      if (chksum == rxsum)
      {
        //Unrolled loop  for 10 channels - no need to copy more than needed.
        // MODIFY IF MORE CHANNELS NEEDED
        rcValue[0] = (ibus[ 3] << 8) + ibus[ 2];
        rcValue[1] = (ibus[ 5] << 8) + ibus[ 4];
        rcValue[2] = (ibus[ 7] << 8) + ibus[ 6];
        rcValue[3] = (ibus[ 9] << 8) + ibus[ 8];
        rcValue[4] = (ibus[11] << 8) + ibus[10];
        rcValue[5] = (ibus[13] << 8) + ibus[12];
        rcValue[6] = (ibus[15] << 8) + ibus[14];
        rcValue[7] = (ibus[17] << 8) + ibus[16];
        rcValue[8] = (ibus[19] << 8) + ibus[18];
        rcValue[9] = (ibus[21] << 8) + ibus[20];
        rxFrameDone = true;
        if (rcValue[0] < FAILSAFELIMIT && rcValue[1] < FAILSAFELIMIT &&
            rcValue[2] < FAILSAFELIMIT && rcValue[3] < FAILSAFELIMIT &&
            rcValue[4] < FAILSAFELIMIT && rcValue[5] < FAILSAFELIMIT ) 
        {
          failsafe = 1;
          cli(); // disable interrupts
          memcpy(rcValueSafe, rcFailsafe, IBUS_MAXCHANNELS * sizeof(uint16_t));
          sei();
          digitalWrite(13, HIGH);  //  Error - turn on error LED
        }
        else
        {
          // Now we need to disable interrupts to copy 16-bit values atomicly
          // Only copy needed signals (10 channels default)
          // MODIFY IF MORE CHANNELS NEEDED
          cli(); // disable interrupts.
          rcValueSafe[0] = rcValue[0];
          rcValueSafe[1] = rcValue[1];
          rcValueSafe[2] = rcValue[2];
          rcValueSafe[3] = rcValue[3];
          rcValueSafe[4] = rcValue[4];
          rcValueSafe[5] = rcValue[5];
          rcValueSafe[6] = rcValue[6];
          rcValueSafe[7] = rcValue[7];
          rcValueSafe[8] = rcValue[8];
          rcValueSafe[9] = rcValue[9];
          sei();
          digitalWrite(13, LOW); // OK packet - Clear error LED
        }
      } else {
        digitalWrite(13, HIGH);  // Checksum error - turn on error LED
      }
      return;
    }
  }
}
