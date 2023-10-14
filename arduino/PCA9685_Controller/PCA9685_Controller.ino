#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

/* Initialize the Adafruit servo driver. It uses the default address of 0x40. */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* Set the min and max pulse lengths. 1000us and 2000us respectively.*/
#define USMIN  1000
#define USMAX  2000

/* Set the servo update frequency. Analog servos run at ~50 Hz apparently. */
#define SERVO_FREQ 50

/* Define the steering servo port number on the PCA. */
static uint8_t STEERING_SERVO= 0;

const byte numChars= 32;
char receivedChars[numChars];
char tempChars[numChars]; /* Temporary array for use when parsing. */

/* Hold the parsed data. */
unsigned int servo_position= 0;

boolean newData= false;

void setup() {
  Serial.begin(115200);
  Serial.println("16 channel servo controller for R2RC.\n");

  /* Initialize the servo controller. */
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  /* Let the controller settle for a short time. */
  delay(100);
}

/* 
Drive each servo one at a time using writeMicroseconds(), it's not 
precise due to calculation rounding! The writeMicroseconds() function is 
used to mimic the Arduino Servo library writeMicroseconds() behavior.
*/


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// split the data into its parts
void parseData() {
  char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,","); // get the first part
    servo_position = atoi(strtokIndx);
 
    strtokIndx = strtok(NULL, ","); // continue where it left off
    servo_position = atoi(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    servo_position = atoi(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    servo_position = atoi(strtokIndx);
}

void sendData(){
  pwm.writeMicroseconds(STEERING_SERVO, servo_position);
}


void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);

    parseData();
    sendData();
    newData= false;
  }
}
