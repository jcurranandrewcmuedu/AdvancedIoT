//Welcome to Jamie's RevengeCycle code. Come along for the ride! Mwahhahaha!

int LEDpin = D0;//sets LED pin to D0

int buttonPin = D1; //button
Timer timerResetServo(2000, resetServo, TRUE);

int HALLpin = D3; // sets hall effects sensor to pin D0
volatile int state = LOW; //for system to switch led on/off by sensor trigger

unsigned long magnetpassedat = 0;//set time that hall senses magnet on wheel spoke, for speed calculation
unsigned long lastmagnetpassedat = 0;//holder for previous time that hall sensed magnet
double speednow = 0;
bool movingnow = FALSE;
bool triggered = FALSE;
unsigned long triggeredAt = 0;

int SERVOpin = A5;//servo setup
Servo myServo;
int servoPos = 0;

//The minimum and maximum values that came from
//the accelerometer while standing still
//You very well may need to change these
int minVal = 0;
int maxVal = 10000;

int IRpinBack = A2; //sets IR sensor to pin A4
int IRpinFront = A1; //sets IR sensor to pin A3
//variables for low pass filter
const int numReadings = 7;
int readIndex = 0;

int FrontTotal = 0;
int FrontAverage = 0;
int FrontReadings[numReadings];

int BackTotal = 0;
int BackAverage = 0;
int BackReadings[numReadings];



void setup(){
 Serial.begin(9600);

 myServo.attach(A5);//sets servo control to pin A5
 pinMode(HALLpin, INPUT_PULLUP);
 attachInterrupt( HALLpin , findspeed, RISING );//anytime hall sensor goes from 0->1, function will trigger

 pinMode(LEDpin, OUTPUT);//LED is an output
 pinMode( buttonPin , INPUT); // sets button as input
 attachInterrupt( buttonPin , buttonSprayPaint, FALLING );//button is an interrupt that will cause the paint to spray

 Particle.function("servo", servoControl); //servo function only for testing
 Particle.variable(  "servoPos" , &servoPos , INT );//servo function only for testing
 Particle.function("trigger", servTrigger);//servo function only for testing

 for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
      FrontReadings[thisReading] = 0;
      BackReadings[thisReading] = 0;
  }

 }


void loop(){

  int HallState = digitalRead( HALLpin );//output from hall effects sensor on wheel
  //Serial.print(HallState);
  //Serial.print(speednow);
  //Serial.print(",");
  delay(10);

  //troubleshoot buttonPin
  int buttonState = digitalRead( buttonPin );
  Serial.print(buttonState);
  Serial.print(",");

//troubleshooting for speed, placeholder
if(speednow > 10){
    //digitalWrite(LEDpin, HIGH);
    bool movingnow = TRUE;
}else{
    //digitalWrite(LEDpin, LOW);
    bool movingnow = FALSE;
}


int distanceFrontState = analogRead(IRpinFront ); //take IR analog input from IR sensor
int distanceBackState = analogRead(IRpinBack ); //take IR analog input from IR sensor
/*Serial.print("front,");
Serial.print(distanceFrontState);
Serial.print(",");
Serial.print(distanceBackState);
Serial.print(",");*/

// subtract the last reading:
      FrontTotal = FrontTotal - FrontReadings[readIndex];
      BackTotal = BackTotal - BackReadings[readIndex];
      // read from the sensor:
      FrontReadings[readIndex] = analogRead(IRpinFront);
      BackReadings[readIndex] = analogRead(IRpinBack);
      // add the reading to the total:
      FrontTotal = FrontTotal + FrontReadings[readIndex];
      BackTotal = BackTotal + BackReadings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;

      // if we're at the end of the array...
      if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
      }

      // calculate the average:
      FrontAverage = FrontTotal / numReadings;
      BackAverage = BackTotal / numReadings;
      // send it to the computer as ASCII digit
      /*Serial.print(FrontAverage);
      Serial.print(",");
      Serial.println(BackAverage);*/

//now for the actual trigger logic. If bike is moving and back IR spikes, set a trigger state, and note time
if (movingnow == TRUE){
      if (BackAverage > 2000){
        triggered = TRUE;
        triggeredAt = millis();
        Serial.print("triggered");
        digitalWrite(LEDpin, HIGH);
          }else{
            triggered = FALSE;
            digitalWrite(LEDpin, LOW);}
      }else{
        triggered = FALSE;
        digitalWrite(LEDpin, LOW);
    }

//calculate time since trigger went off
int triggeredSince = millis()-triggeredAt;

//if bike is in trigger state for less than 80ms, and front IR spikes also, fire away
if (triggered == TRUE){
  if (triggeredSince< 80){
    if(FrontAverage > 2000){
        sprayPaint();
        Serial.print("go spray");
      }else{
        myServo.write(90);
        digitalWrite(LEDpin, LOW);}
    } else{
      triggered = FALSE;
      myServo.write(90);
      digitalWrite(LEDpin, LOW);}
    } else{}

  }

void sprayPaint(){
  Serial.print("spray function");
  digitalWrite(LEDpin, HIGH);
  delay(2500);
  myServo.write(5);
  delay(1000);
  myServo.write(90);
  digitalWrite(LEDpin, LOW);
}

void buttonSprayPaint(){
  Serial.println("button spray");
  digitalWrite(LEDpin, HIGH);
  myServo.write(5);
  timerResetServo.startFromISR();
}

void resetServo(){
  Serial.println("button reset");
    myServo.write(90);
    digitalWrite(LEDpin, LOW);
}

double findspeed() {
    //led will switch between on/off when triggered
    //state = ! state;
    //digitalWrite(LEDpin, state);

    magnetpassedat = millis();
    speednow = 14660.7657 / (magnetpassedat-lastmagnetpassedat); //in feet/sec
    //2piR*1000/12, bike tire is 27", converted to ft/sec

    lastmagnetpassedat = magnetpassedat; //set time for next time function is called
    return speednow;
  }



int servoControl(String command)
{
    // Convert
   int newPos = command.toInt();
   // Make sure it is in the right range
   // And set the position
   servoPos = constrain( newPos, 0 , 180);
   // Set the servo
   myServo.write( servoPos );
   // done
   return 1;
}

int servTrigger(String command){
  int trig = command.toInt();
  if (trig == 4){
    //delay(2500);
    myServo.write(5);
    delay(700);
    myServo.write(90);
  }
}
