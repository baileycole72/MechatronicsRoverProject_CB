
#include <SoftwareSerial.h>;
#include <Servo.h>;

//#define BT_SERIAL_TX 10
//#define BT_SERIAL_RX 11
//SoftwareSerial BluetoothSerial(BT_SERIAL_TX,BT_SERIAL_RX);

Servo myservo;

// Left wheel motor
int motorL = 8;
int motorL_dir2 = 37;         //direction 1 ON is forward for both motors
int motorL_dir1 = 36;
int encL_ch1 = 19;            // encoder channel 1 has interrupt, channel 2 does not
int encL_ch2 = 38;
volatile int enc_countL = 0;

// Right wheel motor
int motorR = 12;
int motorR_dir1 = 34;          //direction 1 ON is forward for both motors
int motorR_dir2 = 35;
int encR_ch1 = 18;             // encoder channel 1 has interrupt, channel 2 does not
int encR_ch2 = 31;
volatile int enc_countR = 0;

// Arm motor
int motorA = 9;               // direction 1 ON raises the arm
int motorA_dir2 = 43;
int motorA_dir1 = 42;
int encA_ch1 = 3;               // encoder channel 1 has interrupt, channel 2 does not
int encA_ch2 = 49;
volatile int enc_countA = 0;


// Gripper motor
int motorG = 5;               // direction 1 ON opens the gripper
int motorG_dir1 = 58;
int motorG_dir2 = 59;

//servo
int servoPin = 62; 

// ultrasonic sensor
int pingPin = 66; 
long duration;
long cm;

// Other variables for main loop
int distF;
int distR;
int distL;

int command = 0;

/******************************************************************************************************/
/******************************************************************************************************/

void setup()
{
  
  Serial.begin(115200);
  Serial3.begin(115200); //for Bluetooth module  
  delay(1000);

  
  // pin and interrupt initialization

  myservo.attach(servoPin,900,2900);
  
  attachInterrupt(5,isr_process_encoderR,CHANGE);  // interrupt connected to pin 18 for right encoder ch1
  attachInterrupt(4,isr_process_encoderL,CHANGE);  // interrupt connected to pin 19 for left encoder ch1
  attachInterrupt(1,isr_process_encoderA,CHANGE);   // interrupt connected to pin 3 for arm encoder ch1
  
  pinMode(motorL_dir1, OUTPUT);
  pinMode(motorL_dir2, OUTPUT);
  pinMode(motorR_dir1, OUTPUT);
  pinMode(motorL_dir2, OUTPUT);
  pinMode(motorA_dir1, OUTPUT);
  pinMode(motorA_dir2, OUTPUT);
  pinMode(motorG_dir1, OUTPUT);
  pinMode(motorG_dir2, OUTPUT);
  

  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

//  phase1();   //run at the beginning of the maze, before solving.
  myservo.write(85);
  
  Bluetooth(); // manual control

}

/******************************************************************************************************/
/******************************************************************************************************/

void loop()
{
  /* Main loop for solving the maze goes here
   *  
   * Phase 1 (Bluetooth) starts in setup to pick up the cup and orient our selves at the first turn
   *     
   * All sub functions are written below for the wheels, arm, gripper, and servo control    
   * 
   * Three encoder sub functions were created to keep track of the wheels and arm rotation
  */


  pingSensor();
  Serial.print("CM");
  Serial.println(cm);
if (cm < 26 && cm > 7)     // if bot gets too close to the wall, slow down and wait to get 7 cm to the wall
{
   digitalWrite(motorL_dir1, HIGH);
   digitalWrite(motorL_dir2, LOW);
   digitalWrite(motorR_dir1, HIGH);
   digitalWrite(motorR_dir2, LOW);     
   analogWrite(motorL,100);
   analogWrite(motorR,100);
   delay(100);
}
else if (cm <= 7) // once at 7 cm from the wall, read both sides with sensor and go towards the open direction
  {
    Stop();
    readSensor();
    if (distL < 40 || distL<distR)
    {
      turnRight(700);
    }
    else if (distR < 40 || distR<distL)
    {
      turnLeft(700);
    }
    else 
    {
      Stop();
    }
  }
  else 
  {
   goStraight();   
  }

} 


/******************************************************************************************************/
/******************************************************************************************************/

void turnLeft(int ticks)
{
  int enc_count = enc_countR + ticks;
  
  while (enc_countR < enc_count)
  {
     digitalWrite(motorL_dir1, LOW);
     digitalWrite(motorL_dir2, HIGH);
     digitalWrite(motorR_dir1, HIGH);
     digitalWrite(motorR_dir2, LOW);  
     analogWrite(motorL,150);
     analogWrite(motorR,150);
     Serial.println(enc_countR);
  }
  digitalWrite(motorL_dir2, LOW);
  digitalWrite(motorR_dir1, LOW);
  
  delay(100);
}

void turnLeftSlow(int ticks)
{
  int enc_count = enc_countR + ticks;
  
  while (enc_countR < enc_count)
  {
     digitalWrite(motorL_dir1, LOW);
     digitalWrite(motorL_dir2, HIGH);
     digitalWrite(motorR_dir1, HIGH);
     digitalWrite(motorR_dir2, LOW);  
     analogWrite(motorL,100);
     analogWrite(motorR,100);
     Serial.println(enc_countR);
  }
  digitalWrite(motorL_dir2, LOW);
  digitalWrite(motorR_dir1, LOW);
  
}

void turnRight(int ticks)
{
 int enc_count = enc_countL + ticks;
  
    while (enc_countL < enc_count)
  {
     digitalWrite(motorL_dir1, HIGH);
     digitalWrite(motorL_dir2, LOW);
     digitalWrite(motorR_dir1, LOW);
     digitalWrite(motorR_dir2, HIGH);  
     analogWrite(motorL,150);
     analogWrite(motorR,150);
     Serial.println(enc_countL);
  }
  digitalWrite(motorL_dir1, LOW);
  digitalWrite(motorR_dir2, LOW);
  enc_countR=0;
  enc_countL=0;
  delay(100);
}
void turnRightSlow(int ticks)
{
 int enc_count = enc_countL + ticks;
  
    while (enc_countL < enc_count)
  {
     digitalWrite(motorL_dir1, HIGH);
     digitalWrite(motorL_dir2, LOW);
     digitalWrite(motorR_dir1, LOW);
     digitalWrite(motorR_dir2, HIGH);  
     analogWrite(motorL,100);
     analogWrite(motorR,100);
     Serial.println(enc_countL);
  }
  digitalWrite(motorL_dir1, LOW);
  digitalWrite(motorR_dir2, LOW);
  enc_countR=0;
  enc_countL=0;
}

void goStraight()
{
  digitalWrite(motorL_dir1, HIGH);
  digitalWrite(motorL_dir2, LOW);
  digitalWrite(motorR_dir1, HIGH);
  digitalWrite(motorR_dir2, LOW);
  analogWrite(motorL,150);
  analogWrite(motorR,150);
  delay(100);

  checkSide();
  
  if (enc_countL > enc_countR+30)
  {
    turnLeft(15);
  }
  else if (enc_countR > enc_countL+30)
  {
    turnRight(15);
  }
  
}

void checkSide()
{
 //sweep to right
/*  for (int i=180; i>0; i--)
  {
    myservo.write(i);

    int degreeread = myservo.read();
    Serial.print("degrees");
    Serial.println(degreeread);
  }*/
  myservo.write(0);
  delay(250);
  pingSensor();
  distR=cm;
  delay(20);
  
// sweeo back to middle
// longer sweeps are commented
 /*  for (int i=0; i<85; i++)
  {
    myservo.write(i);
    
    int degreeread = myservo.read();
    Serial.print("degrees");
    Serial.println(degreeread);
  }
  */
  myservo.write(85);
  delay(250);
  pingSensor();
  distF=cm;
  delay(20);

  if (distR > 19 && distR < 50)  // if wall is too far from right wall, turn closer. if in an intersection ( >50) ignore reading
  {
    turnRight(50);
    digitalWrite(motorL_dir1, HIGH);
    digitalWrite(motorL_dir2, LOW);
    digitalWrite(motorR_dir1, HIGH);
    digitalWrite(motorR_dir2, LOW);
    analogWrite(motorL,175);
    analogWrite(motorR,175);
    delay(500);
  } 
  else if (distR < 15)  // if too close to the right wall, move away
  {
    turnLeft(50);
    digitalWrite(motorL_dir1, HIGH);
    digitalWrite(motorL_dir2, LOW);
    digitalWrite(motorR_dir1, HIGH);
    digitalWrite(motorR_dir2, LOW);
    analogWrite(motorL,175);
    analogWrite(motorR,175);
    delay(500);
  }
}

void Stop()
{
  digitalWrite(motorL_dir1, LOW);
  digitalWrite(motorL_dir2, LOW);
  digitalWrite(motorR_dir1, LOW);
  digitalWrite(motorR_dir2, LOW);
  delay(100);
}

void pingSensor() //read sensor
{
  pinMode(pingPin,OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(5);
  digitalWrite(pingPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  
  //read from the same pin. The duration of the HIGH pulse tells distance
  pinMode(pingPin,INPUT);
  duration=pulseIn(pingPin,HIGH);
  
  //convert the time into distance
  cm=microsecondsToCM(duration);
  Serial.print(cm);
  Serial.println("cm");
}

long microsecondsToCM(long microseconds)
{
  return microseconds/58;
}
 
void readSensor() // read sensor in all 3 directions
{
   //0 degrees is on the right
  myservo.write(85);
  delay(250);
  pingSensor();
  distF=cm;

  
   //sweep to left
/*  for (int i=85; i<180; i++)
  {
    myservo.write(i);
    
    int degreeread = myservo.read();
    Serial.print("degrees");
    Serial.println(degreeread);
  }
  */
  myservo.write(180);
  delay(250);
  pingSensor();
  distL=cm;

  //sweep to right
/*  for (int i=180; i>0; i--)
  {
    myservo.write(i);

    int degreeread = myservo.read();
    Serial.print("degrees");
    Serial.println(degreeread);
  }
  */
  myservo.write(0);
  delay(400);
  pingSensor();
  distR=cm;

  // sweep back to center
 /* for (int i=0; i<85; i++)
  {
    myservo.write(i);
    
    int degreeread = myservo.read();
    Serial.print("degrees");
    Serial.println(degreeread);
  }
 */ 
 myservo.write(85);   
}

// Encoder right wheel
void isr_process_encoderR(void)
{
  volatile int encR1 = digitalRead(encR_ch1);
  volatile int encR2 = digitalRead(encR_ch2);
   
  if (encR2 == HIGH && encR1 == LOW)
  {
    enc_countR--;
  }
  else if (encR2 == LOW && encR1 == HIGH)
  {
    enc_countR--;
  }
  else if (encR2 == LOW && encR1 == LOW)
  {
    enc_countR++;
  }
  else if (encR2 == HIGH && encR1 == HIGH)
  {
    enc_countR++;
  } 
}

// Encoder left wheel
// Forward increments the encoder
void isr_process_encoderL(void)
{
   volatile int encL1 = digitalRead(encL_ch1);
   volatile int encL2 = digitalRead(encL_ch2);  
   
  if (encL2 == HIGH && encL1 == LOW)
  {
    enc_countL++;
  }
  else if (encL2 == LOW && encL1 == HIGH)
  {
    enc_countL++;
  }
  else if (encL2 == LOW && encL1 == LOW)
  {
    enc_countL--;
  }
  else if (encL2 == HIGH && encL1 == HIGH)
  {
    enc_countL--;
  }  
}

// Encoder arm
void isr_process_encoderA(void)
{
   volatile int encA1 = digitalRead(encA_ch1);
   volatile int encA2 = digitalRead(encA_ch2);
   
  if (encA2 == HIGH && encA1 == LOW)
  {
    enc_countA++;
  }
  else if (encA2 == LOW && encA1 == HIGH)
  {
    enc_countA++;
  }
  else if (encA2 == LOW && encA1 == LOW)
  {
    enc_countA--;
  }
  else if (encA2 == HIGH && encA1 == HIGH)
  {
    enc_countA--;
  }
}

void characterMap() // manual control to pick up cup
{  
 if (command == 'e' || command == 'E')//  open gripper
  {
     digitalWrite(motorG_dir1, HIGH);
     digitalWrite(motorG_dir2, LOW);  
     analogWrite(motorG,100);
     delay(25);
     digitalWrite(motorG_dir1, LOW);
  }
  else if (command == 'q' || command == 'Q') // close gripper
  {
     digitalWrite(motorG_dir1, LOW);
     digitalWrite(motorG_dir2, HIGH);  
     analogWrite(motorG,150);
     delay(25);
     digitalWrite(motorG_dir2, LOW);
  }
 else if (command == 'R' || command == 'r') // raise arm 
  {
     digitalWrite(motorA_dir1, HIGH);
     digitalWrite(motorA_dir2, LOW);  
     analogWrite(motorA,125);
     delay(25);
     digitalWrite(motorA_dir1, LOW);
  }
 else if (command == 'F' || command == 'f') // lower arm
 {
     digitalWrite(motorA_dir1, LOW);
     digitalWrite(motorA_dir2, HIGH);  
     analogWrite(motorA,50);
     delay(25);
     digitalWrite(motorA_dir2, LOW);
 }
 else if (command == 'w' || command == 'W') // go forwards
 { 
   digitalWrite(motorL_dir1, HIGH);
   digitalWrite(motorL_dir2, LOW);
   digitalWrite(motorR_dir1, HIGH);
   digitalWrite(motorR_dir2, LOW);  
   analogWrite(motorL,150);
   analogWrite(motorR,150);
   delay(25);
   digitalWrite(motorL_dir1, LOW);
   digitalWrite(motorR_dir1, LOW);  
}
else if (command == 's' || command == 'S') // go backwards
{
  digitalWrite(motorL_dir1, LOW);
  digitalWrite(motorL_dir2, HIGH);
  digitalWrite(motorR_dir1, LOW);
  digitalWrite(motorR_dir2, HIGH);  
  analogWrite(motorL,150);
  analogWrite(motorR,150);
  delay(25);
  digitalWrite(motorL_dir2, LOW);
  digitalWrite(motorR_dir2, LOW); 
   
}
else if (command == 'd' || command == 'D') // turn right
{
  turnRightSlow(10);
}
else if (command == 'a' || command == 'A') // turn left
{
  turnLeftSlow(10);
}
}

void Bluetooth() // talk to bluetooth and let us manually control
{
 while (command != 'L' && command !='l')
 {
 while (Serial.available()>0)
 {
  Serial3.write(Serial.read());
 }
 while (Serial3.available()>0)
 {
  //Serial.write(Serial3.read());
  // initial command above
  command = Serial3.read();
  Serial.print("command:");
  Serial.println(command);
  characterMap();
 }
 }
}
