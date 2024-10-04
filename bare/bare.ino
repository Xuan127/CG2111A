#include <serialize.h>
#include <math.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"

#define s0 (1 << DDB0)        //Module pins wiring
#define s1 (1 << DDB1)
#define s2 (1 << DDB4)
#define s3 (1 << DDB5)
#define out (1 << DDD7)

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;
/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      175

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          21.2
volatile unsigned long LT = 0;
volatile unsigned long RT = 0;
// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  (1 << DDD5)   // Left forward pin
#define LR                  (1 << DDD6)   // Left reverse pin
#define RF                  (1 << DDB3)  // Right forward pin
#define RR                  (1 << DDB2)  // Right reverse pin

volatile long red;
volatile long green;
volatile long blue;


/*
 *    Alex's State Variables
 */
#define pi  3.141592654
// alex length and breadth in cm
#define ALEX_LENGTH 17
#define ALEX_BREADTH 11

// alex diagonal. we compute and store this once
// since it is expensive to compute and really does not change. 
float alexDiagonal = 0.0;

//alex turning circumference, calculated once
float alexCirc = 0.0;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;
/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks; 
  statusPacket.params[1] = rightForwardTicks; 
  statusPacket.params[2] = leftReverseTicks; 
  statusPacket.params[3] = rightReverseTicks; 
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = red;
  statusPacket.params[11] = green;
  statusPacket.params[12] = blue;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char  *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  LT++;
  //Serial.print("Left: ");
  //Serial.println(LT);
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  //Serial.print("LEFT: ");
  //Serial.println(WHEEL_CIRC *leftTicks / COUNTS_PER_REV);
}

void rightISR()
{
  RT++;
  //Serial.print("Right: ");
  //Serial.println(RT);
  if (dir == FORWARD) {
    rightForwardTicks++;
  }
  else if (dir == BACKWARD) {
    rightReverseTicks++;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
  //Serial.print("RIGHT: ");
  //Serial.println(WHEEL_CIRC*rightTicks  / COUNTS_PER_REV);
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
  
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect){
  leftISR();
}

ISR(INT1_vect){
  rightISR();
}


// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  //Serial.begin(9600);
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0C = 0b00000110;
  UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;
  while ((UCSR0A & 0b10000000) == 1) {
     buffer[count++] = UDR0;
  }    

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  //Serial.write(buffer, len);
  for (int i = 0; i < len; ++i){
    while ((UCSR0A & 0b00100000) == 0);
    UDR0 = buffer[i];
  }
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.

void forward(float dist, float speed, int a)
{
  dir = FORWARD;
  int val1, val2;
  if (a = 0){
    val1 = pwmVal(speed);
    val2 = pwmVal(speed + 10);
  }else{
    val1 = pwmVal(speed);
    val2 = pwmVal(speed*1.03);
  }
  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.
  if (dist > 0) {
    deltaDist = dist;
  } else {
    deltaDist = 9999999;
  }
  newDist = forwardDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  TCCR0A = 0b00100001;
  TCCR1A = 0b00000001;
  TCCR2A = 0b10000001;
  OCR0B = val2;
  OCR2A = val1;
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed, int a)
{
  dir = BACKWARD;
  int val1, val2;
  if (a = 0){
    val1 = pwmVal(speed);
    val2 = pwmVal(speed*1.3);
  }else{
    val1 = pwmVal(speed);
    val2 = pwmVal(speed*1.03);
  }
  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.
  if (dist > 0) {
    deltaDist = dist;
  }
  else {
    deltaDist = 999999;
  }
  newDist = reverseDist + deltaDist;
  
  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  TCCR0A = 0b10000001;
  TCCR1A = 0b00100001;
  TCCR2A = 0b00000001;
  OCR0A = val2;
  OCR1B = val1;
}

unsigned long computeDeltaTicks(float ang) {
  // we will assume that angular distance moved = linear distance moved in one wheel
  // revolution. This is probably incorrect but simplifies calculation
  // # of wheel revs to make one full 360 turn is alexCirc / WHEELCIRC
  // this is for 360 degrees. for ang degrees it will be (ang * alexCirc) / (360. 0 * WHEEL_CIRC)
  // to convert to ticks, we multuiply by COUNTS_PER_REV

  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed, int a)
{

  if (ang == 0) {
    deltaTicks = 9999999;
  }
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  dir = LEFT;
  int val1, val2;
  if (a = 0){
    val1 = pwmVal(speed);
    val2 = pwmVal(speed *1.3);
  }else{
    val1 = pwmVal(speed);
    val2 = pwmVal(speed*1.03);
  }

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.

  TCCR0A = 0b10000001;
  TCCR1A = 0b00000001;
  TCCR2A = 0b10000001;
  OCR0A = val2;
  OCR2A = val1;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed, int a)
{
  if (ang == 0) {
    deltaTicks = 9999999;
  }
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
  dir = RIGHT;
  int val1, val2;
  if (a = 0){
    val1 = pwmVal(speed);
    val2 = pwmVal(speed*1.12);
  }else{
    val1 = pwmVal(speed);
    val2 = pwmVal(speed*1.03);
  }

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.

  TCCR0A = 0b00100001;
  TCCR1A = 0b00100001;
  TCCR2A = 0b00000001;
  OCR0B = val2;
  OCR1B = val1;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  TCCR0A = 0b00000001;
  TCCR1A = 0b00000001;
  TCCR2A = 0b00000001;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  switch(which)
  {
    case 0:
      clearCounters();
      break;

    case 1:
      leftForwardTicks=0;
      break;

    case 2:
      rightForwardTicks=0;
      break;

    case 3:
      leftReverseTicks=0;
      break;

    case 4:
      rightReverseTicks=0;
      break;

    case 5:
      forwardDist=0;
      break;

    case 6:
      reverseDist=0;
      break;
  }
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD_S:
        sendOK();
        forward((float) command->params[0], (float) command->params[1], 0);
    case COMMAND_FORWARD_L:
        sendOK();
        forward((float) command->params[0], (float) command->params[1], 1);
      break;
    case COMMAND_REVERSE_S:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1], 0);
    case COMMAND_REVERSE_L:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1], 1);
      break;
    case COMMAND_TURN_LEFT_S:
        sendOK();
        left((float) command->params[0], (float) command->params[1], 0);
    case COMMAND_TURN_LEFT_L:
        sendOK();
        left((float) command->params[0], (float) command->params[1], 1);
      break;
    case COMMAND_TURN_RIGHT_S:
        sendOK();
        right((float) command->params[0], (float) command->params[1], 0);
    case COMMAND_TURN_RIGHT_L:
        sendOK();
        right((float) command->params[0], (float) command->params[1], 1);
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_GET_STATS:
      getColour();
      sendStatus();
    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setupCS() 
{
   DDRB |= (s0 | s1 | s2 | s3);
   DDRD &= ~out;
   Serial.begin(9600);   
   
   PORTB |= (s0 | s1); //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
   //digitalWrite(s0,HIGH);
   //digitalWrite(s1,HIGH);
}

void getColour( ){
  int data = 0;
   PORTB &= ~(s2 | s3);
   //digitalWrite(s2,LOW);        //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green
   //digitalWrite(s3,LOW);
   //Serial.print("Red value= "); 
   data=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration      and stops when "out" is HIGH again
   red = map(data,35,185,255,0);
   
   //Serial.print(map(data,60,15,0,100));        
   //Serial.print(data);   
   //Serial.print("\t");          
   delay(100);
                      
   PORTB &= ~s2;
   PORTB |= s3;
   //digitalWrite(s2,LOW);
   //digitalWrite(s3,HIGH);
   //Serial.print("Blue value= ");
   data=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   blue = map(data,25,220,255,0);
   
   //Serial.print(map(data,80,11,0,100));          
   //Serial.print(data);
   //Serial.print("\t");          
   delay(100);

   PORTB |= s2;
   PORTB |= s3;
   //digitalWrite(s2,HIGH);
   //digitalWrite(s3,HIGH);
   //Serial.print("Green value= ");
   data=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   green = map(data,45,260,255,0);
   
   //Serial.print(map(data,80,20,0,100));          
   //Serial.print(data);
   //Serial.print("\t");          
   delay(100);

   //Serial.println();

   delay(200);
 }
   

void setup() {
  // put your setup code here, to run once:

  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = pi * alexDiagonal;
  
  setupCS();
  
  cli();
  TCNT0 = 0;
  TCNT1 = 0;
  TCNT2 = 0;
  TCCR0A = 0b00000001; // Set OCOM0A to 10 and WGM to 01
  TCCR1A = 0b00000001;
  TCCR2A = 0b00000001;
  //TIMSK0 |= 0b10; // Enable Int for Output Compare Match
  //TCCR0B = 0b00000011; // Set clk source to clk/64
  //Set Output
  DDRD |= (LF | LR);
  DDRB |= (RF | RR);
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

//forward(0, 100);

// Uncomment the code below for Week 8 Studio 2


 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else {
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      }    
  }   
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
        if (reverseDist > newDist) {
          deltaDist = 0;
          newDist = 0;
          stop();
        }
      }
    else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns > targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
    
      }
    }
    else if (dir == RIGHT) {
        if (rightReverseTicksTurns > targetTicks) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
    }
    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
