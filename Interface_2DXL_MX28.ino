/* This example is for controlling two dynamixel motors MX-28AT, using Protocol V1.0
 * User can send desired positions or velocoties to the motors via Serial Port of OpenCM board
 * The board will send feedback to the PC via Serial Port including: Q1,Q2,QP1,QP2
 * 
 * Author: N.D.Quan - 30/10/2021
 */
 
 #include <DynamixelSDK.h>
#include <math.h>

// MX-series Control table address
#define ADDR_MX_CW                  6
#define ADDR_MX_CCW                 8
#define ADDR_MX_TORQUE_ENABLE       24                 // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION       30
#define ADDR_MX_GOAL_SPEED          32
#define ADDR_MX_PRESENT_SPEED       38
#define ADDR_MX_PRESENT_POSITION    36
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
uint8_t DXL_ID[2] = {1, 2};
int16_t dxl_GOAL_SPEED[2] = {0, 0};         // Goal speed, unit = 0.114 rpm 
int16_t dxl_GOAL_POSITION[2] = {0,0};            // Goal position, unit = 0.088deg
int idx = 0;
uint8_t dxl_error = 0;                          // Dynamixel error
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
int dxl_comm_result[2] = {COMM_TX_FAIL,COMM_TX_FAIL};             // Communication result
int16_t dxl_PRESENT_SPEED[2] = {0, 0};               // Present speed, unit = 0.114rpm
int16_t dxl_PRESENT_POSITION[2] = {0, 0};               // Present position = 0.088deg


// SERIAL INTERRUPT
String inputString1 = "";         // a String to hold set-point data for Motor 1
String inputString2 = "";         // a String to hold set-point data for Motor 2
String BUFF;
boolean stringComplete = false;  // whether the string is complete
short ck = 0; // to check the format of incoming String

volatile int MODE, MOVE;
volatile int16_t desired_P1=0, desired_P2=0;
volatile int16_t desired_V1=0, desired_V2=0;
boolean cont = true; 

unsigned long microsPerReading, microsPrevious, microsNow, dT;

void setup() {

  // Setup COM-PC connection
  Serial.begin(500000);
  inputString1.reserve(50); // reserve 200 bytes for the inputString:
  inputString2.reserve(50); // reserve 200 bytes for the inputString:
 
  // Setup COM-DXL connection
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  // Open port
  portHandler->openPort();
  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE);
  // Enable torques
  delay(1000);
  Enable_dxl();
  MODE = 0;
  ck = 0;
  microsPerReading = 1000000 / 100; // 100Hz
  microsPrevious = micros();
}

void send_feedback()
{
  for(idx = 0; idx<2;idx++)
  {
    // Read present speeds
    packetHandler->read2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_PRESENT_SPEED, (uint16_t*)&dxl_PRESENT_SPEED[idx], &dxl_error);
    // Read present positions
    packetHandler->read2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_PRESENT_POSITION, (uint16_t*)&dxl_PRESENT_POSITION[idx], &dxl_error);
  }
  BUFF = "";  
  BUFF += String(dxl_PRESENT_POSITION[0]);
  BUFF +=",";
  BUFF += String(dxl_PRESENT_POSITION[1]);
  BUFF +=",";
  BUFF += String(dxl_PRESENT_SPEED[0]);
  BUFF +=",";
  BUFF += String(dxl_PRESENT_SPEED[1]);
  BUFF +="\r";
  Serial.println(BUFF);
}

void Stop_dxl()
{
  for(idx = 0;idx<2;idx++)
    packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_GOAL_SPEED, 0, &dxl_error);
}

void Enable_dxl()
{
  for(idx = 0;idx<2;idx++)
     packetHandler->write1ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

void Disable_dxl()
{
  for(idx = 0;idx<2;idx++)
     packetHandler->write1ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
}

void Move_vel()
{
  for(idx = 0; idx<2;idx++)
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_GOAL_SPEED, dxl_GOAL_SPEED[idx], &dxl_error);
}

void Move_pos()
{
  for(idx = 0; idx<2;idx++)
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_GOAL_POSITION, dxl_GOAL_POSITION[idx], &dxl_error);
}

void Speed_mode()
{
  for(idx = 0; idx<2;idx++)
  {
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_CW, 0, &dxl_error);
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_CCW, 0, &dxl_error);
  }
}

void Position_mode()
{
  for(idx = 0; idx<2;idx++)
  {
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_CW, 0, &dxl_error);
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_CCW, 4095, &dxl_error);
  }
}

void MultiTurn_mode()
{
  for(idx = 0; idx<2;idx++)
  {
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_CW, 4095, &dxl_error);
     packetHandler->write2ByteTxRx(portHandler, DXL_ID[idx], ADDR_MX_CCW, 4095, &dxl_error);
  }
}

//===============================================
//===============================================
void loop() {
  microsNow = micros();     
  dT = microsNow - microsPrevious;
  if (dT >= microsPerReading) {
    // Update setpoint / control_mode
    if (stringComplete) {
      if ((MODE == 1) or (MODE==2))
      {
        desired_P1 = inputString1.toInt();
        desired_P2 = inputString2.toInt();
        dxl_GOAL_POSITION[0] = desired_P1;
        dxl_GOAL_POSITION[1] = desired_P2;
  //      Serial.print("P1 = ");
  //      Serial.print(desired_P1);
  //      Serial.println(" deg");
  //      Serial.print("P2 = ");
  //      Serial.print(desired_P2);
  //      Serial.println(" deg");
      }
      else if (MODE == 0)
      {
        desired_V1 = inputString1.toInt();
        desired_V2 = inputString2.toInt();
        dxl_GOAL_SPEED[0] = desired_V1;
        dxl_GOAL_SPEED[1] = desired_V2;
  //      Serial.print("V1 = ");
  //      Serial.print(desired_V1);
  //      Serial.println(" rpm");
  //      Serial.print("V2 = ");
  //      Serial.print(desired_V2);
  //      Serial.println(" rpm");
      }
      // clear the string:
      inputString1 = "";
      inputString2 = "";
      stringComplete = false;
    }
    // Control
    if (MOVE == 1)
    {
      if (MODE==0)
        Move_vel();
      else if ((MODE == 1) || (MODE == 2))
        Move_pos();
    }
  
    // SEND FEEDBACK
    send_feedback();
    // Update sample time
    microsPrevious = microsNow;
  } 
}



void serialEvent() {
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == 'M') // MOVE command
    {
      MOVE = 1;
    }
    else if (inChar == 'S') // STOP command
    {
      MOVE = 0;
      Stop_dxl();
    }
    else if (inChar == 'F') // FREE DRIVE
    {
      MODE= 3;
      Disable_dxl();
      delay(100);
    }
    else if (inChar == 'T') // Multi-turns
    {
      Stop_dxl();
      delay(100);
      MODE = 2;    // Switch to position control mode 
      dxl_GOAL_POSITION[0] = 0;
      dxl_GOAL_POSITION[1] = 0;
      MultiTurn_mode();      
      delay(100);
      Enable_dxl(); 
      delay(100);      
    }
    else if (inChar == 'P') // Position control mode
    {
      Stop_dxl();
      delay(100);
      MODE = 1;    // Switch to position control mode 
      dxl_GOAL_POSITION[0] = 0;
      dxl_GOAL_POSITION[1] = 0;
      Position_mode();      
      delay(100);
      Enable_dxl(); 
      delay(100);       
    }
    else if (inChar == 'V') // Velocity control mode
    {
      Stop_dxl();
      delay(100);
      MODE = 0;   // Switch to velocity control mode
      dxl_GOAL_SPEED[0] = 0;
      dxl_GOAL_SPEED[1] = 0;
      Speed_mode();
      delay(100);
      Enable_dxl();
      delay(100);
    }
    else
    {
      // add it to the inputString:
      if (ck==0)
        inputString1 += inChar;
      else
        inputString2 += inChar;
      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
      if (inChar == '#') {
        ck = ck + 1;
        if (ck==2)
        {
          stringComplete = true;
          ck = 0;
        }
      }
    }
  }
}
