// Author: Uriel Martinez-Hernandez
// Date: 7 August 2017

// manual and transparency control with PID control of DC motor for AFO
// latest modification: 2nd August 2017

#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

#define MAX_SIZE_COMMAND 20
#define MAX_NUM_PARAMETERS 20
#define MAX_ENCODER 128         // maximum number of positions from rotary encoder
#define MAX_BITS 8              // number of bits provided by the rotary encoder 
#define linPotPin A10           // input from motorised linear potentiometer


int binaryValue[MAX_BITS];

int lookup_table[MAX_ENCODER] = {127, 63, 62, 58, 56, 184, 152, 24,
                         8, 72, 73, 77, 79, 15, 47, 175,
                         191, 159, 31, 29, 28, 92, 76, 12,
                         4, 36, 164, 166, 167, 135, 151, 215,
                         223, 207, 143, 142, 14, 46, 38, 6,
                         2, 18, 82, 83, 211, 195, 203, 235,
                         239, 231, 199, 71, 7, 23, 19, 3,
                         1, 9, 41, 169, 233, 225, 229, 245,
                         247, 243, 227, 163, 131, 139, 137, 129,
                         128, 132, 148, 212, 244, 240, 242, 250,
                         251, 249, 241, 209, 193, 197, 196, 192,
                         64, 66, 74, 106, 122, 120, 121, 125,
                         253, 252, 248, 232, 224, 226, 98, 96,
                         32, 33, 37, 53, 61, 60, 188, 190,
                         254, 126, 124, 116, 112, 113, 49, 48,
                         16, 144, 146, 154, 158, 30, 94, 95};


int inD2 = 2;
int inD3 = 3;
int inD4 = 4;
int inD5 = 5;
int inD6 = 6;
int inD7 = 7;
int inD8 = 8;
int inD9 = 9;

double currentSensorMotorPosition = 0;
double mapCurrentSensorMotorPosition = 0;
double constrainCurrentSensorMotorPosition = 0;

double inputPositionPID;
double outputPositionPID;
double setTargetPID;
double Kp_position = 2;
double Ki_position = 0;
double Kd_position = 0.05;
double assistiveMotorPositionError = 0;

char commands_char[MAX_NUM_PARAMETERS][MAX_SIZE_COMMAND];
int count = 0;
int ncommand = 0;
char current_char;
bool commandStatus = false;
int calibrationSpeed;
int assistiveSpeed;
int transparencySpeed;


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *assistiveMotor = AFMS.getMotor(1);
Adafruit_DCMotor *sensorMotor = AFMS.getMotor(2);

PID assistiveMotorPID(&inputPositionPID, &outputPositionPID, &setTargetPID, Kp_position, Ki_position, Kd_position, DIRECT);


void setup() {
  // put your setup code here, to run once:

  pinMode(inD2, INPUT);
  pinMode(inD3, INPUT);
  pinMode(inD4, INPUT);
  pinMode(inD5, INPUT);
  pinMode(inD6, INPUT);
  pinMode(inD7, INPUT);
  pinMode(inD8, INPUT);
  pinMode(inD9, INPUT);

  pinMode(linPotPin, INPUT);
  
  calibrationSpeed = 0;
  assistiveSpeed = 0;
  transparencySpeed = 0;

  AFMS.begin();
  assistiveMotor->setSpeed(0);
  sensorMotor->setSpeed(0);

  assistiveMotorPID.SetOutputLimits(-50, 50);
  assistiveMotorPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

   if( Serial.available() > 0 )
   {
        commandStatus = readCommands();
        replyAcknowledge(commandStatus);
        
        if( commandStatus == true )
            replyAcknowledge(executeCommand(commands_char));

        Serial.flush();
    }
}


bool readCommands()
{
    for( int i = 0; i < MAX_NUM_PARAMETERS; i++ )
    {
        for( int j = 0; j < MAX_SIZE_COMMAND; j++ )
            commands_char[i][j] = '\0';
    }
    
    count = 0;
    ncommand = 0;
    
    do
    {
        current_char = Serial.read();
        
        delay(3);
    
        if( current_char != ' ' )
        {
            commands_char[ncommand][count] = current_char;
            count++;
        }
        else
        {
            commands_char[ncommand][count] = '\0';
            count = 0;
            ncommand++;                
        }
        
    }while( current_char != '\r' );
    
    return commandList(commands_char[0]);
}

/* Function for execution of commands */
bool executeCommand(char cmdReceived[][MAX_SIZE_COMMAND])
{
    /* Calibration of X axis */
    if( !strcmp(cmdReceived[0],"@CALIBRATE\r") )
    {
      return true;
    }
    /* Allows to move the motor step by step to a desired position. */
    /* This can be used to adjust initial position of the Assistive Motor. */
    else if( !strcmp(cmdReceived[0],"@MOVESTEP") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
          int incMotorPosition = atoi(cmdReceived[1]);
          int currentMotorPosition = getPositionMotorFeedback();
          int targetMotorPosition = currentMotorPosition + incMotorPosition;
          if( incMotorPosition < -127 || incMotorPosition > 127 )
          {
             Serial.print("Movement value out of range [-127 to 127]");
             return false;
          }
          else if( targetMotorPosition < 0 || targetMotorPosition > 127 )
          {
             Serial.print("Target motor position out of range [0 to 127]");
             return false;
          }
          else
          {
            if( calibrationSpeed > 0 )
            {
              bool motorReadyInPosition = false;

              do
              {
                Serial.print("Increment position: ");
                Serial.print(incMotorPosition);
                Serial.print("\t");
                Serial.print("Target position motor: ");
                Serial.print(targetMotorPosition);
                Serial.print("\t");
                
                setTargetPID = targetMotorPosition;
                inputPositionPID = getPositionMotorFeedback();
                assistiveMotorPID.Compute();
        
                Serial.print("Current motor position: ");
                Serial.print(inputPositionPID);
                Serial.print("\t");

                Serial.print("PID output position: ");
                Serial.print(outputPositionPID);
                Serial.print("\t");
        
                if( outputPositionPID > 0 )
                {
                  //assistiveMotor->setSpeed(calibrationSpeed);
                  //assistiveMotor->run(FORWARD);
                  Serial.println("Motor action: move forward");
                }
                else if( outputPositionPID < 0 )
                {
                  //assistiveMotor->setSpeed(calibrationSpeed);
                  //assistiveMotor->run(BACKWARD);
                  Serial.println("Motor action: move backward");
                }
                else
                {
                  motorReadyInPosition = true;
                  //assistiveMotor->setSpeed(0);
                  Serial.println("Motor action: stop motor");
                }
              }while( !motorReadyInPosition );
            }
          }
        }
        else
          return false;
    }
    /* Calibration of Z axis */
    else if( !strcmp(cmdReceived[0],"@TRANSPARENCY") )
    {
        if( !strcmp(cmdReceived[1],"ON\r") )
        {
            if( transparencySpeed > 0)
            {
                do
                {
                    if( Serial.available() > 0 )
                    {
                        commandStatus = readCommands();
                        replyAcknowledge(commandStatus);
                    }

                    currentSensorMotorPosition = analogRead(linPotPin);
                    mapCurrentSensorMotorPosition = map(currentSensorMotorPosition, 0, 1023, 0, 127);
                    constrainCurrentSensorMotorPosition = constrain(mapCurrentSensorMotorPosition, 0, 127);

                    Serial.print("Mapped sensor motor: ");
                    Serial.print(constrainCurrentSensorMotorPosition);
                    Serial.print("\t");
                    
                    setTargetPID = constrainCurrentSensorMotorPosition;
                    inputPositionPID = getPositionMotorFeedback();

                    Serial.print("Feedback position: ");
                    Serial.print(getPositionMotorFeedback());
                    Serial.print("\t\t");
                      
                    assistiveMotorPID.Compute();
                    
                    Serial.print("PID output position: ");
                    Serial.print(outputPositionPID);
                    Serial.print("\t");

                    if( outputPositionPID > 0 )
                    {
                      Serial.println("Motor action: move forward");
                    }
                    else if( outputPositionPID < 0 )
                    {
                      Serial.println("Motor action: move backward");
                    }
                    else
                    {
                      Serial.println("Motor action: stop motor");
                    }
                                        
                }while( strcmp(commands_char[0],"@TRANSPARENCY") || strcmp(commands_char[1],"OFF\r") );

                Serial.println("End of TRANSPARENCY MODE");
            }
            return true;
        }
        else if( !strcmp(cmdReceived[1],"OFF\r") )
        {
            Serial.println("End of TRANSPARENCY MODE");
            return true;
        }
        else
            return false;
    }
    /* Calibration of Z axis */
    else if( !strcmp(cmdReceived[0],"@ASSISTANCE") )
    {
        if( !strcmp(cmdReceived[1],"ON\r") )
        {
            if( assistiveSpeed > 0)
            {
                do
                {
                    if( Serial.available() > 0 )
                    {
                        commandStatus = readCommands();
                        replyAcknowledge(commandStatus);
                    }

                    /**
                     *  Add here the code to apply assistance in dorsiflexion. 
                     *  For the assistance, you need to define the angle or position to lift up the foot.
                     *  The foot needs to keep in the defined position until the "@ASSISTANCE OFF" 
                     *  command is received. Then, the assistance needs to be released by the command
                     *  "@TRANSPARENCY", which allow the human foot to move freely.
                     */
                     
                }while( strcmp(commands_char[0],"@ASSISTANCE") || strcmp(commands_char[1],"OFF\r") );
            }
        }

        return true;
    }
    /* Calibration of T axis */
    else if( !strcmp(cmdReceived[0],"@SETCALIBSPEED") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            calibrationSpeed = atoi(cmdReceived[1]);            
            return true;
        }
        else
          return false;
    }
    /* Calibration of T axis */
    else if( !strcmp(cmdReceived[0],"@SETASSISTSPEED") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            assistiveSpeed = atoi(cmdReceived[1]);            
            return true;
        }
        else
          return false;
    }
    /* Calibration of T axis */
    else if( !strcmp(cmdReceived[0],"@SETTRANSSPEED") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            transparencySpeed = atoi(cmdReceived[1]);            
            return true;
        }
        else
          return false;
    }
    /* Calibration of Z axis */
    else if( !strcmp(cmdReceived[0],"@INFO\r") )
    {
      Serial.println("======================================");
      Serial.println("AFO INFO version 0.1");
      Serial.println("======================================");
      Serial.print("Calibration speed: ");
      Serial.println(calibrationSpeed);
      Serial.print("Assistive speed: ");
      Serial.println(assistiveSpeed);
      Serial.print("Transparency speed: ");
      Serial.println(transparencySpeed);
      Serial.print("Current assitive motor position: ");
      Serial.println(getPositionMotorFeedback());
      Serial.print("Current sensor motor position: ");
      Serial.println(analogRead(linPotPin));
      Serial.println("======================================");
      return true;
    }
    else
      return false;
}


/* Send reply ACK/NACK to client */
void replyAcknowledge(bool cmdStatus)
{
//    delay(100);
    if( cmdStatus == true )
        sendACK();
    else
        sendNACK();

    Serial.flush();
}

/* Print ACK message */
void sendACK()
{
    Serial.println("ACK");
}

/* Print NACK message */
void sendNACK()
{
    Serial.println("NACK");
}

/* Check the command received */
bool commandList(char *cmdReceived)
{
    char *commandArray[] = {"@CALIBRATE\r","@MOVESTEP","@SETCALIBSPEED","@SETASSISTSPEED","@SETTRANSSPEED","@INFO\r","@TRANSPARENCY","@ASSISTANCE"};
    int ncommands = 8;
    
    for( int i = 0; i < ncommands; i++ )
    {
        if( !strcmp(commandArray[i], cmdReceived) )
            return true;
    }
    
    return false;
}


double getPositionMotorFeedback()
{
  double feedbackPosition = 0;
  
  binaryValue[7] = digitalRead(inD2);
  binaryValue[6] = digitalRead(inD3);
  binaryValue[5] = digitalRead(inD4);
  binaryValue[4] = digitalRead(inD5);
  binaryValue[3] = digitalRead(inD6);
  binaryValue[2] = digitalRead(inD7);
  binaryValue[1] = digitalRead(inD8);
  binaryValue[0] = digitalRead(inD9);

  feedbackPosition = getOutputPositionMotor(lookup_table,binaryToDecimal(binaryValue));

  return feedbackPosition;
}

int binaryToDecimal(int val[])
{
  float decimalNumber = 0.0;

  for(int i = 0; i < MAX_BITS; i++ )
    decimalNumber =  decimalNumber + ( pow(2,i) * val[(MAX_BITS-i)-1]);

  return round(decimalNumber);
}

int getOutputPositionMotor(int table[], int val)
{

  int index = 0;
  
  while( index < MAX_ENCODER && val != table[index] )  
    index = index + 1; 

  return index;
}

