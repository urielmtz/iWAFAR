// Author: Uriel Martinez-Hernandez
// Date: 7 August 2017

// manual and transparency control with PID control of DC motor for AFO
// latest modification: 2nd August 2017

#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
#include <PinChangeInt.h>

#define MAX_SIZE_COMMAND 20
#define MAX_NUM_PARAMETERS 20
#define MAX_ENCODER 128         // maximum number of positions from rotary encoder
#define MAX_BITS 8              // number of bits provided by the rotary encoder 

#define linPotPin A10           // input from motorised linear potentiometer

#define inD2 10
#define inD3 11
#define inD4 12
#define inD5 13
#define inD6 MISO
#define inD7 MOSI
#define inD8 SCK
#define inD9 SS

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


double currentSensorMotorPosition = 0;
double mapCurrentSensorMotorPosition = 0;
double constrainCurrentSensorMotorPosition = 0;

volatile double tempInputEncoder = 0;
double inputPositionPID;
double outputPositionPID;
double setTargetPID = 0;
double Kp_position = 10.0;
double Ki_position = 20.0;
double Kd_position = 0.01;
double assistiveMotorPositionError = 0;

char commands_char[MAX_NUM_PARAMETERS][MAX_SIZE_COMMAND];
int count = 0;
int ncommand = 0;
char current_char;
bool commandStatus = false;
int assistiveSpeed;
int mapInputPositionPID = 0;

bool positionReady = false;
bool limitsReached = false;

int minInputMapSlider = 256;
int maxInputMapSlider = 768;
int minValMapSlider = 10;
int maxValMapSlider = 120;
int minValMapEncoder = 10;
int maxValMapEncoder = 120;
int minLimitVal = 10;
int maxLimitVal = 120;

int midMotorPoint = 0;
int assistiveMotorPoint = 0;
int releaseMotorPoint = 0;

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

  PCintPort::attachInterrupt(inD2, rotEncDig1, CHANGE);
  PCintPort::attachInterrupt(inD3, rotEncDig2, CHANGE);
  PCintPort::attachInterrupt(inD4, rotEncDig3, CHANGE);
  PCintPort::attachInterrupt(inD5, rotEncDig4, CHANGE);
  PCintPort::attachInterrupt(inD6, rotEncDig5, CHANGE);
  PCintPort::attachInterrupt(inD7, rotEncDig6, CHANGE);
  PCintPort::attachInterrupt(inD8, rotEncDig7, CHANGE);
  PCintPort::attachInterrupt(inD9, rotEncDig8, CHANGE);

  pinMode(linPotPin, INPUT);
  
 
  AFMS.begin();
  assistiveMotor->setSpeed(0);
  sensorMotor->setSpeed(0);

  assistiveSpeed = 0;

  assistiveMotorPID.SetSampleTime(1);   // 1 msec
  assistiveMotorPID.SetMode(AUTOMATIC);
  assistiveMotorPID.SetOutputLimits(-assistiveSpeed, assistiveSpeed);

  Serial.begin(115200);
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


void rotEncDig1()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
}

void rotEncDig2()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
}

void rotEncDig3()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
}

void rotEncDig4()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
}

void rotEncDig5()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
}

void rotEncDig6()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
}

void rotEncDig7()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
}

void rotEncDig8()
{
    inputPositionPID = getPositionMotorFeedback();
    if( setTargetPID == inputPositionPID )
        positionReady = true;
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
    /* Calibration of Mid Motor Point */
    if( !strcmp(cmdReceived[0],"@CALMIDPOINT") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            int pointValue = atoi(cmdReceived[1]);            
            midMotorPoint = pointValue;
//            midMotorPoint = getPositionMotorFeedback();
            return true;
        }
        else
          return false;
    }
    /* Calibration of Assistive Motor Point */
    if( !strcmp(cmdReceived[0],"@CALASSISTIVEPOINT") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            int pointValue = atoi(cmdReceived[1]);            
            assistiveMotorPoint = pointValue;
//            assistiveMotorPoint = getPositionMotorFeedback();
            return true;
        }
        else
          return false;
    }
    /* Calibration of Release Motor Point */
    if( !strcmp(cmdReceived[0],"@CALRELEASEPOINT") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            int pointValue = atoi(cmdReceived[1]);            
            releaseMotorPoint = pointValue;
//            releaseMotorPoint = getPositionMotorFeedback();
            return true;
        }
        else
          return false;
    }
    /* Allows to move the motor step by step to a desired position. */
    /* This can be used to adjust initial position of the Assistive Motor. */
    else if( !strcmp(cmdReceived[0],"@MOVE") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            positionReady = false;
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
                setTargetPID = targetMotorPosition;
                inputPositionPID = currentMotorPosition;
                outputPositionPID = 0;
                Serial.println("");
                Serial.print("Target position: ");
                Serial.println(setTargetPID);
    
                do
                {
                    Serial.print("Rotary encoder: ");
                    Serial.print(inputPositionPID);
                    Serial.print("\t\t");
                    Serial.print("PID control signal: ");
                    Serial.println(outputPositionPID);

                    assistiveMotorPID.Compute();
                    assistiveMotor->setSpeed(abs(outputPositionPID));
    
                    if( outputPositionPID > 0 )
                    {
                        assistiveMotor->run(FORWARD);
                    }
                    else if( outputPositionPID < 0 )
                    {
                        assistiveMotor->run(BACKWARD);
                    }
                    else
                    {
                        // temporally empty
                    }
                }while( !positionReady );

                assistiveMotor->setSpeed(0);
                assistiveMotor->run(RELEASE);
                outputPositionPID = 0;
            }
        }
        else
            return false;
    }
    else if( !strcmp(cmdReceived[0],"@MOVETIME") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            int movementTime = atoi(cmdReceived[1]);

            if( movementTime == 0 )
            {
                assistiveMotor->setSpeed(0);
                assistiveMotor->run(RELEASE);              
            }
            else if( movementTime > 0 )
            {
                assistiveMotor->setSpeed(50);
                assistiveMotor->run(FORWARD);
                delay(abs(movementTime));
            }
            else
            {
                assistiveMotor->setSpeed(50);
                assistiveMotor->run(BACKWARD);              
                delay(abs(movementTime));
            }

            assistiveMotor->setSpeed(0);
            assistiveMotor->run(RELEASE);              
        }
    }
    /* Calibration of Z axis */
    else if( !strcmp(cmdReceived[0],"@TRANSPARENCY") )
    {
        if( !strcmp(cmdReceived[1],"ON\r") )
        {
            inputPositionPID = getPositionMotorFeedback();
            outputPositionPID = 0;
//            Serial.println("");

            do
            {
                if( Serial.available() > 0 )
                {
                    commandStatus = readCommands();
                    replyAcknowledge(commandStatus);
                }
/*
                Serial.print("Target position: ");
                Serial.print(setTargetPID);
                Serial.print("\t");
                
                Serial.print("Current position: ");
                Serial.print(inputPositionPID);
                Serial.print("\t");
                                  
                Serial.print("PID control signal: ");
                Serial.println(outputPositionPID);
*/

                currentSensorMotorPosition = analogRead(linPotPin);
                currentSensorMotorPosition = 1023 - currentSensorMotorPosition;
                mapCurrentSensorMotorPosition = map(currentSensorMotorPosition, minInputMapSlider, maxInputMapSlider, minValMapSlider, maxValMapSlider);
                setTargetPID = constrain(mapCurrentSensorMotorPosition, minValMapSlider, maxValMapSlider);

                positionReady = false;

                if( currentSensorMotorPosition <= 256 )
                {
                    sensorMotor->setSpeed(0);
                    sensorMotor->run(RELEASE);
                }
                else
                {
                    sensorMotor->setSpeed(120);
                    sensorMotor->run(BACKWARD);        
                }


                if( ( setTargetPID >= minLimitVal && inputPositionPID < minLimitVal ) || ( setTargetPID <= maxLimitVal && inputPositionPID > maxLimitVal ) )
                {
                    assistiveMotorPID.Compute();
                    assistiveMotor->setSpeed(abs(outputPositionPID));
                    if( outputPositionPID > 0 )
                    {
                        assistiveMotor->run(FORWARD);
                    }
                    else if( outputPositionPID < 0 )
                    {
                        assistiveMotor->run(BACKWARD);
                    }                  
                }


                if( inputPositionPID >= minLimitVal && inputPositionPID <= maxLimitVal )                
                {
                    assistiveMotorPID.Compute();
                    assistiveMotor->setSpeed(abs(outputPositionPID));
                    if( outputPositionPID > 0 )
                    {
                        assistiveMotor->run(FORWARD);
                    }
                    else if( outputPositionPID < 0 )
                    {
                        assistiveMotor->run(BACKWARD);
                    }
                    else
                    {
                        // temporally empty
                    }
                }

            }while( strcmp(commands_char[0],"@TRANSPARENCY") || strcmp(commands_char[1],"OFF\r") );

            assistiveMotor->run(RELEASE);
            sensorMotor->run(RELEASE);
//            Serial.println("End of TRANSPARENCY MODE");
            return true;
        }
        else if( !strcmp(cmdReceived[1],"OFF\r") )
        {
//            Serial.println("End of TRANSPARENCY MODE");
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
            do
            {
                if( Serial.available() > 0 )
                {
                    commandStatus = readCommands();
                    replyAcknowledge(commandStatus);
                }
    
                setTargetPID = assistiveMotorPoint;

                positionReady = false;
                 
                if( inputPositionPID >= minLimitVal && inputPositionPID <= maxLimitVal )                
                {
                    assistiveMotorPID.Compute();
                    assistiveMotor->setSpeed(abs(outputPositionPID));
                    if( outputPositionPID > 0 )
                    {
                        assistiveMotor->run(FORWARD);
                    }
                    else if( outputPositionPID < 0 )
                    {
                        assistiveMotor->run(BACKWARD);
                    }
                    else
                    {
                        // temporally empty
                    }
                }
    
            }while( strcmp(commands_char[0],"@ASSISTANCE") || strcmp(commands_char[1],"OFF\r") );
        }

        assistiveMotor->run(RELEASE);

//        Serial.println("End of ASSISTANCE MODE");

        return true;
    }
    /* Calibration of Z axis */
    else if( !strcmp(cmdReceived[0],"@RELEASE") )
    {
        if( !strcmp(cmdReceived[1],"ON\r") )
        {
            do
            {
                if( Serial.available() > 0 )
                {
                    commandStatus = readCommands();
                    replyAcknowledge(commandStatus);
                }
    
                setTargetPID = releaseMotorPoint;

                positionReady = false;
                 
                if( inputPositionPID >= minLimitVal && inputPositionPID <= maxLimitVal )                
                {
                    assistiveMotorPID.Compute();
                    assistiveMotor->setSpeed(abs(outputPositionPID));
                    if( outputPositionPID > 0 )
                    {
                        assistiveMotor->run(FORWARD);
                    }
                    else if( outputPositionPID < 0 )
                    {
                        assistiveMotor->run(BACKWARD);
                    }
                    else
                    {
                        // temporally empty
                    }
                }
    
            }while( strcmp(commands_char[0],"@RELEASE") || strcmp(commands_char[1],"OFF\r") );
        }

        assistiveMotor->run(RELEASE);

//        Serial.println("End of ASSISTANCE MODE");

        return true;
    }
    /* Calibration of T axis */
    else if( !strcmp(cmdReceived[0],"@SETMAXSPEED") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            assistiveSpeed = atoi(cmdReceived[1]);            
            assistiveMotorPID.SetOutputLimits(-assistiveSpeed, assistiveSpeed);
            assistiveMotorPID.SetMode(AUTOMATIC);
            return true;
        }
        else
          return false;
    }
    /* Calibration of Z axis */
    else if( !strcmp(cmdReceived[0],"@INFO\r") )
    {
        int sensorPot = analogRead(linPotPin);
        sensorPot = 1023 - sensorPot;
        int mapSensorPot = map(sensorPot, minInputMapSlider, maxInputMapSlider, minValMapSlider, maxValMapSlider);
        mapSensorPot = constrain(mapSensorPot, minValMapSlider, maxValMapSlider);

        Serial.println("======================================");
        Serial.println("AFO INFO version 0.1");
        Serial.println("======================================");
        Serial.print("Assistive speed: ");
        Serial.println(assistiveSpeed);
        Serial.print("Mid motor position: ");
        Serial.println(midMotorPoint);
        Serial.print("Assistive motor point: ");
        Serial.println(assistiveMotorPoint);
        Serial.print("Release motor point: ");
        Serial.println(releaseMotorPoint);
        Serial.print("Current assitive motor position: ");
        Serial.println(getPositionMotorFeedback());
        Serial.print("Current sensor motor position: ");
        Serial.print(sensorPot);
        Serial.print(" [");
        Serial.print(mapSensorPot);
        Serial.println("]");
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
    char *commandArray[] = {"@CALMIDPOINT","@CALASSISTIVEPOINT","@CALRELEASEPOINT","@MOVE","@SETMAXSPEED","@INFO\r","@TRANSPARENCY","@ASSISTANCE","@RELEASE","@MOVETIME"};
    int ncommands = 10;
    
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

