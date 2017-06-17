#include<Servo.h>
#include<PID_v1.h>

//Arm
class Arms
{
  private:
  //timer
    long currentMilli;
    long previousMilli;
    
  //servo
    Servo leftFront;
    Servo rightFront;
    Servo leftBack;
    Servo rightBack;

  //initialPosition
    int initialLArm;
    int initialRArm;
    int initialLHand;
    int initialRHand;

  //statePosition
    int maxArm = 80;
    int minArm = 0;
    int pos1Hand = 0;
    int pos2Hand = 40;
    int pos3Hand = 80;

  //Arm Error
    int error = 6;
  
  //Position
    int currentPosition;
    
  public:
    Arms(int LArm, int RArm, int LHand, int RHand);
    void armSetup();
    boolean pickThePP();
    boolean dropThePP();
};
    
  //constructor
  Arms::Arms(int LArm, int RArm, int LHand, int RHand)
  {
    this->initialLArm = LArm;
    this->initialRArm = RArm;
    this->initialLHand = LHand;
    this->initialRHand = RHand;

    this->currentMilli=0;
    this->previousMilli=0;

    this->currentPosition = 0; //resting
  }
  
  void Arms::armSetup()
  {
    //attach servo
    this->leftFront.attach(15);
    this->rightFront.attach(14);
    this->leftBack.attach(18);
    this->rightBack.attach(17);
    
    //initialize servo value
    this->rightFront.write(initialRHand);
    this->leftFront.write(initialLHand);
    this->rightBack.write(initialRArm);
    this->leftBack.write(initialLArm);    
  }

  //pick up pingPong Ball
  boolean Arms::pickThePP()
  {
    long period=0;
    boolean done=false;
    while(!done)
    {
    //handsUp
    if(this->currentPosition == 0)// if the arm is in resting position
    {
        //for(int i=pos2Hand; i<=pos3Hand ; i+=1)
        int i = pos2Hand;
        while(i<=pos3Hand)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - this->previousMilli;          
           
          //Before>> rightHand=40, leftHand=40
          //After >> rightHand=80, leftHand=0
    
          this->rightFront.write(i);
          this->leftFront.write(pos2Hand-(i-pos2Hand));
          
          if(period>60)
          {
            this->previousMilli = this->currentMilli;
            i+=1;            
          }
          //delay(60);
        }
        this->currentPosition++;//1st position
    }
    
    //armsUp
    else if(this->currentPosition == 1)//if the arm is in firstPosition
    {
        int i = minArm;
        //for(int i=minArm; i<=maxArm ; i+=1)
        while(i<=maxArm)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - this->previousMilli;   
           
          this->rightBack.write(maxArm-(i-error));
          this->leftBack.write(i);
          //delay(60);

          if(period>60)
          {
            this->previousMilli = this->currentMilli;   
            i+=1;         
          }
        }
        this->currentPosition=0;
        done=true;
    }
    }
    return done;
  }

  //drop pingPong Ball
  boolean Arms::dropThePP()
  {
    boolean done =false;
    long period =0;
    
    while(!done)
    {
    //handsDown
    if(this->currentPosition == 0)
    {
      int i = pos3Hand;
        //for(int i=pos3Hand; i>=pos1Hand ; i-=1)
        while(i>=pos1Hand)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - previousMilli;
          
          //Before>> rightHand=80, leftHand=0  
          //After >> rightHand=0, leftHand=80 
          this->rightFront.write(i);
          this->leftFront.write(pos3Hand-i);
          //delay(5);
          if(period>5)
          {
            this->previousMilli = this->currentMilli;            
            i-=1;
          }
        }
      this->currentPosition++;
    }
    
    //handsUp
    if(currentPosition == 1)
    {
      int i = pos1Hand;
        //for(int i=pos1Hand; i<=pos2Hand ; i+=1)
        while(i<=pos2Hand)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - previousMilli;
                    
          //Before>> rightHand= 0, leftHand=80
          //After >> rightHand= 40, leftHand=40
            this->rightFront.write(i);
            this->leftFront.write(pos3Hand-i);
            //delay(60);

          if(period>60)
          {
            this->previousMilli = this->currentMilli;                
            i+=1;
          }
        }
        this->currentPosition++;
    }

    //arm down
    if(currentPosition == 2)
    {
      int i = maxArm;
      //for(int i=maxArm; i>=minArm ; i-=1)
      while(i>=minArm)
      {
        //time control
        this->currentMilli = millis();
        period = this->currentMilli - previousMilli;
                  
        this->rightBack.write(maxArm-(i-error));
        this->leftBack.write(i);
        //delay(60);
        if(period>60)
        {
            this->previousMilli = this->currentMilli;                
            i+=1;          
        }
      }
      this->currentPosition=0;
      done = true;
    } 
    } 
    return done; 
  }
//Legs , exempted PID
class Legs
{
  //driverPins
  private:
    int d1Pwm;
    int d1Dir;
    int d2Pwm;
    int d2Dir;
    
  //timer
    long currentMilli;
    long previousMilli;

  //default
    int walkSpeed = 80;
    int turnSpeed = 120;
    double speedOdistance = 0;
    double speedOdegree = 0;
  
    long walkTime = 0;
    long spinTime = 0;  

  public:
    Legs(int pwm1, int dir1, int pwm2, int dir2);
    void legSetup(double wspeed, double tspeed, double speed_distance, double speed_degree);
    void setWalkSpeed(double wspeed);
    void setSpeedODistance();
    void setTurnSpeed(double tspeed);
    void setSpeedODegree();
    void setWalkTime(double wtime);
    void setSpinTime(double stime);
    boolean moveForward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance);
    boolean moveBackward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance);
    boolean rotateClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree);
    boolean rotateAntiClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree);
    boolean stopMove(long stopTime); 
};
  //constructor
  Legs::Legs(int pwm1, int dir1, int pwm2, int dir2)
  {
    this->d1Pwm = pwm1;
    this->d1Dir = dir1;
    this->d2Pwm = pwm2;
    this->d2Dir = dir2;

    currentMilli=0;
    previousMilli=0;
  }

  void Legs::legSetup(double wspeed, double tspeed, double speed_distance, double speed_degree)
  {
    pinMode(this->d1Pwm,OUTPUT);
    pinMode(this->d1Dir,OUTPUT);
    pinMode(this->d2Pwm,OUTPUT);
    pinMode(this->d2Dir,OUTPUT);

    this->setWalkSpeed(wspeed);
    this->setTurnSpeed(tspeed);
    this->speedOdistance = speed_distance;
    this->speedOdegree = speed_degree;
  }
   
  void Legs::setWalkSpeed(double wspeed)
  {
    this->walkSpeed = wspeed;
  }

  void Legs::setSpeedODistance()
  {
    switch(this->walkSpeed)
    {
      case 0:
        this->speedOdistance = 0;
        break;
      //other options, for further implementation
      default:
        break;
    }
  }
  
  void Legs::setTurnSpeed(double tspeed)
  {
    this->turnSpeed = tspeed;
  }

  void Legs::setSpeedODegree()
  {
    switch(this->turnSpeed)
    {
      case 0 :
        this->speedOdegree = 0;
        break;
      //other options, for further implementation
      default:
        break;
    }
  }

  void Legs::setWalkTime(double wtime)
  {
    this->walkTime = wtime;
  }

  void Legs::setSpinTime(double stime)
  {
    this->spinTime = stime;
  }
  
  boolean Legs::moveForward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult=false;
    boolean RconversionResult=false;
    boolean done = false;
    
    //computeTimeTravel
    timeTravel = distance*this->speedOdistance;
    this->currentMilli = millis();
    if(this->currentMilli - this-> previousMilli<=timeTravel)
    {
         //left driver
         analogWrite(d1Pwm,*LpidOutput);
         digitalWrite(d1Dir,LOW);
         //right driver
         analogWrite(d2Pwm,*RpidOutput);
         digitalWrite(d2Dir,LOW);
              
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);
        
        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      }
       
    }
    else
    {
        this->previousMilli = this->currentMilli;
        Serial.println("reset");
        done=true;
    }

     return done;
  }

  boolean Legs::moveBackward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult=false;
    boolean RconversionResult=false;
    boolean done=false;
    
    //computeTimeTravel
    timeTravel = distance*this->speedOdistance;
    
    this->currentMilli = millis();
    period = this->currentMilli - this->previousMilli;
    
      if(this->currentMilli - this->previousMilli<=timeTravel)
      {
         //left driver
         analogWrite(d1Pwm,*LpidOutput);
         //analogWrite(d1Pwm,80);
         digitalWrite(d1Dir,HIGH);
         //right driver
         analogWrite(d2Pwm,*RpidOutput);
         //analogWrite(d2Pwm,80);
         digitalWrite(d2Dir,HIGH);

        Serial.println(*LpidOutput);
        Serial.println(*RpidOutput);
        
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);

        Serial.println(*abs_Lwspeed);
        Serial.println(*abs_Rwspeed);

        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      }
      }
      else
      {
         this->previousMilli = this->currentMilli;
         done = true;
      }

      return done;
  }

  boolean Legs::rotateClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult;
    boolean RconversionResult;
    boolean done=false;
    
    //computeTimeTravel
    timeTravel = degree*this->speedOdegree;

    this->currentMilli = millis();
    period = this->currentMilli - previousMilli;
    
      if(period<=timeTravel)
      {
         //left driver
         analogWrite(this->d1Pwm,*LpidOutput);
         digitalWrite(this->d1Dir,LOW);
         //right driver
         analogWrite(this->d2Pwm,*RpidOutput);
         digitalWrite(this->d2Dir,HIGH);
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);

        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      } 
      }
      else
      {
         this->previousMilli = this->currentMilli;
         done = true;
      }   

      return done;
  }

  boolean Legs::rotateAntiClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult;
    boolean RconversionResult;
    boolean done = false;
    //computeTimeTravel
    timeTravel = degree*this->speedOdegree;

    this->currentMilli = millis();
    period = this->currentMilli - previousMilli;
    
      if(period<=timeTravel)
      {
         //left driver
         analogWrite(this->d1Pwm,*LpidOutput);
         digitalWrite(this->d1Dir,HIGH);
         //right driver
         analogWrite(this->d2Pwm,*RpidOutput);
         digitalWrite(this->d2Dir,LOW);
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);

        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      } 
     }
     else
     {
         this->previousMilli = this->currentMilli;
         done=true;
     }

     return done;
  }

  boolean Legs::stopMove(long stopTime)
  {
    boolean done=false;
    this->currentMilli = millis();
    if(this->currentMilli - this->previousMilli<=stopTime)
    {
      //left driver
      analogWrite(this->d1Pwm,0);
      //right driver
      analogWrite(this->d2Pwm,0); 
    }
    else
    {
      this->previousMilli = this->currentMilli;
      done=true;
    }

    return done;
  }

class Sequencer
{
  private:
    String state2[2];
    String state3[3];
    String state4[4];
    String state5[5];
    String state6[6];
    String state7[7];
    String state8[8];
    int phases;
    
    int amp2[2]; //amplitute
    int amp3[3];
    int amp4[4];
    int amp5[5];
    int amp6[6];
    int amp7[7];
    int amp8[8];

  public:
    void package8(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight);
    void package7(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven);
    void package6(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix);
    void package5(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive);
    void package4(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour);
    void package3(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree);
    void package2(String stateOne, int ampOne, String stateTwo, int ampTwo);
    void setPhases(int _phases);
    String getState(int stateNumber);
    int getAmp(int stateNumber);
    int getPhases();
};

  void Sequencer::package8(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight)
  {
    this->state8[0] = stateOne;
    this->state8[1] = stateTwo;
    this->state8[2] = stateThree;
    this->state8[3] = stateFour;
    this->state8[4] = stateFive;
    this->state8[5] = stateSix;
    this->state8[6] = stateSeven;
    this->state8[7] = stateEight;

    this->phases = 8;

    this->amp8[0] = ampOne;
    this->amp8[1] = ampTwo;
    this->amp8[2] = ampThree;
    this->amp8[3] = ampFour;
    this->amp8[4] = ampFive;
    this->amp8[5] = ampSix;
    this->amp8[6] = ampSeven;
    this->amp8[7] = ampEight;
  }

  void Sequencer::package7(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven)
  {
    this->state7[0] = stateOne;
    this->state7[1] = stateTwo;
    this->state7[2] = stateThree;
    this->state7[3] = stateFour;
    this->state7[4] = stateFive;
    this->state7[5] = stateSix;
    this->state7[6] = stateSeven;

    this->phases = 7;

    this->amp7[0] = ampOne;
    this->amp7[1] = ampTwo;
    this->amp7[2] = ampThree;
    this->amp7[3] = ampFour;
    this->amp7[4] = ampFive;
    this->amp7[5] = ampSix;
    this->amp7[6] = ampSeven;
  }

  void Sequencer::package6(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix)
  {
    this->state6[0] = stateOne;
    this->state6[1] = stateTwo;
    this->state6[2] = stateThree;
    this->state6[3] = stateFour;
    this->state6[4] = stateFive;
    this->state6[5] = stateSix;

    this->phases = 6;

    this->amp6[0] = ampOne;
    this->amp6[1] = ampTwo;
    this->amp6[2] = ampThree;
    this->amp6[3] = ampFour;
    this->amp6[4] = ampFive;
    this->amp6[5] = ampSix;
  }
  
  void Sequencer::package5(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive)
  {
    this->state5[0] = stateOne;
    this->state5[1] = stateTwo;
    this->state5[2] = stateThree;
    this->state5[3] = stateFour;
    this->state5[4] = stateFive;

    this->phases = 5;

    this->amp5[0] = ampOne;
    this->amp5[1] = ampTwo;
    this->amp5[2] = ampThree;
    this->amp5[3] = ampFour;
    this->amp5[4] = ampFive;
  }

  void Sequencer::package4(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour)
  {
    this->state4[0] = stateOne;
    this->state4[1] = stateTwo;
    this->state4[2] = stateThree;
    this->state4[3] = stateFour;

    this->phases = 4;

    this->amp4[0] = ampOne;
    this->amp4[1] = ampTwo;
    this->amp4[2] = ampThree;
    this->amp4[3] = ampFour;
  }

  void Sequencer::package3(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree)
  {
    this->state3[0] = stateOne;
    this->state3[1] = stateTwo;
    this->state3[2] = stateThree;
    
    this->phases = 3;

    this->amp3[0] = ampOne;
    this->amp3[1] = ampTwo;
    this->amp3[2] = ampThree;
  }

  void Sequencer::package2(String stateOne, int ampOne, String stateTwo, int ampTwo)
  {
    this->state2[0] = stateOne;
    this->state2[1] = stateTwo;

    this->phases = 2;

    this->amp2[0] = ampOne;
    this->amp2[1] = ampTwo;
  }
  
  String Sequencer::getState(int stateNumber)
  {
    switch(this->phases)
    {
      case 2:
        return this->state2[stateNumber];
        break;
      case 3:
        return this->state3[stateNumber];
        break;
      case 4:
        return this->state4[stateNumber];
        break;
      case 5:
        return this->state5[stateNumber];
        break;
      case 6:
        return this->state6[stateNumber];
        break;
      case 7:
        return this->state7[stateNumber];
        break;
      case 8:
        return this->state8[stateNumber];
        break;
      default:
        break;  
    }
  }

  int Sequencer::getAmp(int stateNumber)
  {
    switch(this->phases)
    {
      case 2:
        return this->amp2[stateNumber];
        break;
      case 3:
        return this->amp3[stateNumber];
        break;
      case 4:
        return this->amp4[stateNumber];
        break;
      case 5:
        return this->amp5[stateNumber];
        break;
      case 6:
        return this->amp6[stateNumber];
        break;
      case 7:
        return this->amp7[stateNumber];
        break;
      case 8:
        return this->amp8[stateNumber];
        break;
      default:
        break;  
    }
  }

  int Sequencer::getPhases()
  {
    return this->phases;
  }

Arms *botArm = new Arms(0,80,40,40);
Legs *botLeg = new Legs(5,22,4,24);
Sequencer *msequence = new Sequencer();

//pid controller
boolean LconversionResult;
boolean RconversionResult;

double LpidOutput;
double RpidOutput;
double LSetpoint;
double RSetpoint;
double Kp=1, Ki=1, Kd=0;

double Lwspeed, abs_Lwspeed;
double Rwspeed, abs_Rwspeed;

PID myPIDL(&abs_Lwspeed,&LpidOutput,&LSetpoint,Kp,Ki,Kd,DIRECT);
PID myPIDR(&abs_Rwspeed,&RpidOutput,&RSetpoint,Kp,Ki,Kd,DIRECT);

//encoder
//Encoder
const byte encoderLInterrupt = 20;// interrupt 3
const byte encoderLDigital = 8;
const byte encoderRInterrupt = 21;// interrupt 2
const byte encoderRDigital = 9;

byte encoderLInterruptLast;
byte encoderRInterruptLast;

boolean Ldirection;
boolean Rdirection;

void encoderInit()
{
  Ldirection = true;
  Rdirection = true;

  //Left Encoder
  pinMode(encoderLDigital,INPUT);
  attachInterrupt(3,LwheelSpeed,CHANGE);
  //Right Encoder
  pinMode(encoderRDigital,INPUT);
  attachInterrupt(2,RwheelSpeed,CHANGE);  
}

void LwheelSpeed()
{
  int Lstate = digitalRead(encoderLInterrupt);
  if((encoderLInterruptLast == LOW) && Lstate==HIGH)//Lstate will be current state
  {
    int val = digitalRead(encoderLDigital);
    if(val == LOW && Ldirection)
    {
      Serial.println("Lpulse");
      Ldirection = false;
    }
    else if(val == HIGH && !Ldirection)
    {
      Ldirection = true;
    }
  }
  encoderLInterruptLast = Lstate;

  if(!Ldirection)
  {
    Lwspeed++;
  }
  else 
  {
    Lwspeed--;
  }
}

void RwheelSpeed()
{
  int Rstate = digitalRead(encoderRInterrupt);
  if((encoderRInterruptLast == LOW) && Rstate==HIGH)//Rstate will be current state
  {
    int val = digitalRead(encoderRDigital);
    if(val == LOW && Rdirection)
    {
      Serial.println("Rpulse");
      Rdirection = false;
    }
    else if(val == HIGH && !Rdirection)
    {
      Rdirection = true;
    }
  }
  encoderRInterruptLast = Rstate;

  if(Rdirection)
  {
    Rwspeed++;
  }
  else 
  {
    Rwspeed--;  
  }
}

int stateNo=0;
boolean done=false;
void setup() 
{
  // put your setup code here, to run once:
  botArm->armSetup();
  botLeg->legSetup(80,40,1,60);
  //msequence->package4("forward",20000,"stopMove",100,"backward",20000,"stopMove",100);
  //walk in a square clockwise
  msequence->package8("forward",10000,"right",90,"forward",10000,"right",90,"forward",10000,"right",90,"forward",10000,"right",90);
  
  //pidController
  LSetpoint = 20;
  RSetpoint = 20;
  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetSampleTime(50);
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetSampleTime(50);

  //encoder
  encoderInit();
  Serial.begin(9600);
}

void loop() 
{
  // put your main code here, to run repeatedly:

  /*Serial.print("Pulse:");
  Serial.println(Rwspeed);
  Rwspeed = 0;
  delay(100);*/

  //Serial.println(abs_Rwspeed);
  myPIDL.SetOutputLimits(0,255);
  myPIDL.SetOutputLimits(0,255);
  myPIDR.SetOutputLimits(0,255);
  myPIDR.SetOutputLimits(0,255);
  
  if(msequence->getState(stateNo) == "forward")
  {
    done = botLeg->moveForward(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));

    Serial.println("forward");
  }
  else if(msequence->getState(stateNo) == "backward")
  {
    done = botLeg->moveBackward(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));
    Serial.println("backward");
  }
  else if(msequence->getState(stateNo) == "left")
  {
    done = botLeg->rotateClockWise(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));
    Serial.println("left");
  }
  else if(msequence->getState(stateNo) == "right")
  {
    done = botLeg->rotateAntiClockWise(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));
    Serial.println("right");
  }
  else if(msequence->getState(stateNo) == "pickUp")
  {
    botArm->pickThePP();
    Serial.println("pickup");
  }
  else if(msequence->getState(stateNo) == "drop")
  {
    botArm->dropThePP();
    Serial.println("drop");
  }
  else if(msequence->getState(stateNo) == "stopMove")
  {
    done = botLeg->stopMove(msequence->getAmp(stateNo));
    Serial.println("stop");
  }
  
  if(done)
  {
    Serial.println("done");
    stateNo++;
    done=false;
    myPIDL.SetOutputLimits(0.0, 1.0);
    myPIDL.SetOutputLimits(-1.0, 0.0);
    myPIDR.SetOutputLimits(0.0, 1.0);
    myPIDR.SetOutputLimits(-1.0, 0.0);
    Lwspeed=0;
    Rwspeed=0;
    
  }
  
  if(stateNo>=msequence->getPhases())
  {
    //repeat all over again
    stateNo=0;
  }
}
