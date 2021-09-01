#include <Arduino.h>

#include <SPI.h>


//CONFIGS

#define Log
#ifdef Log
    #define LogPrintln(x) Serial.println(x)
    #define LogPrint(x) Serial.print(x)
#else
    #define LogPrintln(x)
    #define LogPrint(x)
#endif

//serial channel to use
#define Port Serial1

//#define USE_MOTORS      //if all the FOC stuff is runned




#define MotorNumber 4

#define BatteryPin 23
//teensy motor pins
#define M1pwmA 2
#define M1pwmB 3
#define M1pwmC 4
#define M2pwmA 5
#define M2pwmB 6
#define M2pwmC 7
#define M3pwmA 8
#define M3pwmB 9
#define M3pwmC 10
#define M4pwmA 22
#define M4pwmB 19
#define M4pwmC 18

#define M1curA 17
#define M1curB 16
#define M2curA 15
#define M2curB 14
#define M3curA 41
#define M3curB 40
#define M4curA 39
#define M4curB 38

#define M1CS 26
#define M2CS 27
#define M3CS 28
#define M4CS 29


float SupplyVoltage = 14;

//divider constant * Vref / analog resolution
const float BatteryK = 6.01511 * 3.3 /1024;

void UpdateBatteryVoltage(){
  SupplyVoltage = analogRead(BatteryPin)*BatteryK;

  //update motor objects
  //__________________
}

//constants
const float GearReduction[MotorNumber] = {10,10,10,10};
const float JointInertia[MotorNumber] = {1,1,1,1};     //joint acceleration to required joint torque constant, i.e. joint inertia
const float TorqueToVoltage[MotorNumber] = {1,1,1,1};  //desired joint torque to required motor voltage constant
const float TorqueJerk = 100;


bool MotorMode[MotorNumber] = {false,false,false,false};   //0-> torque, 1-> position

float PositionTarget[MotorNumber];

float TorqueTarget[MotorNumber];
float Torque[MotorNumber];



#include <PIDobjects.h>
#include <MotorObjects.h>
#include <SerialUtils.h>


//time management
unsigned long ThisLoop = 0;
unsigned long LastLoop = 0; //timestamp of last loop
float LoopDT = 0;           //seconds passed since last loop

//send encoder data timer
const uint32_t Timer1T = 50000; //micros
uint32_t Timer1t = 0;
//check serial commands, update motor targets
const uint32_t Timer2T = 1000; //micros
uint32_t Timer2t = 0;


void setup() {
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
    Port.begin(800000);

    Serial.begin(115200);
    for(byte i=0; i<14; i++){ digitalWrite(LED_BUILTIN,HIGH); delay(13); digitalWrite(LED_BUILTIN,LOW); delay(25); }
    LogPrintln("HelloWorld");

    bool ready=false;
    while(!ready){
        Port.print("READY");    //continuosly print ready message
        if(Port.available()){ if(Port.read() == '\n'){ ready=true;}} //once received confirmation, stop
        digitalWrite(LED_BUILTIN,HIGH); delay(4);
        digitalWrite(LED_BUILTIN,LOW); delay(22);
    }
    while(Port.available()){ Port.read(); }  //flush serial buffer

    pinMode(BatteryPin,INPUT);
    
    pinMode(M1pwmA,OUTPUT); pinMode(M1pwmB,OUTPUT); pinMode(M1pwmC,OUTPUT);
    pinMode(M2pwmA,OUTPUT); pinMode(M2pwmB,OUTPUT); pinMode(M2pwmC,OUTPUT);
    pinMode(M3pwmA,OUTPUT); pinMode(M3pwmB,OUTPUT); pinMode(M3pwmC,OUTPUT);
    pinMode(M4pwmA,OUTPUT); pinMode(M4pwmB,OUTPUT); pinMode(M4pwmC,OUTPUT);

    pinMode(M1curA,INPUT); pinMode(M1curB,INPUT);
    pinMode(M2curA,INPUT); pinMode(M2curB,INPUT);
    pinMode(M3curA,INPUT); pinMode(M3curB,INPUT);
    pinMode(M4curA,INPUT); pinMode(M4curB,INPUT);

    pinMode(M1CS, OUTPUT);
    pinMode(M2CS, OUTPUT);
    pinMode(M3CS, OUTPUT);
    pinMode(M4CS, OUTPUT);

    #ifdef USE_MOTORS
    MotorObjectsSetup();
    #endif

    for(byte i=0; i<8; i++){ digitalWrite(LED_BUILTIN,HIGH); delay(13); digitalWrite(LED_BUILTIN,LOW); delay(25); }
    digitalWrite(LED_BUILTIN,HIGH);
}


void loop() {

    ThisLoop = micros();
    LoopDT = ((float)(ThisLoop-LastLoop))*1e-6;
    LastLoop = ThisLoop;

    if(ThisLoop>Timer1t){
        Timer1t = ThisLoop + Timer1T;

        //test
        for(byte i=0; i<MotorNumber; i++){
            Motors[i].LatestAngle += 0.1*(i+1);
            Motors[i].LatestVelocity += -0.1*(i+1);
        }

        SerialSend();
    }
    if(ThisLoop>Timer2t){
        Timer2t = ThisLoop + Timer2T;

        UpdateBatteryVoltage();

        TR.run();

        for(byte i = 0; i < MotorNumber; i++){

            if(MotorMode[i]){
                //position control, -> torque target determined by PID loops
                A_PID[i].SetTarget(V_PID[i].Get(LoopDT,Motors[i].LatestAngle));
                TorqueTarget[i] = A_PID[i].Get(LoopDT,Motors[i].LatestVelocity) * JointInertia[i];
            }
            //else: torquetarget already set by SerialComm

            if(abs(TorqueTarget[i]-Torque[i])>TorqueJerk*LoopDT){
                Torque[i] += ((TorqueTarget[i]>Torque[i])?TorqueJerk:-TorqueJerk)*LoopDT;
            }else{
                Torque[i] = TorqueTarget[i];
            }

            #ifdef USE_MOTORS
            Motors[i].move(Torque[i]*TorqueToVoltage[i]);
            #endif
        }
    }

    
    #ifdef USE_MOTORS
    for(byte i = 0; i < MotorNumber; i++){ Motors[i].loopFOC(); }
    #endif

    delayMicroseconds(50);
}



