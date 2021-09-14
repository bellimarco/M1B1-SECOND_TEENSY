#include <Arduino.h>

#include <SPI.h>


//CONFIGS

//if you are programming teensy 1, teensy 1 constants will be set
//  else teensy 2 constants will be set
#define TEENSY1

#define WAITTEENSY  //if at setup should wait for main teensy connection

#define Log
#ifdef Log
    #define LogPrintln(x) Serial.println(x)
    #define LogPrint(x) Serial.print(x)
    #define Log_MotorControl    //print every received motor control
    #define Log_ENCdata     //print every encoder data that is sent to the main teensy
#else
    #define LogPrintln(x)
    #define LogPrint(x)
#endif


//#define USE_BATTERY     //if gonna read supply voltage from battery


//serial channel to use
#define Port Serial1
#define BAUDRATE 800000

#define USE_MOTORS      //if all the FOC stuff is runned
#ifdef USE_MOTORS
    //#define USE_MOTORSAFEMODE   //motors voltage is not set, foc voltage target remains 0 from setup
#endif


#define SENDENCODERS    //if encoder information is sent to the main teensy port



//send encoder data timer
const uint32_t Timer1T = 50000; //micros
uint32_t Timer1t = 0;
//check serial commands, update motor targets
const uint32_t Timer2T = 40000; //micros
uint32_t Timer2t = 0;



const float DefaultSupplyVoltage = 13.8;

#define BatteryPin 23

// 0-> hip1, 1-> hip2, 2-> leg, 3-> knee
#define MotorNumber 4

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

#define M1CS 29
#define M2CS 28
#define M3CS 27
#define M4CS 26




//constants definition

#ifdef TEENSY1
const float GearReduction[MotorNumber] = {1,1,1,1};
const float TorqueToVoltage[MotorNumber] = {0.1,0.1,0.1,0.1};  //desired joint torque to required motor voltage constant
#else
const float GearReduction[MotorNumber] = {1,1,1,1};
const float TorqueToVoltage[MotorNumber] = {0.1,0.1,0.1,0.1};
#endif
const float JointInertia[MotorNumber] = {1,1,1,1};     //joint acceleration to required joint torque constant, i.e. joint inertia
const float TorqueJerk = 100;
const float MaxVoltage = 3;

float JointPosition[MotorNumber] = {0,0,0,0};
float JointVelocity[MotorNumber] = {0,0,0,0};

bool MotorMode[MotorNumber] = {false,false,false,false};   //0-> torque, 1-> position
float PositionTarget[MotorNumber];
float TorqueTarget[MotorNumber];    //set directly, or indirectly by position controller

float Torque[MotorNumber];  //actual torque applied to motors (jerk adjusted torquetarget)


//time management
unsigned long ThisLoop = 0;
unsigned long LastLoop = 0; //timestamp of last loop
float LoopDT = 0;           //seconds passed since last loop


//imports
#include <PIDobjects.h>
#include <MotorObjects.h>
#include <SerialUtils.h>


//battery management
//divider constant * Vref / analog resolution
const float BatteryK = 6.01511 * 3.3 /1024;
float SupplyVoltage = DefaultSupplyVoltage;

void UpdateBatteryVoltage(){
    #ifdef USE_BATTERY
    SupplyVoltage = analogRead(BatteryPin)*BatteryK;

    //update motor objects
    #ifdef USE_MOTORS
    for(uint8_t i=0; i<MotorNumber; i++){
        MotorDrivers[i].voltage_power_supply = SupplyVoltage;
    }
    #endif
    #endif
}




void setup() {
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
    Port.begin(BAUDRATE);

    #ifdef Log
    Serial.begin(115200);
    for(byte i=0; i<14; i++){ digitalWrite(LED_BUILTIN,HIGH); delay(13); digitalWrite(LED_BUILTIN,LOW); delay(25); }
    LogPrintln("Starting Setup");
    #endif

    #ifdef WAITTEENSY
    bool ready=false;
    while(!ready){
        Port.print("READY");    //continuosly print ready message
        if(Port.available()){ if(Port.read() == '\n'){ ready=true;}} //once received confirmation, stop
        digitalWrite(LED_BUILTIN,HIGH); delay(4);
        digitalWrite(LED_BUILTIN,LOW); delay(22);
    }
    while(Port.available()){ Port.read(); }  //flush serial buffer
    #endif

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

    UpdateBatteryVoltage();

    #ifdef USE_MOTORS
    MotorObjectsSetup();
    #endif

    for(byte i=0; i<8; i++){ digitalWrite(LED_BUILTIN,HIGH); delay(13); digitalWrite(LED_BUILTIN,LOW); delay(25); }
    digitalWrite(LED_BUILTIN,HIGH);
    LogPrintln("Setup Done!");
}


void loop() {

    ThisLoop = micros();
    LoopDT = ((float)(ThisLoop-LastLoop))*1e-6;
    LastLoop = ThisLoop;


    #ifdef SENDENCODERS
    if(ThisLoop>Timer1t){
        Timer1t = ThisLoop + Timer1T;

        for(byte i=0; i<MotorNumber; i++){
            #ifdef USE_MOTORS
            JointPosition[i] = Motors[i].shaft_angle*GearReduction[i];
            JointVelocity[i] = Motors[i].shaft_velocity*GearReduction[i];
            #else
            //test
            JointPosition[i] += 0.1*(i+1);
            JointVelocity[i] += -0.1*(i+1);
            #endif

        }
        #ifdef Log_ENCdata
        LogPrint("ENC: ");
        LogPrint("( "); LogPrint(JointPosition[0]);
        LogPrint(" , "); LogPrint(JointPosition[1]);
        LogPrint(" , "); LogPrint(JointPosition[2]);
        LogPrint(" , "); LogPrint(JointPosition[3]);
        LogPrint(" ) , ( "); LogPrint(JointVelocity[0]);
        LogPrint(" , "); LogPrint(JointVelocity[1]);
        LogPrint(" , "); LogPrint(JointVelocity[2]);
        LogPrint(" , "); LogPrint(JointVelocity[3]);
        LogPrint(" )\n");
        #endif

        SerialSend();
    }
    #endif


    if(ThisLoop>Timer2t){
        Timer2t = ThisLoop + Timer2T;

        UpdateBatteryVoltage();

        TR.run();

        #ifdef USE_MOTORS
        for(byte i = 0; i < MotorNumber; i++){
            JointPosition[i] = Motors[i].shaft_angle*GearReduction[i];
            JointVelocity[i] = Motors[i].shaft_velocity*GearReduction[i];

            if(MotorMode[i]){
                //position control, -> torque target determined by PID loops
                A_PID[i].SetTarget(V_PID[i].Get(LoopDT,JointPosition[i]));
                TorqueTarget[i] = A_PID[i].Get(LoopDT,JointVelocity[i]) * JointInertia[i];
            }
            //else: torquetarget already set by SerialComm

            if(abs(TorqueTarget[i]-Torque[i])>TorqueJerk*LoopDT){
                Torque[i] += TorqueJerk*LoopDT*((TorqueTarget[i]>Torque[i])?1:-1);
            }else{
                Torque[i] = TorqueTarget[i];
            }

            #ifndef USE_MOTORSAFEMODE
            Motors[i].move(min(Torque[i]*TorqueToVoltage[i],MaxVoltage));
            #endif
        }
       #endif
    }

    
    #ifdef USE_MOTORS
    for(byte i = 0; i < MotorNumber; i++){ Motors[i].loopFOC(); }
    #endif

    delayMicroseconds(50);
}



