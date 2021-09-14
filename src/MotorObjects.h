
#include <SimpleFOC.h>


//motor stuff
#define FOCShieldShunt 0.006
#define FOCShieldGain 100
#define powerFOCShieldShunt 0.001
#define powerFOCShieldGain 50


BLDCMotor Motors[MotorNumber] = {BLDCMotor(12),BLDCMotor(12),BLDCMotor(7),BLDCMotor(7)};

BLDCDriver3PWM MotorDrivers[MotorNumber] = {
    BLDCDriver3PWM(M1pwmA, M1pwmB, M1pwmC),
    BLDCDriver3PWM(M2pwmA, M2pwmB, M2pwmC),
    BLDCDriver3PWM(M3pwmA, M3pwmB, M3pwmC),
    BLDCDriver3PWM(M4pwmA, M4pwmB, M4pwmC)
};

MagneticSensorSPI MotorEncoders[MotorNumber] = {
    MagneticSensorSPI(AS5147_SPI, M1CS),
    MagneticSensorSPI(AS5147_SPI, M2CS),
    MagneticSensorSPI(AS5147_SPI, M3CS),
    MagneticSensorSPI(AS5147_SPI, M4CS)
};

bool MotorUseCurrSense[MotorNumber] = {false,false,false,false};
InlineCurrentSense MotorCurrSenses[MotorNumber] = {
    InlineCurrentSense(FOCShieldShunt, FOCShieldGain, M1curA, M1curB),
    InlineCurrentSense(powerFOCShieldShunt, powerFOCShieldGain, M2curA, M2curB),
    InlineCurrentSense(powerFOCShieldShunt, powerFOCShieldGain, M3curA, M3curB),
    InlineCurrentSense(powerFOCShieldShunt, powerFOCShieldGain, M4curA, M4curB)
};

//if gonna use the configgd values for initFOC
const bool MotorUseZeroConfigs[MotorNumber] = {false,false,false,false};
const float MotorZeroAngle[MotorNumber] = {5.46,0,0,0};
//CCW -> 0, CW -> 1
const Direction MotorZeroDirection[MotorNumber] = {Direction::CW,Direction::CW,Direction::CW,Direction::CW};

void MotorObjectsSetup(){
    for(byte i=0; i<MotorNumber; i++){
        LogPrintln("\nMotorSetup "+String(i)+":");
        MotorEncoders[i].init();
        Motors[i].linkSensor(&MotorEncoders[i]);

        MotorDrivers[i].voltage_power_supply = DefaultSupplyVoltage;
        MotorDrivers[i].init();
        Motors[i].linkDriver(&MotorDrivers[i]);

        if(MotorUseCurrSense[i]){
            MotorCurrSenses[i].init();
            Motors[i].linkCurrentSense(&MotorCurrSenses[i]);
        }

        Motors[i].foc_modulation = FOCModulationType::SpaceVectorPWM;
        Motors[i].controller = MotionControlType::torque;
        Motors[i].target = 0;

        Motors[i].voltage_limit = 3;

        Motors[i].useMonitoring(Serial);

        Motors[i].init();
        if(MotorUseZeroConfigs[i]){
            Motors[i].initFOC(MotorZeroAngle[i],MotorZeroDirection[i]);
        }else{
            Motors[i].initFOC();
        }

        //actual voltage limit is set manually
        Motors[i].voltage_limit = 20;
        Motors[i].move(0);

        JointPosition[i] = Motors[i].shaft_angle*GearReduction[i];
        JointVelocity[i] = Motors[i].shaft_velocity*GearReduction[i];
    }
}