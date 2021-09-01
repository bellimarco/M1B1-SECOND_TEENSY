
#include <SimpleFOC.h>

//motor stuff
#define FOCShieldShunt 0.006
#define FOCShieldGain 100
#define powerFOCShieldShunt 0.001
#define powerFOCShieldGain 50


BLDCMotor Motors[MotorNumber] = {BLDCMotor(12),BLDCMotor(12),BLDCMotor(12),BLDCMotor(12)};

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

InlineCurrentSense MotorCurrSenses[MotorNumber] = {
    InlineCurrentSense(FOCShieldShunt, FOCShieldGain, M1curA, M1curB),
    InlineCurrentSense(FOCShieldShunt, FOCShieldGain, M2curA, M2curB),
    InlineCurrentSense(FOCShieldShunt, FOCShieldGain, M3curA, M3curB),
    InlineCurrentSense(FOCShieldShunt, FOCShieldGain, M4curA, M4curB)
};

void MotorObjectsSetup(){

  for(byte i=0; i<MotorNumber; i++){
    MotorEncoders[i].init();
    Motors[i].linkSensor(&MotorEncoders[i]);

    MotorDrivers[i].voltage_power_supply = SupplyVoltage;
    MotorDrivers[i].init();
    Motors[i].linkDriver(&MotorDrivers[i]);

    MotorCurrSenses[i].init();
    Motors[i].linkCurrentSense(&MotorCurrSenses[i]);

    Motors[i].foc_modulation = FOCModulationType::SpaceVectorPWM;
    Motors[i].controller = MotionControlType::torque;
    Motors[i].target = 0;

    Motors[i].voltage_limit = 5;

    Motors[i].useMonitoring(Serial);

    Motors[i].init();
    //Motors[i].initFOC(5.46,Direction::CW);
    Motors[i].initFOC();
  }
}