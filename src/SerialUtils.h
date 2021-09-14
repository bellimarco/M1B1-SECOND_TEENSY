



byte datasendBytes[8][4];

const byte checkintMask = 0b00001110;     //3bit int check arbitrary value
const byte checkintValue = 0b00001010;
const byte parityMask = 0b00000001;

//sending encoder data periodically
void SerialSend(){
    byte *b;
    for(byte i=0; i<MotorNumber; i++){
        b = (byte *) & JointPosition[i];
        for(byte j=0; j<4; j++){ datasendBytes[i][j] = b[j]; }
    }
    for(byte i=0; i<MotorNumber; i++){
        b = (byte *) & JointVelocity[i];
        for(byte j=0; j<4; j++){ datasendBytes[i+MotorNumber][j] = b[j]; }
    }
    byte parity=0;
    byte mask;
    for(byte k=0; k<MotorNumber*2; k++){
        for(byte i=0; i<4; i++){
            //for every byte in the (MotorNumber*2)x4 byte array
            mask = 0b10000000;
            for(byte j=0; j<8; j++){
                if(datasendBytes[k][i] & mask){ parity++; }
                mask>>=1;
            }
        }
    }
    parity &= 0b00000001;


    for(byte k=0; k<MotorNumber*2; k++){
        for(byte i=0; i<4; i++){
            Port.write(datasendBytes[k][i]);
        }
    }

    Port.write(checkintValue | parity);
}




//object to manage serial read on a teensy communication channel,
//designed specifically for my arbitrary protocol (4float values+1control byte)
class TeensyReader{
    public:

    const unsigned long Timeout = 10;   //millis
    unsigned long TimeoutStart = 0;        //msgStarted  timestamp

    bool msgStarted = false;
    byte msgBytesReceived = 0;           //bytes already received, 0 to 33
    byte cntrlByte = 0;
    //data bytes received, grouped by 4 for later float conversion
    const byte Values = 4;
    byte dataBytes[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
    float val[4] = {0,0,0,0}; //last received values, updated when new data has been processed
    bool valMode[4] = {false,false,false,false};

    const byte checkintMask = 0b00001110;
    const byte checkintValue = 0b00001010;  //arbitrary
    const byte parityMask = 0b00000001;

    TeensyReader(){}

    void callback(){
        #ifdef Log_MotorControl
        LogPrint("Control:  ");
        for(byte i=0; i<MotorNumber; i++){
            LogPrint(" (");
            LogPrint(valMode[i]?"ps, ":"tq, ");
            LogPrint(val[i]); LogPrint(")");
        }
        LogPrintln();
        #endif

        for(byte i=0; i<MotorNumber; i++){
            if(!valMode[i]){
            //position control
            //was in torque control
            if(!MotorMode[i]){
                V_PID[i].Reset(JointPosition[i]);
                A_PID[i].Reset(JointVelocity[i]);
            }
            MotorMode[i] = true;
            V_PID[i].SetTarget( val[i] );
            }else{
            //torque control
            MotorMode[i] = false;
            TorqueTarget[i] = val[i];
            }
        }
    }

    void run(){
        while(Port.available()){
            if(!msgStarted){
                msgStarted = true;
                msgBytesReceived = 0;
                TimeoutStart = millis();
            }
            if(millis()-TimeoutStart<Timeout){
                msgBytesReceived++;
                cntrlByte = Port.read();

                if(msgBytesReceived < Values*4 +1){
                    //group byte at right place in dataBytes array
                    dataBytes[(msgBytesReceived-1)/4][(msgBytesReceived-1)%4] = cntrlByte;
                }else{
                    //check last byte
                    byte parity=0;
                    byte mask;
                    for(byte k=0; k<Values; k++){
                        for(byte i=0; i<4; i++){
                            //for every byte in the Valuesx4 byte array
                            mask = 0b10000000;
                            for(byte j=0; j<8; j++){
                                if(dataBytes[k][i] & mask){ parity++; }
                                mask>>=1;
                            }
                        }
                    }
                    //if everything went well copy data to val array
                    if( ((cntrlByte & checkintMask) == checkintValue) && 
                        (parity & 0b00000001) == (cntrlByte & parityMask)){
                        //cast 4bytearrayreference as floatpointer, then
                        //  dereference that pointer to obtain a float variable
                        byte motormodeMask = 0b10000000;
                        for(byte i=0; i<Values; i++){
                            val[i] = * ((float *) &dataBytes[i]);

                            valMode[i] = (cntrlByte & motormodeMask)?true:false;
                            motormodeMask >>= 1;
                        }
                        motormodeMask = 0b10000000; //reset mask

                        callback();
                    }


                    //end message
                    msgStarted = false;
                }
            }else{
                //timed out
                msgStarted = false;
            }
        }
    }
};

TeensyReader TR = TeensyReader();
