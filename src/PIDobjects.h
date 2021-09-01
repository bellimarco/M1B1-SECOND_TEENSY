

class PIDobj{
  public:
  //target value
  float target=0;

  //PID constants
  float pk=0;
  float ik=0; 
  float dk=0;
  //PID error values
  float e=0;
  float ie=0; //integral error
  float de=0; //average derivative error
  //previous instantaneous error derivative values
  float de1 = 0; float de2=0; float de3=0; float de4=0;

  PIDobj(float p,float i, float d){
    pk = p; ik = i; dk = d;
  }

  void SetTarget(float t){
    e += t-target;  //solve derivative inconsistency between target change
    target = t;
  }
  //just update
  void Tick(float dt, float val){
    de4 = de3; de3 = de2; de2 = de1; de1 = (val-target-e)/dt;
    e = val-target; ie += e*dt;
    //weighted average of previous insta derivatives
    de = (de1*3+de2*2+de3*2+de4)/8;
  }
  //update and get PID output
  float Get(float dt, float val){
    de4 = de3; de3 = de2; de2 = de1; de1 = (val-target-e)/dt;
    e = val-target; ie += e*dt;
    //weighted average of previous insta derivatives
    de = de1*0.375+de2*0.25+de3*0.25+de4*0.125;

    return (pk*e + ik*ie + dk*de);
  }

  void Reset(float val){
    e=val-target; ie=0; de=0;
    de1=0; de2=0; de3=0; de4=0;
  }
};

//velocity PID from position error
PIDobj V_PID[MotorNumber] = {
  PIDobj(0,0,0),
  PIDobj(0,0,0),
  PIDobj(0,0,0),
  PIDobj(0,0,0)
};
//acceleration PID from velocity error
PIDobj A_PID[MotorNumber] = {
  PIDobj(0,0,0),
  PIDobj(0,0,0),
  PIDobj(0,0,0),
  PIDobj(0,0,0)
};