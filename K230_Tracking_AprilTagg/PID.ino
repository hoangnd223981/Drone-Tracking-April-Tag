/* ===================== PID RATE ===================== */
float DesiredRateRoll  , DesiredRatePitch  , DesiredRateYaw;
float ErrorRateRoll    , ErrorRatePitch    , ErrorRateYaw;
float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;
float PrevDtermRateRoll = 0, PrevDtermRatePitch = 0, PrevDtermRateYaw = 0;
float DummyDterm = 0;

/* ===================== PID ANGLE ===================== */
float DesiredAngleRoll  , DesiredAnglePitch, DesiredAngleYaw;
float ErrorAngleRoll    , ErrorAnglePitch, ErrorAngleYaw;
float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0, PrevErrorAngleYaw = 0;
float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0, PrevItermAngleYaw = 0;

/* ===================== PID VERTICAL VELOCITY ===================== */
float DesiredVelocityVz, VelocityVz;
float ErrorVelocityVz = 0;
float PrevErrorVelocityVz = 0; 
float PrevItermVelocityVz = 0;

/* ===================== PID POSITION ===================== */
float DesiredVelocityVx, VelocityVx;
float ErrorVelocityVx = 0;
float PrevErrorVelocityVx = 0;
float PrevItermVelocityVx = 0;

float DesiredVelocityVy, VelocityVy;
float ErrorVelocityVy;
float PrevErrorVelocityVy = 0;
float PrevItermVelocityVy = 0;

float DesiredPosZ, OutputPosZ;
float ErrorPosZ = 0;

/* ===================== PID COMMON ===================== */
float PIDReturn[] = {0, 0, 0};

/* ===================== PID RATE ===================== */
float PRateRoll  = 0.8;
float PRatePitch = 0.8;
float PRateYaw   = 2;

float IRateRoll  = 2;
float IRatePitch = 2;
float IRateYaw   = 10;

float DRateRoll  = 0.06;
float DRatePitch = 0.06;
float DRateYaw   = 0;

/* ===================== PID ANGLE ===================== */
float PAngleRoll  = 10;
float PAnglePitch = 10;
float PAngleYaw   = 0;

float IAngleRoll  = 0;
float IAnglePitch = 0;
float IAngleYaw   = 0;

float DAngleRoll  = 0;
float DAnglePitch = 0;
float DAngleYaw   = 0;

/*===================== PID VERTICAL VELOCITY =====================*/
float PVelocityVz = 6;
float IVelocityVz = 10;
float DVelocityVz = 0;

/* ===================== PID POSITION ===================== */
float PVelocityVx = 0.3;
float IVelocityVx = 0.5;
float DVelocityVx = 0;

float PVelocityVy = 0.3;
float IVelocityVy = 0.5;
float DVelocityVy = 0;

float PPosZ = 2;

/* ===================== PID CORE ===================== */
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm, float &PrevDterm, float Imax, float PIDmax, float lpf_d) 
{
  float Pterm = P * Error;

  float Iterm = PrevIterm + I * (Error + PrevError) * 0.005 / 2;
  if (Iterm > Imax) Iterm = Imax;
  else if (Iterm < -Imax) Iterm = -Imax;

  float Dterm_raw = D * (Error - PrevError) / 0.005;
  float Dterm = (Dterm_raw * lpf_d) + (PrevDterm * (1.0 -lpf_d));
  PrevDterm = Dterm;

  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > PIDmax) PIDOutput = PIDmax;
  else if (PIDOutput < -PIDmax) PIDOutput = -PIDmax;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

/* ===================== RESET PID ===================== */
void reset_pid(void) 
{
  PrevItermRateRoll    *= 0.9;
  PrevItermRatePitch   *= 0.9;
  PrevItermRateYaw     *= 0.9;

  PrevItermAngleRoll   *= 0.9;
  PrevItermAnglePitch  *= 0.9;
  PrevItermAngleYaw    *= 0.9;

  PrevDtermRateRoll  *= 0.9;
  PrevDtermRatePitch *= 0.9;
  PrevDtermRateYaw   *= 0.9;

  PrevItermVelocityVx = 0;
  PrevItermVelocityVy = 0;

  PrevErrorVelocityVx = 0;
  PrevErrorVelocityVy = 0;
}

void reset_status(void) 
{
  PrevItermRateRoll    = 0;
  PrevItermRatePitch   = 0;
  PrevItermRateYaw     = 0;

  PrevItermAngleRoll   = 0;
  PrevItermAnglePitch  = 0;
  PrevItermAngleYaw    = 0;

  PrevItermVelocityVx  = 0;
  PrevItermVelocityVy  = 0;
  PrevItermVelocityVz  = 0;

  PrevDtermRateRoll  = 0;
  PrevDtermRatePitch = 0;
  PrevDtermRateYaw   = 0;

  PrevErrorVelocityVx = 0;
  PrevErrorVelocityVy = 0;
}