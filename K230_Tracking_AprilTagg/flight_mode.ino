/* ===================== ANGLE MODE ===================== */
void angle_mod() 
{
  angle_PID(-roll_target, pitch_target, 0, roll, pitch, 0);
  rate_PID(DesiredRateRoll, DesiredRatePitch, vel_target_yaw, RateRoll, RatePitch, RateYaw);
  esc_1 = throttle_smoot - InputRoll - InputPitch - InputYaw;
  esc_2 = throttle_smoot + InputRoll + InputPitch - InputYaw;
  esc_3 = throttle_smoot + InputRoll - InputPitch + InputYaw;
  esc_4 = throttle_smoot - InputRoll + InputPitch + InputYaw;

  limit_value(900, 1600);
}

void alt_hold_mod() 
{
  velocity_Vz(throttle_vel_target, V_z_KF);
  angle_PID(-roll_target, pitch_target, 0, roll, pitch, 0);
  rate_PID(DesiredRateRoll, DesiredRatePitch, vel_target_yaw, RateRoll, RatePitch, RateYaw);

  esc_1= InputThrottle - InputRoll - InputPitch - InputYaw;
  esc_2= InputThrottle + InputRoll + InputPitch - InputYaw;
  esc_3= InputThrottle + InputRoll - InputPitch + InputYaw;
  esc_4= InputThrottle - InputRoll + InputPitch + InputYaw;
  limit_value(900, 1600);
}
 
void loiter_mod() 
{
  vx_feedback = -velocity_kf_X;
  vy_feedback = -velocity_kf_Y;

  velocity_Vx(vel_target_x, vx_feedback);
  velocity_Vy(vel_target_y, vy_feedback);
  velocity_Vz(pos_z(RC_PosZ, z_KF), V_z_KF);

  roll_target  = InputRollTarget;
  pitch_target = InputPitchTarget;

  angle_PID(-roll_target, pitch_target, 0, roll, pitch, 0);
  rate_PID(DesiredRateRoll, DesiredRatePitch, vel_target_yaw, RateRoll, RatePitch, RateYaw);
  esc_1= InputThrottle - InputRoll - InputPitch - InputYaw;
  esc_2= InputThrottle + InputRoll + InputPitch - InputYaw;
  esc_3= InputThrottle + InputRoll - InputPitch + InputYaw;
  esc_4= InputThrottle - InputRoll + InputPitch + InputYaw;

  limit_value(900, 1600);
}

void auto_land()
{
  throttle_vel_target = -3.5;

  velocity_Vz(throttle_vel_target, V_z_KF);

  roll_target = 0;
  pitch_target = 0;
  vel_target_yaw = 0;

  angle_PID(roll_target, -pitch_target, 0, roll, pitch, 0);
  rate_PID(DesiredRateRoll, DesiredRatePitch, vel_target_yaw, RateRoll, RatePitch, RateYaw);
  esc_1= InputThrottle - InputRoll - InputPitch - InputYaw;
  esc_2= InputThrottle + InputRoll + InputPitch - InputYaw;
  esc_3= InputThrottle + InputRoll - InputPitch + InputYaw;
  esc_4= InputThrottle - InputRoll + InputPitch + InputYaw;

  limit_value(900, 1600);

  if (InputThrottle < 1100 && abs(V_z_KF) < 15)
  {
    detected_land++;
  
    if (detected_land > 100)
    {
      no_fly();
      detected_land = 0;
    }
  }
  else
  {
    detected_land = 0;
  } 
}

void return_home()
{
  throttle_vel_target = -3.5;

  velocity_Vz(throttle_vel_target, V_z_KF);

  roll_target = 0;
  pitch_target = 0;
  vel_target_yaw = 0;

  angle_PID(roll_target, pitch_target, 0, roll, pitch, 0);
  rate_PID(DesiredRateRoll, DesiredRatePitch, vel_target_yaw, RateRoll, RatePitch, RateYaw);
  esc_1= InputThrottle - InputRoll - InputPitch - InputYaw;
  esc_2= InputThrottle + InputRoll + InputPitch - InputYaw;
  esc_3= InputThrottle + InputRoll - InputPitch + InputYaw;
  esc_4= InputThrottle - InputRoll + InputPitch + InputYaw;

  limit_value(900, 1600);

  if (InputThrottle < 1100 && abs(V_z_KF) < 15)
  {
    detected_land++;
  
    if (detected_land > 100)
    {
      no_fly();
      detected_land = 0;
    }
  }
  else
  {
    detected_land = 0;
  } 
}

void land_on_tag() 
{
  static float filt_vel_x = 0.0f;
  static float filt_vel_y = 0.0f;
  static float filt_vel_z = 0.0f;
  static float filt_yaw   = 0.0f;

  const float alpha = 0.15f;

  float raw_vel_x = 0.0f;
  float raw_vel_y = 0.0f;
  float raw_vel_z = 0.0f;
  float raw_yaw   = 0.0f;

  if (K230_LinkOK)
  {
    if (K230_FlightCmd == CMD_HOLD)
    {
      raw_vel_x = 0.0f;
      raw_vel_y = 0.0f;
      raw_vel_z = 0.0f;
      raw_yaw   = 0.0f;
    }
    else if (K230_FlightCmd == CMD_TRACK)
    {
      raw_vel_x = K230_VX_DIR * K230_XY_SCALE * (float)K230_Vx_cm_s;
      raw_vel_y = K230_VY_DIR * K230_XY_SCALE * (float)K230_Vy_cm_s;
      raw_vel_z = 0.0f;
      raw_yaw   = K230_YAW_DIR * K230_YAW_SCALE * (float)K230_YawRate_dps;
    }
    else if (K230_FlightCmd == CMD_DESCEND)
    {
      raw_vel_x = 0.0f;
      raw_vel_y = 0.0f;

      /* K230 gui VzDown > 0 la ha thap
       * Code cua anh auto_land dang dung throttle_vel_target = -3.5 de ha
       * Nen o day doi dau thanh am
       */
      raw_vel_z = -K230_Z_SCALE * (float)K230_VzDown_cm_s;

      raw_yaw = K230_YAW_DIR * K230_YAW_SCALE * (float)K230_YawRate_dps;
    }
    else if (K230_FlightCmd == CMD_SPIRAL)
    {
      /* Xoan oc da duoc K230 tinh thanh vx/vy/yaw */
      raw_vel_x = K230_VX_DIR * K230_XY_SCALE * (float)K230_Vx_cm_s;
      raw_vel_y = K230_VY_DIR * K230_XY_SCALE * (float)K230_Vy_cm_s;
      raw_vel_z = 0.0f;
      raw_yaw   = K230_YAW_DIR * K230_YAW_SCALE * (float)K230_YawRate_dps;
    }
    else if (K230_FlightCmd == CMD_LAND)
    {
      raw_vel_x = 0.0f;
      raw_vel_y = 0.0f;
      raw_vel_z = -3.5f;
      raw_yaw   = 0.0f;
    }
    else
    {
      raw_vel_x = 0.0f;
      raw_vel_y = 0.0f;
      raw_vel_z = 0.0f;
      raw_yaw   = 0.0f;
    }
  }
  else
  {
    raw_vel_x = 0.0f;
    raw_vel_y = 0.0f;
    raw_vel_z = 0.0f;
    raw_yaw   = 0.0f;
  }

  raw_vel_x = constrain(raw_vel_x, -MAX_VEL_XY, MAX_VEL_XY);
  raw_vel_y = constrain(raw_vel_y, -MAX_VEL_XY, MAX_VEL_XY);
  raw_vel_z = constrain(raw_vel_z, -MAX_VEL_Z, MAX_VEL_Z);
  raw_yaw   = constrain(raw_yaw, -MAX_YAW_RATE, MAX_YAW_RATE);

  filt_vel_x = filt_vel_x * (1.0f - alpha) + raw_vel_x * alpha;
  filt_vel_y = filt_vel_y * (1.0f - alpha) + raw_vel_y * alpha;
  filt_vel_z = filt_vel_z * (1.0f - alpha) + raw_vel_z * alpha;
  filt_yaw   = filt_yaw   * (1.0f - alpha) + raw_yaw   * alpha;

  vel_target_x = filt_vel_x;
  vel_target_y = filt_vel_y;
  throttle_vel_target = filt_vel_z;
  vel_target_yaw = filt_yaw;

  /* Giong loiter_mod de dam bao feedback dung chieu */
  vx_feedback = -velocity_kf_X;
  vy_feedback = -velocity_kf_Y;

  velocity_Vx(vel_target_x, vx_feedback);
  velocity_Vy(vel_target_y, vy_feedback);
  velocity_Vz(throttle_vel_target, V_z_KF);

  /* Quan trong:
   * Khong duoc set roll_target = 0, pitch_target = 0
   * Vi nhu vay lenh velocity_Vx/Vy bi mat tac dung
   */
  roll_target  = InputRollTarget;
  pitch_target = InputPitchTarget;

  angle_PID(-roll_target, pitch_target, 0, roll, pitch, 0);
  rate_PID(DesiredRateRoll, DesiredRatePitch, vel_target_yaw, RateRoll, RatePitch, RateYaw);

  esc_1 = InputThrottle - InputRoll - InputPitch - InputYaw;
  esc_2 = InputThrottle + InputRoll + InputPitch - InputYaw;
  esc_3 = InputThrottle + InputRoll - InputPitch + InputYaw;
  esc_4 = InputThrottle - InputRoll + InputPitch + InputYaw;

  limit_value(900, 1600);

  if (K230_FlightCmd == CMD_LAND)
  {
    if (InputThrottle < 1100 && abs(V_z_KF) < 15)
    {
      detected_land++;

      if (detected_land > 100)
      {
        no_fly();
        detected_land = 0;
      }
    }
    else
    {
      detected_land = 0;
    }
  }
  else
  {
    detected_land = 0;
  }
}


/* ===================== NO FLY ===================== */
void no_fly() 
{
  esc_1 = 800;
  esc_2 = 800;
  esc_3 = 800;
  esc_4 = 800;

  reset_status() ;
  status_arm = 0;
}

/* ===================== LIMIT VALUE ===================== */
void limit_value(int min, int max) 
{
  if (esc_1 < min) esc_1 = min;
  if (esc_2 < min) esc_2 = min;
  if (esc_3 < min) esc_3 = min;
  if (esc_4 < min) esc_4 = min;

  if (esc_1 > max) esc_1 = max;
  if (esc_2 > max) esc_2 = max;
  if (esc_3 > max) esc_3 = max;
  if (esc_4 > max) esc_4 = max;
}