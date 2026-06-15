// #include <SoftwareSerial.h>

uint32_t LoopTimer;

int esc_1, esc_2, esc_3, esc_4;

float rollRad, pitchRad;

float acc_x_trung_binh, acc_y_trung_binh, acc_z_trung_binh;
float offset_gyro_roll = 0, offset_gyro_pitch = 0, offset_gyro_yaw = 0;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float roll, pitch, yaw;
float lpf_alpha_gyro = 0.4;

float roll_target, pitch_target, yaw_target;
float throttle_smoot;
float throttle_vel_target;
float vel_target_x, vel_target_y, vel_target_yaw, yaw_rap;

float AngleRoll, AnglePitch;
float AltitudeBarometer;
float AltitudeKalman, VelocityVerticalKalman;

float AccXInertial, AccYInertial, AccZInertial;
float AccX_gravity, AccY_gravity, AccZ_gravity;
// float AccX_Filtered, AccY_Filtered, AccZ_Filtered;
float lpf_alpha_acc = 0.25;

float InputRoll, InputPitch, InputYaw, InputThrottle; 
float InputRollTarget, InputPitchTarget;

float x_KF, V_x_KF;
float y_KF, V_y_KF;
float z_KF, V_z_KF;

float RC_PosZ;

float v_x_oftical_imu, v_y_oftical_imu;
float velocity_kf_X,  velocity_kf_Y;
float KalmanUncertainty_velocity_X, KalmanUncertainty_velocity_Y; 

float detected_land = 0;

float vx_feedback, vy_feedback;

 
/* ===================== RC CONFIG ===================== */
#define arm_disarm          5
#define flight_mode         6
#define rth_autoland        7
#define autonomous          8


#define arm_value           2009
#define disarm_value        990

#define angle_mode          1000
#define alt_hold_mode       1500
#define loiter_mode         2000

#define auto_land_mode      1500
#define return_home_mode    2000

#define chu_ki_tinh_toan    0.005

static unsigned int sbus_ch[17];
bool sbus_status;


bool status_switch_arm = 0;
bool status_arm = 0;

/* ===================== SETUP ===================== */
void setup() 
{
  Serial.begin(115200); ;
  K230_Init();
  setup_motor();
  // setup_BMP388();
  setup_icm_20602();
  // setup_IST8310();
  setup_optical();
  setup_kalman();
  setup_sbus();
  // calib_baro();

  while (!sbus_status) 
  {
    sbus_status = read_data_control();
    Serial.println("ko co tin hieu");
  }

  while (sbus_ch[arm_disarm] > arm_value - 100 && sbus_ch[arm_disarm] < arm_value + 100) 
  {
    read_data_control();
    Serial.println("nut_arm_dang_bat_nguy_hiem");
  }
}

/* ===================== LOOP ===================== */
void loop() 
{
  K230_Task();
  loop_icm_20602();
  MadgwickAHRSupdateIMU(RateRoll * 1 / (180 / 3.142), RatePitch * 1 / (180 / 3.142), RateYaw * 1 / (180 / 3.142), AccX, AccY, AccZ);
  // read_optical();
  rollRad = roll * 0.0174533f;
  pitchRad = pitch * 0.0174533f;
  // KalmanFilter(0, 0, baro_filtered, -AccXInertial, AccYInertial, AccZInertial); //kalman cho độ cao bmp388 dùng gps
  KalmanFilter(0, 0, get_mtf01_distance() * cos(rollRad) * cos(pitchRad), -AccXInertial, AccYInertial, AccZInertial); //kalman cho độ cao mtf01
  // AltitudeBarometer = read_baro();
  read_optical();
  // //kalman 1d cho velocity x và y
  Kalman_Velocity_1d(velocity_kf_X, KalmanUncertainty_velocity_X, -AccXInertial * 981, -get_mtf01_final_velocity_x(RatePitch), get_mtf01_distance());
  Kalman_Velocity_1d(velocity_kf_Y, KalmanUncertainty_velocity_Y,  AccYInertial * 981, -get_mtf01_final_velocity_y(RateRoll), get_mtf01_distance());

  
  RC_PosZ = get_RC_PosZ(z_KF, chu_ki_tinh_toan);
  if (sbus_ch[arm_disarm] > disarm_value - 100 && sbus_ch[arm_disarm] < disarm_value + 100) 
  {
    status_switch_arm = 0;
  } 
  else if (sbus_ch[arm_disarm] > arm_value - 100 && sbus_ch[arm_disarm] < arm_value + 100) 
  {
    status_switch_arm = 1;
  }

  if (status_switch_arm == 0) 
  {
    status_arm = 0;
  } 
  else if (status_switch_arm == 1 && sbus_ch[3] < 1050) 
  {
    status_arm = 1;
  }

  if (sbus_ch[3] < 1050) 
  {
    reset_pid();
  }

  if (sbus_status == 1) 
  {
    if (!status_arm) 
    {
      no_fly();
    } 
    else if (sbus_ch[rth_autoland] > auto_land_mode - 100 && sbus_ch[rth_autoland] < auto_land_mode + 100)
    {
      auto_land();
    }
    else if (sbus_ch[rth_autoland] > return_home_mode - 100 && sbus_ch[rth_autoland] < return_home_mode + 100)
    {
      return_home();
    }
    else if (sbus_ch[autonomous] > 1800)
    {
      land_on_tag();
    }
    else if (sbus_ch[flight_mode] > angle_mode - 100 && sbus_ch[flight_mode] < angle_mode + 100) 
    {
      angle_mod();
    } 
    else if (sbus_ch[flight_mode] > alt_hold_mode - 100 && sbus_ch[flight_mode] < alt_hold_mode + 100) 
    {
      alt_hold_mod();
    } 
    else if (sbus_ch[flight_mode] > loiter_mode - 100 && sbus_ch[flight_mode] < loiter_mode + 100) 
    {
      loiter_mod();
    } 
    else //failsafe
    {
      if (status_arm == 1)
      {
        auto_land();
      }
      else
      {
        no_fly();
      }
    }
  }

  control_motor(esc_1, esc_2, esc_3, esc_4);
  sbus_status = read_data_control();
  
  while (micros() - LoopTimer < 5000);
  LoopTimer = micros();

  display();
}



void Kalman_Velocity_1d(float &velocity_kf, float &KalmanUncertainty, float KF_Tangential_Acc_Input, float KF_Velocity_Input, float dis_lidar)
{
  velocity_kf = velocity_kf + 0.005 * KF_Tangential_Acc_Input; // dự đoán vận tốc mới
  KalmanUncertainty =  KalmanUncertainty + 0.005 * 0.005 * 400 * 400; // dự đoán sai số
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 140 * 140 * dis_lidar); //tính hệ số tin cậy
  velocity_kf = velocity_kf + KalmanGain * (KF_Velocity_Input - velocity_kf); //chốt số vận tốc cuối cùng
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty; // cập nhật độ tin cậy
}