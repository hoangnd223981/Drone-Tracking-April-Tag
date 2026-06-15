#include <HardwareSerial.h>

/* ===================== DEFINE ===================== */

#define low_value_filter_roll_pitch 0.1
#define yaw_i_value 0.002
#define thr_smoot_value 0.3
#define euler_max 10
#define yaw_v_max 90

#define throttle_v_max 100
#define low_value_filter_throttle 0.3
#define low_value_filter_yaw 0.3

#define low_value_filter_x_y 0.03
#define V_xy_max 200

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811

#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0F
#define SBUS_FRAME_FOOTER 0x00

#define SIGNAL_TIMEOUT 100

/* ===================== SERIAL ===================== */
HardwareSerial Serial_sbus(2);   // UART2

/* ===================== GLOBAL VARIABLES ===================== */
static unsigned int sbusByte;
static unsigned int byteNmbr;
static byte frame[25];
static unsigned int channel_input[17];
static bool newFrame;
static unsigned int i;

unsigned long last_timer_sbus;

// static unsigned int sbus_ch[17];
// float roll_target, pitch_target, yaw_target, throttle_smoot;
// float yaw_vel_target, throttle_vel_target, throttle_vel_target_out;
// bool sbus_status;

/* ===================== SBUS DECODE ===================== */
void decodeChannels() 
{
  int bitPtr = 0;
  int bytePtr = 1;

  channel_input[0] = frame[23];

  for (int chan = 1; chan <= 16; chan++) 
  {
    channel_input[chan] = 0;
    for (int chanBit = 0; chanBit < 11; chanBit++) 
    {
      channel_input[chan] |= ((frame[bytePtr] >> bitPtr) & 1) << chanBit;
      if (++bitPtr > 7) 
      {
        bitPtr = 0;
        bytePtr++;
      }
    }
  }
}

/* ===================== GET SBUS FRAME ===================== */
bool getFrame() 
{
  while (Serial_sbus.available()) 
  {
    sbusByte = Serial_sbus.read();

    if ((sbusByte == SBUS_FRAME_HEADER) && newFrame) 
    {
      newFrame = false;
      byteNmbr = 0;
    } 
    else if (sbusByte == SBUS_FRAME_FOOTER) 
    {
      newFrame = true;
    }

    if (byteNmbr <= 24) 
    {
      frame[byteNmbr++] = sbusByte;

      if ((byteNmbr == 25) && (sbusByte == SBUS_FRAME_FOOTER) && (frame[0] == SBUS_FRAME_HEADER)) 
      {
        return true;
      }
    }
  }
  return false;
}

/* ===================== SETUP SBUS ===================== */
void setup_sbus() 
{
  Serial_sbus.begin(100000, SERIAL_8E2, 16, 17);

  while (!Serial);

  byteNmbr = 255;
  newFrame = false;
}

/* ===================== READ SBUS ===================== */
int read_sbus() 
{
  if (getFrame()) 
  {
    decodeChannels();

    for (i = 1; i <= 16; i++) 
    {
      sbus_ch[i] = map(channel_input[i], SBUS_MIN_OFFSET, SBUS_MAX_OFFSET, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    }

    last_timer_sbus = millis();
  }

  if (millis() - last_timer_sbus > 200) 
  {
    return 0;
  } 
  else 
  {
    return 1;
  }
}

/* ===================== READ CONTROL ===================== */
int read_data_control() 
{
  int status_sbus = read_sbus();

  roll_target =  roll_target * (1 - low_value_filter_roll_pitch) + (((float)sbus_ch[1] - 1500) / (500.0 / euler_max)) * low_value_filter_roll_pitch;

  pitch_target = pitch_target * (1 - low_value_filter_roll_pitch) + (((float)sbus_ch[2] - 1500) / -(500.0 / euler_max)) * low_value_filter_roll_pitch;

  yaw_target = (yaw_target - vel_target_yaw * 0.005) * 0.98 + get_mf_yaw() * 0.02;
  
  if(yaw_target > 360) yaw_target = yaw_target - 360;
  if(yaw_target <= 0) yaw_target = yaw_target + 360;

  throttle_smoot = throttle_smoot * (1 - thr_smoot_value) + map(sbus_ch[3], 1000, 2000, 800, 1600) * thr_smoot_value;

  throttle_vel_target= (throttle_vel_target * (1 - low_value_filter_throttle) + -(((float)sbus_ch[3] - 1500) / -(500.0 / throttle_v_max)) * low_value_filter_throttle);
  
  vel_target_x = vel_target_x * (1 - low_value_filter_x_y) + (((float)sbus_ch[2] - 1500) / (500.0 / V_xy_max)) * low_value_filter_x_y;

  vel_target_y = vel_target_y * (1 - low_value_filter_x_y) + (((float)sbus_ch[1] - 1500) / (500.0 / V_xy_max)) * low_value_filter_x_y;

  vel_target_yaw = vel_target_yaw * (1 - low_value_filter_yaw) + -(((float)sbus_ch[4] - 1500) / (500.0 / yaw_v_max)) * low_value_filter_yaw;

  if (sbus_status == 0) 
  {
    roll_target = 0;
    pitch_target = 0;
    vel_target_yaw = 0;
    throttle_smoot = 800;
  }

  return status_sbus;
}

float get_yaw_rap(){
  return yaw_rap;
}

float get_RC_PosZ(float height, float dt){
  static float rc_pos_z_rap = 0; 
  
  rc_pos_z_rap = (rc_pos_z_rap + throttle_vel_target * dt) * 0.999 + height * 0.001;
  if(rc_pos_z_rap > 200) rc_pos_z_rap = 200;
  return rc_pos_z_rap;
}