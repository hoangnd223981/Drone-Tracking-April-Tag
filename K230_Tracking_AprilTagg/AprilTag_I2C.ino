#include <Wire.h>
#include <string.h>
#include <math.h>

/* ===================== APRILTAG I2C ===================== */
#define APRILTAG_I2C_ADDRESS       0x42
#define APRILTAG_I2C_SDA           21
#define APRILTAG_I2C_SCL           22
#define APRILTAG_I2C_FREQUENCY     10000

#define APRILTAG_PACKET_LENGTH     26
#define APRILTAG_HEADER_1          0xAA
#define APRILTAG_HEADER_2          0x55
#define APRILTAG_PACKET_VERSION    0x01
#define APRILTAG_FLAG_VALID        0x01
#define APRILTAG_TIMEOUT_MS        300

/* ===================== APRILTAG LOST SEARCH =====================
 * Khi dang o MODE_APRILTAG ma khong thay Tag, ESP32 tao quy dao tim kiem
 * dang xoan oc ngang bang setpoint van toc X/Y.
 * Don vi van toc: cm/s. Nen test treo hoac thao canh quat truoc khi bay that.
 */
#define APRILTAG_SEARCH_ENABLE          1
#define APRILTAG_SEARCH_DELAY_MS        700
#define APRILTAG_SEARCH_RAMP_TIME_MS    18000
#define APRILTAG_SEARCH_MIN_SPEED_CM_S  8.0f
#define APRILTAG_SEARCH_MAX_SPEED_CM_S  30.0f
#define APRILTAG_SEARCH_OMEGA_RAD_S     0.55f
#define APRILTAG_SEARCH_UP_CM_S         0.0f
#define APRILTAG_SEARCH_YAW_DEG_S       0.0f

portMUX_TYPE apriltag_i2c_mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t apriltag_rx_buffer[APRILTAG_PACKET_LENGTH];
volatile uint8_t apriltag_rx_length = 0;
volatile bool apriltag_rx_pending = false;

bool apriltag_tag_valid = false;
uint16_t apriltag_tag_id = 0xFFFF;
float apriltag_error_right_cm = 0.0f;
float apriltag_error_forward_cm = 0.0f;
float apriltag_height_cm = 0.0f;
float apriltag_heading_deg = 0.0f;
float apriltag_cmd_forward_cm_s = 0.0f;
float apriltag_cmd_right_cm_s = 0.0f;
float apriltag_cmd_up_cm_s = 0.0f;
float apriltag_cmd_yaw_deg_s = 0.0f;
uint8_t apriltag_quality = 0;
uint8_t apriltag_sequence = 0;

uint32_t apriltag_last_rx_ms = 0;
uint32_t apriltag_last_valid_tag_ms = 0;
uint32_t apriltag_packet_ok = 0;
uint32_t apriltag_packet_error = 0;
uint32_t apriltag_bad_length = 0;
uint32_t apriltag_bad_header = 0;
uint32_t apriltag_bad_checksum = 0;
bool apriltag_i2c_started = false;

bool apriltag_search_active = false;
uint32_t apriltag_search_start_ms = 0;
float apriltag_search_cmd_forward_cm_s = 0.0f;
float apriltag_search_cmd_right_cm_s = 0.0f;
float apriltag_search_cmd_up_cm_s = 0.0f;
float apriltag_search_cmd_yaw_deg_s = 0.0f;

/* ===================== DOC DU LIEU BIG-ENDIAN ===================== */
int16_t apriltag_join_i16_high_low(uint8_t high_byte, uint8_t low_byte)
{
  uint16_t value = ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
  return (int16_t)value;
}

uint16_t apriltag_join_u16_high_low(uint8_t high_byte, uint8_t low_byte)
{
  return ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
}

uint8_t apriltag_checksum(const uint8_t *data)
{
  uint8_t checksum = 0;

  for(int i = 0; i < APRILTAG_PACKET_LENGTH - 1; i++)
  {
    checksum += data[i];
  }

  return checksum;
}

/* ===================== XOA LENH APRILTAG ===================== */
void clear_apriltag_command()
{
  apriltag_tag_valid = false;
  apriltag_cmd_forward_cm_s = 0.0f;
  apriltag_cmd_right_cm_s = 0.0f;
  apriltag_cmd_up_cm_s = 0.0f;
  apriltag_cmd_yaw_deg_s = 0.0f;
}

/* ===================== TIM KIEM TAG THEO QUY DAO XOAN OC ===================== */
void reset_apriltag_search()
{
  apriltag_search_active = false;
  apriltag_search_start_ms = 0;
  apriltag_search_cmd_forward_cm_s = 0.0f;
  apriltag_search_cmd_right_cm_s = 0.0f;
  apriltag_search_cmd_up_cm_s = 0.0f;
  apriltag_search_cmd_yaw_deg_s = 0.0f;
}

void update_apriltag_search()
{
#if APRILTAG_SEARCH_ENABLE
  uint32_t now_ms = millis();

  if(apriltag_i2c_ready())
  {
    reset_apriltag_search();
    return;
  }

  if(apriltag_last_valid_tag_ms != 0 && now_ms - apriltag_last_valid_tag_ms < APRILTAG_SEARCH_DELAY_MS)
  {
    reset_apriltag_search();
    return;
  }

  if(apriltag_search_start_ms == 0)
  {
    apriltag_search_start_ms = now_ms;
  }

  apriltag_search_active = true;

  float t_s = (now_ms - apriltag_search_start_ms) * 0.001f;
  float ramp = (now_ms - apriltag_search_start_ms) / (float)APRILTAG_SEARCH_RAMP_TIME_MS;

  if(ramp > 1.0f) ramp = 1.0f;
  if(ramp < 0.0f) ramp = 0.0f;

  float speed = APRILTAG_SEARCH_MIN_SPEED_CM_S +
                (APRILTAG_SEARCH_MAX_SPEED_CM_S - APRILTAG_SEARCH_MIN_SPEED_CM_S) * ramp;

  float angle = APRILTAG_SEARCH_OMEGA_RAD_S * t_s;

  apriltag_search_cmd_forward_cm_s = speed * cosf(angle);
  apriltag_search_cmd_right_cm_s = speed * sinf(angle);
  apriltag_search_cmd_up_cm_s = APRILTAG_SEARCH_UP_CM_S;
  apriltag_search_cmd_yaw_deg_s = APRILTAG_SEARCH_YAW_DEG_S;
#else
  reset_apriltag_search();
#endif
}

bool apriltag_searching()
{
  return apriltag_search_active;
}

float get_apriltag_search_cmd_forward()
{
  return apriltag_search_cmd_forward_cm_s;
}

float get_apriltag_search_cmd_right()
{
  return apriltag_search_cmd_right_cm_s;
}

float get_apriltag_search_cmd_up()
{
  return apriltag_search_cmd_up_cm_s;
}

float get_apriltag_search_cmd_yaw()
{
  return apriltag_search_cmd_yaw_deg_s;
}

/* ===================== I2C CALLBACK ===================== */
void apriltag_on_receive(int length)
{


  uint8_t local_buffer[APRILTAG_PACKET_LENGTH];
  uint8_t index = 0;

  while(Wire.available())
  {
    uint8_t value = Wire.read();
    if(index < APRILTAG_PACKET_LENGTH)
    {
      local_buffer[index] = value;
    }
    index++;
  }

  if(index != APRILTAG_PACKET_LENGTH)
  {
    apriltag_bad_length++;
    return;
  }

  portENTER_CRITICAL_ISR(&apriltag_i2c_mux);
  for(int i = 0; i < APRILTAG_PACKET_LENGTH; i++)
  {
    apriltag_rx_buffer[i] = local_buffer[i];
  }
  apriltag_rx_length = APRILTAG_PACKET_LENGTH;
  apriltag_rx_pending = true;
  portEXIT_CRITICAL_ISR(&apriltag_i2c_mux);
  latency_i2c_packet_start();
}

void apriltag_on_request()
{
  // Giup master doc thu/scan kieu read co byte phan hoi.
  Wire.write(0x5A);
}

/* ===================== SETUP I2C ===================== */
void setup_apriltag_i2c()
{
  Wire.end();
  delay(100);

  // ESP32 làm Slave không cần tự cấp xung nhịp
  apriltag_i2c_started = Wire.begin(
    (uint8_t)APRILTAG_I2C_ADDRESS,
    APRILTAG_I2C_SDA,
    APRILTAG_I2C_SCL,
    0 // Đặt tần số bằng 0 hoặc bỏ trống tham số này
  );

  Wire.onReceive(apriltag_on_receive);
  Wire.onRequest(apriltag_on_request);

  clear_apriltag_command();
}

/* ===================== XU LY PACKET ===================== */
void process_apriltag_i2c()
{
  uint8_t packet[APRILTAG_PACKET_LENGTH];
  bool has_packet = false;

  portENTER_CRITICAL(&apriltag_i2c_mux);
  if(apriltag_rx_pending && apriltag_rx_length == APRILTAG_PACKET_LENGTH)
  {
    for(int i = 0; i < APRILTAG_PACKET_LENGTH; i++)
    {
      packet[i] = apriltag_rx_buffer[i];
    }

    apriltag_rx_pending = false;
    has_packet = true;
  }
  portEXIT_CRITICAL(&apriltag_i2c_mux);

  if(!has_packet)
  {
    if(millis() - apriltag_last_rx_ms > APRILTAG_TIMEOUT_MS)
    {
      clear_apriltag_command();
    }

    return;
  }

  if(packet[0] != APRILTAG_HEADER_1 || packet[1] != APRILTAG_HEADER_2 || packet[2] != APRILTAG_PACKET_VERSION)
  {
    apriltag_packet_error++;
    apriltag_bad_header++;
    return;
  }

  if(packet[25] != apriltag_checksum(packet))
  {
    apriltag_packet_error++;
    apriltag_bad_checksum++;
    return;
  }

  apriltag_sequence = packet[3];
  apriltag_tag_valid = (packet[4] & APRILTAG_FLAG_VALID) != 0;
  apriltag_tag_id = apriltag_join_u16_high_low(packet[6], packet[7]);

  apriltag_error_right_cm = apriltag_join_i16_high_low(packet[8], packet[9]) * 0.1f;
  apriltag_error_forward_cm = apriltag_join_i16_high_low(packet[10], packet[11]) * 0.1f;
  apriltag_height_cm = apriltag_join_i16_high_low(packet[12], packet[13]) * 0.1f;
  apriltag_heading_deg = apriltag_join_i16_high_low(packet[14], packet[15]) * 0.1f;

  apriltag_cmd_forward_cm_s = apriltag_join_i16_high_low(packet[16], packet[17]) * 0.1f;
  apriltag_cmd_right_cm_s = apriltag_join_i16_high_low(packet[18], packet[19]) * 0.1f;
  apriltag_cmd_up_cm_s = apriltag_join_i16_high_low(packet[20], packet[21]) * 0.1f;
  apriltag_cmd_yaw_deg_s = apriltag_join_i16_high_low(packet[22], packet[23]) * 0.1f;
  apriltag_quality = packet[24];

  apriltag_last_rx_ms = millis();
  apriltag_packet_ok++;
  latency_packet_processed_mark();

  if(apriltag_tag_valid)
  {
    apriltag_last_valid_tag_ms = apriltag_last_rx_ms;
    reset_apriltag_search();
  }
  else
  {
    clear_apriltag_command();
  }
}

/* ===================== TRANG THAI SAN SANG ===================== */
bool apriltag_i2c_ready()
{
  if(!apriltag_tag_valid)
  {
    return false;
  }

  if(millis() - apriltag_last_rx_ms > APRILTAG_TIMEOUT_MS)
  {
    return false;
  }

  return true;
}

/* ===================== GET COMMAND ===================== */
float get_apriltag_cmd_forward()
{
  return apriltag_cmd_forward_cm_s;
}

float get_apriltag_cmd_right()
{
  return apriltag_cmd_right_cm_s;
}

float get_apriltag_cmd_up()
{
  return apriltag_cmd_up_cm_s;
}

float get_apriltag_cmd_yaw()
{
  return apriltag_cmd_yaw_deg_s;
}
