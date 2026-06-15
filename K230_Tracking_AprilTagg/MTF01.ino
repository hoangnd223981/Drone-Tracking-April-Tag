#include "C:\Users\Admin\Documents\Arduino\libraries\MAVLink\mavlink\ardupilotmega\mavlink.h"

HardwareSerial SerialMav(1);
#define LED_PIN_MTF01 13

float distance_m = 0;
float velocity_x = 0;
float velocity_y = 0;

float raw_flow_x = 0;
float raw_flow_y = 0;
int flow_quality = 0;

unsigned long MTF01_last_time_read_data = 0;

bool flow_valid = false;
bool status_mtf01 = 0;

void setup_optical()
{
  SerialMav.begin(115200, SERIAL_8N1, 14, 15);
  pinMode(LED_PIN_MTF01, OUTPUT);
  digitalWrite(LED_PIN_MTF01, 0);

  Serial.println("==========================");
  Serial.println("ESP32 MAVLink Optical Flow");
  Serial.println(".... Đang chờ dữ liệu ....");
  Serial.println("==========================");
}

void read_optical()
{
  static mavlink_message_t msg;
  static mavlink_status_t status;

  //Đọc dữ liệu MAVLink
  while(SerialMav.available())
  {
    uint8_t c = SerialMav.read();

    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      switch(msg.msgid)
      {
        //1. Nhận khoảng cách từ Lidar
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        {
          mavlink_distance_sensor_t dist;
          mavlink_msg_distance_sensor_decode(&msg, &dist);

          //Chuyển từ cm -> m
          float new_distance = dist.current_distance / 100.0f;
          if(new_distance >= 0.02f && new_distance <= 3.0f)
          {
            distance_m = new_distance;
          }
          break;
        }

        //2. Nhận và tính toán vận tốc Optical Flow
        case MAVLINK_MSG_ID_OPTICAL_FLOW: 
        {
          mavlink_optical_flow_t flow;
          mavlink_msg_optical_flow_decode(&msg, &flow);
          
          // Lưu giá trị flow thô vào biến toàn cục
          raw_flow_x = flow.flow_x;
          raw_flow_y = flow.flow_y;
          flow_quality = flow.quality;
          
          // if (flow.quality >= 10 && distance_m > 0.02f) {
          if (flow.quality >= 10 ) 
          {
          
            // TÍNH TOÁN VẬN TỐC THỰC TẾ
            // flow_x, flow_y: pixel displacement
            // distance_m: khoảng cách thực tế (m)
            velocity_x = (flow.flow_y / 1.0f) * distance_m;  // cm/s
            velocity_y = (flow.flow_x / 1.0f) * distance_m;  // cm/s
            flow_valid = true;
            MTF01_last_time_read_data = millis();
            
            // IN KẾT QUẢ
            // Serial.print("✅ VEL_X: ");
            // Serial.print(velocity_x, 2);
            // Serial.print(",100,-100,");
            // Serial.print(" cm/s");
            // Serial.print(" | VEL_Y: ");
            // Serial.println(velocity_y, 2);
            // Serial.print(" cm/s");
            // Serial.print(" | Distance: ");
            // Serial.print(distance_m, 3);
            // Serial.print(" m");
            // Serial.print(" | Quality: ");
            // Serial.println(flow.quality);
          } else {
            flow_valid = false;
            if (flow.quality < 10) 
            {
              Serial.println("❌ Quality too low");
            } else 
            {
              Serial.println("❌ Invalid distance");
            }
          }
          break;
        }

        //3. Kiểm tra MTF01
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
          static unsigned long last_heartbeat = 0;
          if(millis() - last_heartbeat > 5000)
          {
            Serial.println("Connected rồi nhé");
            last_heartbeat = millis();
          }
          break;
        }
        default:
          break;
      }
    }
  }

  if(millis() - MTF01_last_time_read_data < 100)
  {
    digitalWrite(LED_PIN_MTF01, 1);
    status_mtf01 = 1;
  }
  else
  {
    digitalWrite(LED_PIN_MTF01, 0);
    status_mtf01 = 0;
  }
}

float get_mtf01_distance()
{
  return distance_m;
}

float get_mtf01_raw_velocity_x()
{
  return raw_flow_y;
}

float get_mtf01_raw_velocity_y()
{
  return raw_flow_x;
}

float get_mtf01_velocity_x()
{
  return -velocity_x;
}

float get_mtf01_velocity_y()
{
  return velocity_y;
}

float get_mtf01_noise_velocity_x(float rate_x)
{
  return rate_x * (3.14/180) * distance_m * 100;
}

float get_mtf01_noise_velocity_y(float rate_y)
{
  return rate_y * (3.14 / 180) * distance_m * 100;
}

float get_mtf01_final_velocity_x(float rate_x)
{
  return -velocity_x - rate_x * (3.14/180) * distance_m * 100; //cm/s
}

float get_mtf01_final_velocity_y(float rate_y)
{
  return velocity_y - rate_y * (3.14 / 180) * distance_m * 100;  //cm/s
}

float get_mtf01_status()
{
  return status_mtf01;
}

float get_mtf01_quality()
{
  return flow_quality;
}