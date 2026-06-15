#define offset_acc_x -0.0160;
#define offset_acc_y 0.0200;
#define offset_acc_z -0.0067;

#include <SPI.h>

// Định nghĩa các chân
#define CS_PIN 5
#define SPI_CLOCK 1000000  // 1 MHz
#define imu_led_pin 13

// Địa chỉ thanh ghi ICM20602
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define CONFIG 0x1A
#define ACCEL_CONFIG2 0x1D


// Khai báo biến
SPISettings settings(SPI_CLOCK, MSBFIRST, SPI_MODE0);

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;


float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

void setup_icm_20602() 
{
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // Khởi tạo cảm biến ICM20602
  ICM20602_Init();
  
  // ==================== BẮT ĐẦU AUTO CALIBRATION GYRO ====================
  pinMode(imu_led_pin, OUTPUT);
  analogWrite(imu_led_pin, 0);
  float n = 0;
  float step = 1; // Biến điều khiển chiều tăng/giảm độ sáng LED

  float RateCalibrationRoll_1 = 0, RateCalibrationPitch_1 = 0, RateCalibrationYaw_1 = 0;
  float RateCalibrationRoll_2 = 0, RateCalibrationPitch_2 = 0, RateCalibrationYaw_2 = 0;

  Serial.println("Bat dau hieu chinh Gyro. Vui long giu yen Drone!");

  while(1) 
  {
    RateCalibrationRoll_2   = RateCalibrationRoll_1;
    RateCalibrationPitch_2  = RateCalibrationPitch_1;
    RateCalibrationYaw_2    = RateCalibrationYaw_1;
    
    RateCalibrationRoll_1 = 0;
    RateCalibrationPitch_1 = 0;
    RateCalibrationYaw_1 = 0;

    for (RateCalibrationNumber = 0; RateCalibrationNumber < 500; RateCalibrationNumber++) 
    {
      gyro_signals();
      RateCalibrationRoll_1  += RateRoll;   // Sử dụng biến RateRoll ông đã tính
      RateCalibrationPitch_1 += RatePitch;
      RateCalibrationYaw_1   += RateYaw;
      
      delayMicroseconds(50); // Cho nó thở một tí, nếu để 10us có thể đọc trùng mẫu cũ

      // --- HIỆU ỨNG ĐÈN NHỊP THỞ (BREATHING LED) ---
      analogWrite(imu_led_pin, n); 
      n += step;
      if (n <= 0 || n >= 250) { // Sửa lại logic nhịp thở mượt hơn từ 0-250
        step = -step; 
      }
    }

    float rate_cali_roll_diff, rate_cali_pitch_diff, rate_cali_yaw_diff;
    rate_cali_roll_diff  = fabsf(RateCalibrationRoll_1 - RateCalibrationRoll_2);
    rate_cali_pitch_diff = fabsf(RateCalibrationPitch_1 - RateCalibrationPitch_2);
    rate_cali_yaw_diff   = fabsf(RateCalibrationYaw_1 - RateCalibrationYaw_2);

    // Ngưỡng < 10 cho 500 mẫu (tức là lệch trung bình mỗi mẫu < 0.02 độ/s). 
    // Đây là mức cực kỳ nhạy và chuẩn xác.
    if(rate_cali_roll_diff < 10 && rate_cali_pitch_diff < 10 && rate_cali_yaw_diff < 10) 
    {
      Serial.println("Calib Gyro thanh cong!");  
      analogWrite(imu_led_pin, 0); // Tắt đèn khi calib xong
      break; 
    } 
    else 
    {
      Serial.println("Phat hien rung lac! Dang Calib lai...");
    }
  }

  // Chốt thông số Calib cuối cùng
  RateCalibrationRoll  = RateCalibrationRoll_1 / 500.0;
  RateCalibrationPitch = RateCalibrationPitch_1 / 500.0;
  RateCalibrationYaw   = RateCalibrationYaw_1 / 500.0;
}

void loop_icm_20602() 
{
  gyro_signals();
  
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  
  roll= KalmanAngleRoll;
  pitch= KalmanAnglePitch;
  // Serial.print("R: ");
  // Serial.print(KalmanAngleRoll);
  // Serial.print(" P: ");
  // Serial.println(KalmanAnglePitch);
}

void ICM20602_Init() 
{
  // Thoát chế độ sleep
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);
  
  // Cấu hình bộ lọc và thang đo (tương tự MPU6050 code)
  // writeRegister(CONFIG, 0x05);           // Bộ lọc tần số thấp 10Hz
  writeRegister(CONFIG, 0x06);     // 5.1Hz - độ trễ 16ms (có thể chấp nhận)
  // writeRegister(CONFIG, 0x07);     // 2.5Hz - độ trễ 32ms (quá chậm cho điều khiển)

  writeRegister(ACCEL_CONFIG, 0x10);     // Thang đo gia tốc ±8g
  writeRegister(GYRO_CONFIG, 0x08);      // Thang đo gyro ±500dps
  writeRegister(ACCEL_CONFIG2, 0x05);    // Bộ lọc gia tốc
  delay(100);
}

void gyro_signals(void) 
{
  // Đọc gia tốc
  int16_t AccXLSB, AccYLSB, AccZLSB;
  readAccel(AccXLSB, AccYLSB, AccZLSB);
  
  // Đọc gyro
  int16_t GyroX, GyroY, GyroZ;
  readGyro(GyroX, GyroY, GyroZ);
  
  // Chuyển đổi gyro (thang đo ±500dps)
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  // Chuyển đổi gia tốc (thang đo ±8g)
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AccX = AccX + offset_acc_x;
  AccY = AccY + offset_acc_y;
  AccZ = AccZ + offset_acc_z;

  
  // Tính góc từ gia tốc kế
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) 
{
  KalmanState = KalmanState + 0.005 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.005 * 0.005 * 1 * 1;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

void writeRegister(uint8_t reg, uint8_t value) 
{
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(settings);
  SPI.transfer(reg & 0x7F); // Bit 7 là 0 cho ghi
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
}

uint8_t readRegister(uint8_t reg) 
{
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(settings);
  SPI.transfer(reg | 0x80); // Bit 7 là 1 cho đọc
  uint8_t value = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return value;
}

void readAccel(int16_t &x, int16_t &y, int16_t &z) 
{
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(settings);
  SPI.transfer(ACCEL_XOUT_H | 0x80);
  x = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  y = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  z = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
}

void readGyro(int16_t &x, int16_t &y, int16_t &z) 
{
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(settings);
  SPI.transfer(GYRO_XOUT_H | 0x80);
  x = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  y = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  z = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
}