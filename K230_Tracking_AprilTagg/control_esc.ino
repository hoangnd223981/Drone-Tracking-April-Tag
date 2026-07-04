/* ===================== ESC PIN ===================== */
#define esc_1_pin 12
#define esc_2_pin 27
#define esc_3_pin 26
#define esc_4_pin 25
// #define esc_5_pin 35 
// #define esc_6_pin 34
// #define esc_7_pin 35
// #define esc_8_pin 34

/* ===================== ESC CHANNEL ===================== */
const int esc_1_channel = 0;
const int esc_2_channel = 1;
const int esc_3_channel = 2;
const int esc_4_channel = 3;
const int esc_5_channel = 4;
const int esc_6_channel = 5;
const int esc_7_channel = 6;
const int esc_8_channel = 7;

/* ===================== PWM CONFIG ===================== */
const int freq = 391;        // tương ứng 800–1600 → 1000–2000
const int resolution = 11;

/* ===================== SETUP MOTOR ===================== */
void setup_motor() 
{
  ledcSetup(esc_1_channel, freq, resolution);
  ledcAttachPin(esc_1_pin, esc_1_channel);

  ledcSetup(esc_2_channel, freq, resolution);
  ledcAttachPin(esc_2_pin, esc_2_channel);

  ledcSetup(esc_3_channel, freq, resolution);
  ledcAttachPin(esc_3_pin, esc_3_channel);

  ledcSetup(esc_4_channel, freq, resolution);
  ledcAttachPin(esc_4_pin, esc_4_channel);

  // ledcSetup(esc_5_channel, freq, resolution);
  // ledcAttachPin(esc_5_pin, esc_5_channel);

  // ledcSetup(esc_6_channel, freq, resolution);
  // ledcAttachPin(esc_6_pin, esc_6_channel);

  // ledcSetup(esc_7_channel, freq, resolution);
  // ledcAttachPin(esc_7_pin, esc_7_channel);

  // ledcSetup(esc_8_channel, freq, resolution);
  // ledcAttachPin(esc_8_pin, esc_8_channel);

  control_motor(800, 800, 800, 800);
  delay(1000);

  control_motor(800, 800, 800, 800);
  delay(5000);
}

/* ===================== CONTROL MOTOR ===================== */
void control_motor(int m1, int m2, int m3, int m4)
{


  ledcWrite(esc_1_channel, m1);
  ledcWrite(esc_2_channel, m2);
  ledcWrite(esc_3_channel, m3);
  ledcWrite(esc_4_channel, m4);
}