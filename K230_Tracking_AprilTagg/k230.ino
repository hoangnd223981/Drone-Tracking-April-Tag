/* ===================== K230 UART RECEIVER ===================== */
#define K230_BAUD                 115200
#define K230_RX_PIN               16
#define K230_TX_PIN               17
#define K230_PACKET_SIZE          10
#define K230_REQUEST_PERIOD_MS    20
#define K230_TIMEOUT_MS           500

#define CMD_HOLD                  0
#define CMD_TRACK                 1
#define CMD_DESCEND               2
#define CMD_SPIRAL                3
#define CMD_LAND                  4
#define CMD_FAILSAFE              5

#define SIGN_VX_BIT               0
#define SIGN_VY_BIT               1
#define SIGN_VZ_BIT               2
#define SIGN_YAW_BIT              3

HardwareSerial k230Serial(2);

uint8_t K230_LinkOK = 0;
uint8_t K230_FlightCmd = CMD_HOLD;

int16_t K230_Vx_cm_s = 0;
int16_t K230_Vy_cm_s = 0;
int16_t K230_VzDown_cm_s = 0;
int16_t K230_YawRate_dps = 0;

uint32_t K230_LastUpdate = 0;
uint32_t K230_PacketCount = 0;
uint32_t K230_ChecksumError = 0;

static uint8_t k230_rx_buf[K230_PACKET_SIZE];
static uint8_t k230_rx_index = 0;
static uint32_t k230_last_request = 0;

/* Neu may bay di nguoc huong thi doi dau tai day */
float K230_VX_DIR = 1.0f;
float K230_VY_DIR = 1.0f;
float K230_YAW_DIR = 1.0f;

/* Neu toc do qua manh thi giam scale xuong 0.5 hoac 0.3 */
float K230_XY_SCALE = 1.0f;
float K230_Z_SCALE = 1.0f;
float K230_YAW_SCALE = 1.0f;

static int16_t K230_DecodeSigned(uint16_t abs_value, uint8_t sign_state, uint8_t sign_bit)
{
  if (sign_state & (1 << sign_bit))
  {
    return -(int16_t)abs_value;
  }

  return (int16_t)abs_value;
}

static uint8_t K230_ChecksumOK(uint8_t *buf)
{
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < 9; i++)
  {
    checksum ^= buf[i];
  }

  return checksum == buf[9];
}

static uint8_t K230_DecodePacket(uint8_t *buf)
{
  if (!K230_ChecksumOK(buf))
  {
    K230_ChecksumError++;
    return 0;
  }

  uint8_t sign_state = buf[0];

  K230_FlightCmd = (sign_state >> 4) & 0x0F;

  uint16_t vx_abs  = ((uint16_t)buf[1] << 8) | buf[2];
  uint16_t vy_abs  = ((uint16_t)buf[3] << 8) | buf[4];
  uint16_t vz_abs  = ((uint16_t)buf[5] << 8) | buf[6];
  uint16_t yaw_abs = ((uint16_t)buf[7] << 8) | buf[8];

  K230_Vx_cm_s = K230_DecodeSigned(vx_abs, sign_state, SIGN_VX_BIT);
  K230_Vy_cm_s = K230_DecodeSigned(vy_abs, sign_state, SIGN_VY_BIT);
  K230_VzDown_cm_s = K230_DecodeSigned(vz_abs, sign_state, SIGN_VZ_BIT);
  K230_YawRate_dps = K230_DecodeSigned(yaw_abs, sign_state, SIGN_YAW_BIT);

  K230_LinkOK = 1;
  K230_LastUpdate = millis();
  K230_PacketCount++;

  return 1;
}

void K230_Init(void)
{
  k230Serial.begin(K230_BAUD, SERIAL_8N1, K230_RX_PIN, K230_TX_PIN);

  K230_LinkOK = 0;
  K230_FlightCmd = CMD_HOLD;
  K230_LastUpdate = millis();
  k230_rx_index = 0;
  k230_last_request = millis();
}

void K230_Task(void)
{
  uint32_t now = millis();

  if ((uint32_t)(now - k230_last_request) >= K230_REQUEST_PERIOD_MS)
  {
    k230_last_request = now;
    k230Serial.write('H');
  }

  while (k230Serial.available() > 0)
  {
    uint8_t b = (uint8_t)k230Serial.read();

    k230_rx_buf[k230_rx_index++] = b;

    if (k230_rx_index >= K230_PACKET_SIZE)
    {
      k230_rx_index = 0;
      K230_DecodePacket(k230_rx_buf);
    }
  }

  if ((uint32_t)(now - K230_LastUpdate) > K230_TIMEOUT_MS)
  {
    K230_LinkOK = 0;
    K230_FlightCmd = CMD_HOLD;

    K230_Vx_cm_s = 0;
    K230_Vy_cm_s = 0;
    K230_VzDown_cm_s = 0;
    K230_YawRate_dps = 0;
  }
}