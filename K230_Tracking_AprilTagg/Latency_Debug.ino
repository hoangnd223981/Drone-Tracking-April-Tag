#define LAT_I2C_RX_PIN       32
#define LAT_CONTROL_PIN      33

volatile bool latency_packet_received = false;
volatile bool latency_packet_processed = false;
volatile bool latency_control_done = false;
#define LAT_DEBUG_HOLD_MS 100


void setup_latency_debug()
{
  pinMode(LAT_I2C_RX_PIN, OUTPUT);
  pinMode(LAT_CONTROL_PIN, OUTPUT);

  digitalWrite(LAT_I2C_RX_PIN, LOW);
  digitalWrite(LAT_CONTROL_PIN, LOW);
}

void latency_i2c_packet_start()
{
  digitalWrite(LAT_I2C_RX_PIN, HIGH);
  latency_packet_received = true;
  latency_packet_processed = false;
  latency_control_done = false;
}

void latency_packet_processed_mark()
{
  if(latency_packet_received)
  {
    latency_packet_processed = true;
  }
}

void latency_control_mark()
{
  if(latency_packet_processed)
  {
    digitalWrite(LAT_CONTROL_PIN, HIGH);
    latency_control_done = true;
  }
}

void latency_debug_reset()
{
  if(latency_control_done)
  {
    delay(LAT_DEBUG_HOLD_MS);

    digitalWrite(LAT_I2C_RX_PIN, LOW);
    digitalWrite(LAT_CONTROL_PIN, LOW);

    latency_packet_received = false;
    latency_packet_processed = false;
    latency_control_done = false;
  }
}

void latency_debug_abort()
{
  digitalWrite(LAT_I2C_RX_PIN, LOW);
  digitalWrite(LAT_CONTROL_PIN, LOW);

  latency_packet_received = false;
  latency_packet_processed = false;
  latency_control_done = false;
}