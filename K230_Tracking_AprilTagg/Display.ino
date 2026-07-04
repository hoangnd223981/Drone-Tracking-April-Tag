float loop_timer;
unsigned long last_timer;
unsigned long display_timer;

const char *get_flight_mode_name()
{
  if(active_flight_mode == MODE_ANGLE) return "ANGLE";
  if(active_flight_mode == MODE_ALT_HOLD) return "ALT_HOLD";
  if(active_flight_mode == MODE_LOITER) return "LOITER";
  if(active_flight_mode == MODE_APRILTAG) return "APRILTAG";
  return "NO_FLY";
}

void display()
{
  loop_timer = (float)(micros() - last_timer) / 1000000.0f;
  last_timer = micros();

  acc_x_trung_binh = acc_x_trung_binh * 0.998f + AccX * 0.002f;
  acc_y_trung_binh = acc_y_trung_binh * 0.998f + AccY * 0.002f;
  acc_z_trung_binh = acc_z_trung_binh * 0.998f + AccZ * 0.002f;

  if(millis() - display_timer < 100)
  {
    return;
  }
  display_timer = millis();

  Serial.print("MODE:");
  Serial.print(get_flight_mode_name());

  Serial.print(" I2C:");
  Serial.print(apriltag_i2c_started ? 1 : 0);

  Serial.print(" READY:");
  Serial.print(apriltag_i2c_ready() ? 1 : 0);

  Serial.print(" SEARCH:");
  Serial.print(apriltag_searching() ? 1 : 0);

  Serial.print(" TAG:");
  if(apriltag_tag_valid)
  {
    Serial.print(apriltag_tag_id);
  }
  else
  {
    Serial.print(-1);
  }

  Serial.print(" EX:");
  Serial.print(apriltag_error_right_cm, 1);

  Serial.print(" EY:");
  Serial.print(apriltag_error_forward_cm, 1);

  Serial.print(" H:");
  Serial.print(apriltag_height_cm, 1);

  Serial.print(" YAW:");
  Serial.print(apriltag_heading_deg, 1);

  float debug_cmd_forward = apriltag_searching() ? get_apriltag_search_cmd_forward() : apriltag_cmd_forward_cm_s;
  float debug_cmd_right = apriltag_searching() ? get_apriltag_search_cmd_right() : apriltag_cmd_right_cm_s;
  float debug_cmd_up = apriltag_searching() ? get_apriltag_search_cmd_up() : apriltag_cmd_up_cm_s;
  float debug_cmd_yaw = apriltag_searching() ? get_apriltag_search_cmd_yaw() : apriltag_cmd_yaw_deg_s;

  Serial.print(" VF:");
  Serial.print(debug_cmd_forward, 1);

  Serial.print(" VR:");
  Serial.print(debug_cmd_right, 1);

  Serial.print(" VZ:");
  Serial.print(debug_cmd_up, 1);

  Serial.print(" WY:");
  Serial.print(debug_cmd_yaw, 1);

  Serial.print(" Q:");
  Serial.print(apriltag_quality);

  Serial.print(" OK:");
  Serial.print(apriltag_packet_ok);

  Serial.print(" ERR:");
  Serial.print(apriltag_packet_error);

  Serial.print(" LOOP_US:");
  Serial.println(loop_timer * 1000000.0f, 0);
}
