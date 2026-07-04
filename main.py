# AprilTag TAG36H11 - K230 -> ESP32 qua I2C
# Ban test latency bang oscilloscope
# CH2 -> GPIO14 = lat_frame: bat dau lay anh frame duoc gui
# CH1 -> GPIO17 = lat_tx: bat dau gui I2C
# Do tre K230 = canh len GPIO17 - canh len GPIO14

import time
import math
import os
import gc
import image

from machine import I2C, Pin
from media.sensor import *
from media.display import *
from media.media import *

# =========================================================
# LCD / CAMERA
# =========================================================
LCD_WIDTH = 800
LCD_HEIGHT = 480

PROCESS_WIDTH = 144
PROCESS_HEIGHT = 144
ROI_X = (LCD_WIDTH - PROCESS_WIDTH) // 2
ROI_Y = (LCD_HEIGHT - PROCESS_HEIGHT) // 2
PROCESS_ROI = (ROI_X, ROI_Y, PROCESS_WIDTH, PROCESS_HEIGHT)
ROI_CENTER_X = ROI_X + PROCESS_WIDTH // 2
ROI_CENTER_Y = ROI_Y + PROCESS_HEIGHT // 2

TAG_FAMILY = image.TAG36H11
TARGET_TAG_ID = 9            # Dat -1 de chap nhan moi ID

# =========================================================
# CAMERA / APRILTAG CALIBRATION
# =========================================================
TAG_SIZE_CM = 13.5

FOCAL_X_PX = 500.0
FOCAL_Y_PX = 500.0

TARGET_HEIGHT_CM = 80.0
TARGET_HEADING_DEG = 0.0

DESCEND_ONLY = True
ENABLE_YAW_COMMAND = False

RIGHT_ERROR_SIGN = 1.0
FORWARD_ERROR_SIGN = -1.0
YAW_ERROR_SIGN = 1.0

# =========================================================
# OUTER PID - K230
# =========================================================
XY_MAX_SPEED_CM_S = 100.0
Z_MAX_SPEED_CM_S = 60.0
YAW_MAX_RATE_DEG_S = 60.0

XY_DEADBAND_CM = 1.5
Z_DEADBAND_CM = 2.0
YAW_DEADBAND_DEG = 2.0

KP_X = 1.20
KI_X = 0.02
KD_X = 0.00

KP_Y = 1.20
KI_Y = 0.02
KD_Y = 0.00

KP_Z = 0.80
KI_Z = 0.02
KD_Z = 0.00

KP_YAW = 1.50
KI_YAW = 0.01
KD_YAW = 0.00

# =========================================================
# I2C K230 MASTER -> ESP32 SLAVE
# =========================================================
I2C_BUS = 2
K230_SCL_PIN = 11
K230_SDA_PIN = 12

# Neu 100000 bi loi voi ESP32 thi doi ve 10000
I2C_FREQ = 100000

ESP32_I2C_ADDR = 0x42

# Test oscilloscope: gui 2 Hz de nhin song ro
# Chay that thi doi lai 50 ms
SEND_PERIOD_MS = 2000

# Giu xung TX them 20 ms de may analog de nhin
# Khong dung canh xuong de tinh latency, chi dung canh len
DEBUG_PULSE_HOLD_MS = 200

# =========================================================
# LATENCY DEBUG GPIO
# =========================================================
K230_LAT_FRAME_PIN = 14
K230_LAT_TX_PIN = 17

lat_frame = Pin(K230_LAT_FRAME_PIN, Pin.OUT)
lat_tx = Pin(K230_LAT_TX_PIN, Pin.OUT)

lat_frame.value(0)
lat_tx.value(0)

# =========================================================
# PACKET
# =========================================================
PACKET_HEADER_1 = 0xAA
PACKET_HEADER_2 = 0x55
PACKET_VERSION = 0x01
PACKET_LENGTH = 26

FLAG_TAG_VALID = 0x01

sensor = None
i2c = None


def constrain(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def wrap_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def apply_deadband(value, deadband):
    if abs(value) <= deadband:
        return 0.0
    return value


class PID:
    def __init__(self, kp, ki, kd, out_limit, i_limit, output_alpha=0.30):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = abs(out_limit)
        self.i_limit = abs(i_limit)
        self.output_alpha = output_alpha
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
        self.first_run = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
        self.first_run = True

    def update(self, error, dt):
        if dt < 0.005:
            dt = 0.005
        elif dt > 0.150:
            dt = 0.150

        p_term = self.kp * error

        self.integral += self.ki * error * dt
        self.integral = constrain(self.integral, -self.i_limit, self.i_limit)

        if self.first_run:
            derivative = 0.0
            self.first_run = False
        else:
            derivative = (error - self.prev_error) / dt

        raw_output = p_term + self.integral + self.kd * derivative
        raw_output = constrain(raw_output, -self.out_limit, self.out_limit)

        output = self.prev_output + self.output_alpha * (raw_output - self.prev_output)
        output = constrain(output, -self.out_limit, self.out_limit)

        self.prev_error = error
        self.prev_output = output

        return output


pid_right = PID(KP_X, KI_X, KD_X, XY_MAX_SPEED_CM_S, 25.0)
pid_forward = PID(KP_Y, KI_Y, KD_Y, XY_MAX_SPEED_CM_S, 25.0)
pid_height = PID(KP_Z, KI_Z, KD_Z, Z_MAX_SPEED_CM_S, 20.0)
pid_yaw = PID(KP_YAW, KI_YAW, KD_YAW, YAW_MAX_RATE_DEG_S, 20.0)


def reset_all_pid():
    pid_right.reset()
    pid_forward.reset()
    pid_height.reset()
    pid_yaw.reset()


def tag_side_size_px(tag):
    try:
        corners = tag.corners()
        total = 0.0

        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            dx = x2 - x1
            dy = y2 - y1
            total += math.sqrt(dx * dx + dy * dy)

        return total * 0.25

    except Exception:
        return 0.5 * (tag.w() + tag.h())


def estimate_height_cm(tag):
    side_px = tag_side_size_px(tag)

    if side_px < 2.0:
        return 0.0

    focal_mean = 0.5 * (FOCAL_X_PX + FOCAL_Y_PX)
    return (TAG_SIZE_CM * focal_mean) / side_px


def select_target_tag(tags):
    selected = None
    best_score = -1

    for tag in tags:
        if TARGET_TAG_ID >= 0 and tag.id() != TARGET_TAG_ID:
            continue

        dx = tag.cx() - ROI_CENTER_X
        dy = tag.cy() - ROI_CENTER_Y
        area = tag.w() * tag.h()
        center_penalty = dx * dx + dy * dy
        score = area * 1000 - center_penalty

        if selected is None or score > best_score:
            selected = tag
            best_score = score

    return selected


def put_i16_high_low(packet, index, value):
    value = int(constrain(round(value), -32768, 32767))

    if value < 0:
        value += 65536

    packet[index] = (value >> 8) & 0xFF
    packet[index + 1] = value & 0xFF


def put_u16_high_low(packet, index, value):
    value = int(constrain(value, 0, 65535))

    packet[index] = (value >> 8) & 0xFF
    packet[index + 1] = value & 0xFF


def build_packet(sequence, valid, tag_id, error_right_cm, error_forward_cm,
                 height_cm, heading_deg, cmd_fb, cmd_lr, cmd_up, cmd_yaw, quality):
    packet = bytearray(PACKET_LENGTH)

    packet[0] = PACKET_HEADER_1
    packet[1] = PACKET_HEADER_2
    packet[2] = PACKET_VERSION
    packet[3] = sequence & 0xFF
    packet[4] = FLAG_TAG_VALID if valid else 0
    packet[5] = 0

    put_u16_high_low(packet, 6, tag_id if tag_id >= 0 else 0xFFFF)

    put_i16_high_low(packet, 8, error_right_cm * 10.0)
    put_i16_high_low(packet, 10, error_forward_cm * 10.0)
    put_i16_high_low(packet, 12, height_cm * 10.0)
    put_i16_high_low(packet, 14, heading_deg * 10.0)

    put_i16_high_low(packet, 16, cmd_fb * 10.0)
    put_i16_high_low(packet, 18, cmd_lr * 10.0)
    put_i16_high_low(packet, 20, cmd_up * 10.0)
    put_i16_high_low(packet, 22, cmd_yaw * 10.0)

    packet[24] = int(constrain(quality, 0, 255))

    checksum = 0
    for i in range(PACKET_LENGTH - 1):
        checksum = (checksum + packet[i]) & 0xFF

    packet[25] = checksum

    return packet


def send_packet(packet):
    global i2c

    if i2c is None:
        return False

    try:
        sent = i2c.writeto(ESP32_I2C_ADDR, packet)

        if sent != len(packet):
            print("I2C short write:", sent, "/", len(packet))
            return False

        return True

    except Exception as e:
        print("I2C write error:", e)
        return False


def draw_text(img, x, y, text, size=16, color=(255, 255, 255)):
    img.draw_string_advanced(x, y, size, text, color=color)


def draw_status(img, fps, valid, error_right_cm, error_forward_cm,
                height_cm, heading_deg, cmd_fb, cmd_lr, cmd_up, cmd_yaw, i2c_ok):
    img.draw_rectangle(PROCESS_ROI, color=(255, 255, 255), thickness=2)
    img.draw_cross(ROI_CENTER_X, ROI_CENTER_Y, color=(200, 200, 200), size=8, thickness=2)

    draw_text(img, 10, 10, "FPS: %.1f" % fps, size=24)
    draw_text(img, 10, 40, "TAG: %s" % ("OK" if valid else "LOST"), size=24)
    draw_text(img, 10, 70, "I2C: %s" % ("OK" if i2c_ok else "ERR"), size=16)

    if valid:
        draw_text(img, 10, 95, "ER:%.1f EF:%.1fcm" % (error_right_cm, error_forward_cm), size=16)
        draw_text(img, 10, 115, "H:%.1f HD:%.1f" % (height_cm, heading_deg), size=16)
        draw_text(img, 10, 135, "FB:%.1f LR:%.1f" % (cmd_fb, cmd_lr), size=16)
        draw_text(img, 10, 155, "UP:%.1f YAW:%.1f" % (cmd_up, cmd_yaw), size=16)


try:
    i2c = I2C(
        I2C_BUS,
        scl=K230_SCL_PIN,
        sda=K230_SDA_PIN,
        freq=I2C_FREQ
    )

    print("I2C init OK, ESP32 addr = 0x%02X" % ESP32_I2C_ADDR)

    sensor = Sensor(width=LCD_WIDTH, height=LCD_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=LCD_WIDTH, height=LCD_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()

    fps_clock = time.clock()
    frame_counter = 0
    sequence = 0

    last_us = time.ticks_us()
    lost_frames = 0
    last_i2c_ok = False

    last_send_ms = time.ticks_ms()

    while True:
        fps_clock.tick()
        os.exitpoint()

        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, last_us) / 1000000.0
        last_us = now_us

        if dt <= 0.0 or dt > 0.2:
            dt = 0.03

        # Chon frame se gui I2C de danh dau oscilloscope
        now_ms_before_frame = time.ticks_ms()
        debug_this_frame = time.ticks_diff(now_ms_before_frame, last_send_ms) >= SEND_PERIOD_MS

        if debug_this_frame:
            # CH2 len: bat dau lay anh cua frame se gui
            lat_frame.value(1)

        img = sensor.snapshot()

        tags = img.find_apriltags(
            roi=PROCESS_ROI,
            families=TAG_FAMILY,
            fx=FOCAL_X_PX,
            fy=FOCAL_Y_PX,
            cx=LCD_WIDTH * 0.5,
            cy=LCD_HEIGHT * 0.5
        )

        target = select_target_tag(tags)

        valid = target is not None
        tag_id = -1

        error_right_cm = 0.0
        error_forward_cm = 0.0
        height_cm = 0.0
        heading_deg = 0.0

        cmd_fb = 0.0
        cmd_lr = 0.0
        cmd_up = 0.0
        cmd_yaw = 0.0

        quality = 0

        if valid:
            lost_frames = 0
            tag_id = target.id()
            height_cm = estimate_height_cm(target)

            pixel_error_x = target.cx() - ROI_CENTER_X
            pixel_error_y = target.cy() - ROI_CENTER_Y

            if height_cm > 0.0:
                error_right_cm = RIGHT_ERROR_SIGN * pixel_error_x * height_cm / FOCAL_X_PX
                error_forward_cm = FORWARD_ERROR_SIGN * pixel_error_y * height_cm / FOCAL_Y_PX

            try:
                heading_deg = wrap_angle_deg(target.z_rotation() * 180.0 / math.pi)
            except Exception:
                heading_deg = wrap_angle_deg(target.rotation() * 180.0 / math.pi)

            error_right_cm = apply_deadband(error_right_cm, XY_DEADBAND_CM)
            error_forward_cm = apply_deadband(error_forward_cm, XY_DEADBAND_CM)

            height_error_cm = apply_deadband(TARGET_HEIGHT_CM - height_cm, Z_DEADBAND_CM)

            heading_error_deg = apply_deadband(
                YAW_ERROR_SIGN * wrap_angle_deg(TARGET_HEADING_DEG - heading_deg),
                YAW_DEADBAND_DEG
            )

            cmd_lr = pid_right.update(error_right_cm, dt)
            cmd_fb = pid_forward.update(error_forward_cm, dt)

            if DESCEND_ONLY:
                if height_error_cm < 0.0:
                    cmd_up = pid_height.update(height_error_cm, dt)
                    cmd_up = constrain(cmd_up, -Z_MAX_SPEED_CM_S, 0.0)
                else:
                    pid_height.reset()
                    cmd_up = 0.0
            else:
                cmd_up = pid_height.update(height_error_cm, dt)

            if ENABLE_YAW_COMMAND:
                cmd_yaw = pid_yaw.update(heading_error_deg, dt)
            else:
                pid_yaw.reset()
                cmd_yaw = 0.0

            img.draw_rectangle(target.rect(), color=(255, 255, 255), thickness=4)
            img.draw_cross(target.cx(), target.cy(), color=(255, 255, 255), size=15, thickness=3)
            draw_text(img, target.x(), max(0, target.y() - 20), "ID:%d" % tag_id, size=24)

            try:
                quality = int(target.decision_margin())
            except Exception:
                quality = 0

        else:
            lost_frames += 1

            if lost_frames >= 3:
                reset_all_pid()

        # Gui goi I2C theo chu ky cham de de nhin song
        if debug_this_frame:
            packet = build_packet(
                sequence,
                valid,
                tag_id,
                error_right_cm,
                error_forward_cm,
                height_cm,
                heading_deg,
                cmd_fb,
                cmd_lr,
                cmd_up,
                cmd_yaw,
                quality
            )

            # CH1 len: bat dau gui I2C
            lat_tx.value(1)

            last_i2c_ok = send_packet(packet)

            # Giu xung de oscilloscope analog nhin ro
            # Khi tinh latency chi lay canh len CH2 den canh len CH1
            time.sleep_ms(DEBUG_PULSE_HOLD_MS)

            lat_tx.value(0)
            lat_frame.value(0)

            sequence = (sequence + 1) & 0xFF
            last_send_ms = time.ticks_ms()

        draw_status(
            img,
            fps_clock.fps(),
            valid,
            error_right_cm,
            error_forward_cm,
            height_cm,
            heading_deg,
            cmd_fb,
            cmd_lr,
            cmd_up,
            cmd_yaw,
            last_i2c_ok
        )

        if frame_counter % 10 == 0:
            print(
                "VALID:%d ID:%d ER:%.2f EF:%.2f H:%.2f HD:%.2f FB:%.2f LR:%.2f UP:%.2f YAW:%.2f I2C:%d" %
                (
                    1 if valid else 0,
                    tag_id,
                    error_right_cm,
                    error_forward_cm,
                    height_cm,
                    heading_deg,
                    cmd_fb,
                    cmd_lr,
                    cmd_up,
                    cmd_yaw,
                    1 if last_i2c_ok else 0
                )
            )

        Display.show_image(img)

        frame_counter += 1

        del tags
        del img

        if frame_counter % 30 == 0:
            gc.collect()

except KeyboardInterrupt:
    print("User stop")

except BaseException as e:
    print("Exception: %s" % str(e))

finally:
    lat_frame.value(0)
    lat_tx.value(0)

    if isinstance(sensor, Sensor):
        sensor.stop()

    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()

    if i2c is not None:
        try:
            i2c.deinit()
        except Exception:
            pass

    print("Program stopped")
