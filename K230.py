
# UART 10 byte:
# Byte 0   : Sign/State
#            bit0 = dau Vx       (1 = am)
#            bit1 = dau Vy       (1 = am)
#            bit2 = dau Vz_down  (1 = am)
#            bit3 = dau YawRate  (1 = am)
#            bit4..bit7 = FlightCommand
# Byte 1-2 : Vx_body_cm_s
# Byte 3-4 : Vy_body_cm_s
# Byte 5-6 : Vz_down_cm_s
# Byte 7-8 : YawRate_dps
# Byte 9   : Checksum XOR byte 0..8

import time, math, gc
from machine import UART, FPIOA
import image
from math import pi, isnan

from media.display import *
from media.sensor import *
from media.media import *

# ==================== LOP PID ====================
class PID:
    def __init__(self, p=0.0, i=0.0, d=0.0, imax=0.0):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = float('nan')
        self._last_error = 0.0
        self._last_t = 0
        self._integrator = 0.0
        self._RC = 1.0 / (2.0 * pi * 20.0)

    def get_pid(self, error, scaler=1.0):
        tnow = time.ticks_ms()
        dt = time.ticks_diff(tnow, self._last_t)
        output = 0.0

        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()

        self._last_t = tnow
        delta_time = float(dt) / 1000.0

        output += error * self._kp

        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0.0
                self._last_derivative = 0.0
            else:
                derivative = (error - self._last_error) / delta_time

            derivative = self._last_derivative + (delta_time / (self._RC + delta_time)) * (derivative - self._last_derivative)
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative

        output *= scaler

        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time

            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax

            output += self._integrator

        return output

    def reset_I(self):
        self._integrator = 0.0
        self._last_derivative = float('nan')
        self._last_error = 0.0

# ==================== FLIGHT COMMAND ====================
CMD_HOLD     = 0
CMD_TRACK    = 1
CMD_DESCEND  = 2
CMD_SPIRAL   = 3
CMD_LAND     = 4
CMD_FAILSAFE = 5

# ==================== HAM PHU ====================
def constrain(v, vmin, vmax):
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v

def wrap_180(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg

def to_uint16_abs_and_sign(v):
    iv = int(round(v))

    if iv < -32767:
        iv = -32767
    elif iv > 32767:
        iv = 32767

    negative = iv < 0
    return abs(iv), negative

# ==================== CAU HINH UART ====================
# K230 TX pin 9  -> ESP32 RX
# K230 RX pin 10 -> ESP32 TX
# K230 GND       -> ESP32 GND
fpioa = FPIOA()
fpioa.set_function(9, FPIOA.UART1_TXD)
fpioa.set_function(10, FPIOA.UART1_RXD)

UART_BAUD = 115200
uart = UART(UART.UART1, baudrate=UART_BAUD, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

# True: ESP gui 'H', K230 moi tra goi 10 byte
# False: K230 tu gui dinh ky
HANDSHAKE_ENABLE = True

# ==================== CAU HINH CAMERA ====================
gc.collect()

sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=400, height=240)
sensor.set_pixformat(Sensor.RGB565)

FRAME_WIDTH = 400
FRAME_HEIGHT = 240

MediaManager.init()
Display.init(Display.ST7701, width=640, height=480, to_ide=True)
sensor.run()

tag_families = image.TAG36H11

# ==================== THAM SO APRILTAG ====================
SETPOINT_CX = FRAME_WIDTH / 2
SETPOINT_CY = FRAME_HEIGHT / 2

# Theo bai bao: AprilTag 16 cm x 16 cm
TAG_REAL_SIZE_CM = 16.0

# Can hieu chinh bang calibration camera thuc te
FOCAL_LENGTH_PX = 300.0

TARGET_HEADING_DEG = 0.0

def estimate_distance_cm(tag):
    w = tag.w()
    if w > 0:
        return (FOCAL_LENGTH_PX * TAG_REAL_SIZE_CM) / w
    return 100.0

# ==================== PID TRACKING TREN K230 ====================
# K230 tinh lenh vx/vy/yaw, ESP chi thuc hien
PID_X = PID(p=0.10, i=0.01, d=0.006, imax=25.0)
PID_Y = PID(p=0.10, i=0.01, d=0.006, imax=25.0)
PID_HEADING = PID(p=1.20, i=0.05, d=0.03, imax=20.0)

# ==================== THAM SO BAY ====================
CENTER_X_TH = 18          # pixel
CENTER_Y_TH = 14          # pixel
LAND_DIST_CM = 35         # cm
DESCEND_SPEED_CM_S = 18   # >0 ha thap

MAX_VX_CM_S = 35
MAX_VY_CM_S = 35
MAX_YAW_RATE_DPS = 45

LOST_HOLD_MS = 400
LOST_FAILSAFE_MS = 10000

SPIRAL_START_R = 10.0
SPIRAL_R_RATE = 4.0
SPIRAL_OMEGA = 1.2
SPIRAL_YAW_RATE_DPS = 15.0

# Doi dau neu huong camera/body nguoc
# vx > 0: bay thang toi
# vy > 0: bay ngang phai
VX_DIR = 1.0
VY_DIR = 1.0
YAW_DIR = 1.0

last_tag_time = time.ticks_ms()
spiral_t0 = time.ticks_ms()
last_uart_send = time.ticks_ms()

# ==================== UART PACKET ====================
def send_flight_packet(cmd, vx_cm_s, vy_cm_s, vz_down_cm_s, yaw_rate_dps):
    """
    Gui goi 10 byte K230 -> ESP.

    cmd:
        0 HOLD
        1 TRACK
        2 DESCEND
        3 SPIRAL
        4 LAND
        5 FAILSAFE

    vx_cm_s:
        >0 bay thang toi, <0 bay lui
    vy_cm_s:
        >0 bay ngang phai, <0 bay ngang trai
    vz_down_cm_s:
        >0 ha thap, =0 giu do cao
    yaw_rate_dps:
        >0 quay phai, <0 quay trai
    """

    abs_vx, sign_vx = to_uint16_abs_and_sign(vx_cm_s)
    abs_vy, sign_vy = to_uint16_abs_and_sign(vy_cm_s)
    abs_vz, sign_vz = to_uint16_abs_and_sign(vz_down_cm_s)
    abs_yaw, sign_yaw = to_uint16_abs_and_sign(yaw_rate_dps)

    sign_state = 0

    if sign_vx:
        sign_state |= (1 << 0)
    if sign_vy:
        sign_state |= (1 << 1)
    if sign_vz:
        sign_state |= (1 << 2)
    if sign_yaw:
        sign_state |= (1 << 3)

    sign_state |= (cmd & 0x0F) << 4

    packet = bytearray(10)
    packet[0] = sign_state

    packet[1] = (abs_vx >> 8) & 0xFF
    packet[2] = abs_vx & 0xFF

    packet[3] = (abs_vy >> 8) & 0xFF
    packet[4] = abs_vy & 0xFF

    packet[5] = (abs_vz >> 8) & 0xFF
    packet[6] = abs_vz & 0xFF

    packet[7] = (abs_yaw >> 8) & 0xFF
    packet[8] = abs_yaw & 0xFF

    checksum = 0
    for i in range(9):
        checksum ^= packet[i]

    packet[9] = checksum & 0xFF

    uart.write(packet)

def uart_has_handshake():
    if uart.any() <= 0:
        return False

    data = uart.read()

    if data is None:
        return False

    return b'H' in data

# ==================== SPIRAL SEARCH ====================
def calc_spiral_command():
    """
    K230 tu tinh lenh xoan oc khi mat tag.
    ESP chi bay theo vx/vy/yaw_rate nhan duoc.
    """
    now = time.ticks_ms()
    t = time.ticks_diff(now, spiral_t0) / 1000.0

    r = SPIRAL_START_R + SPIRAL_R_RATE * t
    theta = SPIRAL_OMEGA * t

    vx = SPIRAL_R_RATE * math.cos(theta) - r * SPIRAL_OMEGA * math.sin(theta)
    vy = SPIRAL_R_RATE * math.sin(theta) + r * SPIRAL_OMEGA * math.cos(theta)

    vx = constrain(vx, -MAX_VX_CM_S, MAX_VX_CM_S)
    vy = constrain(vy, -MAX_VY_CM_S, MAX_VY_CM_S)

    yaw_rate = SPIRAL_YAW_RATE_DPS

    return vx, vy, yaw_rate

# ==================== MAIN LOOP ====================
clock = time.clock()

TARGET_PERIOD_MS = 20     # 50 Hz target loop
UART_PERIOD_MS = 40       # 25 Hz neu tu gui

while True:
    clock.tick()
    loop_start = time.ticks_ms()

    cmd = CMD_HOLD
    vx_cmd = 0.0
    vy_cmd = 0.0
    vz_cmd = 0.0
    yaw_cmd = 0.0

    img = sensor.snapshot()
    tags = img.find_apriltags(families=tag_families)

    if tags:
        tag = tags[0]
        last_tag_time = time.ticks_ms()
        spiral_t0 = last_tag_time

        cx = tag.cx()
        cy = tag.cy()
        rotation_rad = tag.rotation()
        rotation_deg = (180.0 * rotation_rad) / math.pi
        dist_cm = estimate_distance_cm(tag)

        img.draw_rectangle(tag.rect(), color=(255, 0, 0), thickness=4)
        img.draw_cross(cx, cy, color=(0, 255, 0), thickness=2)

        err_x = SETPOINT_CX - cx
        err_y = SETPOINT_CY - cy
        err_heading = wrap_180(TARGET_HEADING_DEG - rotation_deg)

        # Anh luu y:
        # err_y anh xa sang vx: tag o tren/duoi anh -> bay thang/lui
        # err_x anh xa sang vy: tag o trai/phai anh -> bay ngang
        vx_cmd = VX_DIR * PID_Y.get_pid(err_y)
        vy_cmd = VY_DIR * PID_X.get_pid(err_x)
        yaw_cmd = YAW_DIR * PID_HEADING.get_pid(err_heading)

        vx_cmd = constrain(vx_cmd, -MAX_VX_CM_S, MAX_VX_CM_S)
        vy_cmd = constrain(vy_cmd, -MAX_VY_CM_S, MAX_VY_CM_S)
        yaw_cmd = constrain(yaw_cmd, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS)

        if abs(err_x) < CENTER_X_TH and abs(err_y) < CENTER_Y_TH:
            vx_cmd = 0.0
            vy_cmd = 0.0

            if dist_cm > LAND_DIST_CM:
                cmd = CMD_DESCEND
                vz_cmd = DESCEND_SPEED_CM_S
            else:
                cmd = CMD_LAND
                vz_cmd = 0.0
        else:
            cmd = CMD_TRACK
            vz_cmd = 0.0

    else:
        PID_X.reset_I()
        PID_Y.reset_I()
        PID_HEADING.reset_I()

        lost_time = time.ticks_diff(time.ticks_ms(), last_tag_time)

        if lost_time < LOST_HOLD_MS:
            cmd = CMD_HOLD
            vx_cmd = 0.0
            vy_cmd = 0.0
            vz_cmd = 0.0
            yaw_cmd = 0.0
        elif lost_time < LOST_FAILSAFE_MS:
            cmd = CMD_SPIRAL
            vx_cmd, vy_cmd, yaw_cmd = calc_spiral_command()
            vz_cmd = 0.0
        else:
            cmd = CMD_FAILSAFE
            vx_cmd = 0.0
            vy_cmd = 0.0
            vz_cmd = 0.0
            yaw_cmd = 0.0

    Display.show_image(img, x=120, y=120)

    if HANDSHAKE_ENABLE:
        if uart_has_handshake():
            send_flight_packet(cmd, vx_cmd, vy_cmd, vz_cmd, yaw_cmd)
    else:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_uart_send) >= UART_PERIOD_MS:
            last_uart_send = now
            send_flight_packet(cmd, vx_cmd, vy_cmd, vz_cmd, yaw_cmd)

    elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
    remaining = TARGET_PERIOD_MS - elapsed

    if remaining > 0:
        time.sleep_ms(remaining)
