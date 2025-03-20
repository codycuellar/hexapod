from time import sleep, ticks_ms
import math
from hexapod import interpolation
from hexapod.leg import Leg, ServoConfig
from pimoroni import Button
from servo import Servo, servo2040

def lerp(v1:float, v2:float, t:float):
    return v1 + (v2 - v1) * t

USER_BUTTON = Button(servo2040.USER_SW)

UPDATES_PER_SEC = 50
WALK_CYCLE_TIME = 2000


l1_coxa = ServoConfig(servo2040.SERVO_1, 80, -80, 0)
l1_femur = ServoConfig(servo2040.SERVO_2, -47, 90, 27.65, True)
l1_tibia = ServoConfig(servo2040.SERVO_3, -70, 90, 0, True)
leg = Leg(l1_coxa, l1_femur, l1_tibia, femur_len=65, tibia_len=90)
start_time = ticks_ms()

p1 = [90, 50, -100]
p2 = [30, -50, -100]
p3 = [(90+30)/2, 0, 0]

sleep(1)
while not USER_BUTTON.raw():
    ticks = ticks_ms()
    elapsed = (ticks - start_time) % WALK_CYCLE_TIME
    cycle_t = elapsed / WALK_CYCLE_TIME  # Normalize time to 0 → 1

    if (cycle_t < .5):
        t = cycle_t * 2
        pos_x = lerp(p1[0], p2[0], t)
        pos_y = lerp(p1[1], p2[1], t)
        pos_z = lerp(p1[2], p2[2], t)
    else:
        t = (cycle_t - .5) * 2
        pos_x = interpolation.quad_bez(p2[0], p3[0], p1[0], t)
        pos_y = interpolation.quad_bez(p2[1], p3[1], p1[1], t)
        pos_z = interpolation.quad_bez(p2[2], p3[2], p1[2], t)

    angles = leg.set_position(pos_x, pos_y, pos_z)

    sleep(1 / UPDATES_PER_SEC)

leg.zero_servos()

