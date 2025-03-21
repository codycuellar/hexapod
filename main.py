from time import sleep, ticks_ms
import math
from hexapod import interpolation
from hexapod.leg import Leg, ServoConfig
from pimoroni import Button
from servo import Servo, servo2040

USER_BUTTON = Button(servo2040.USER_SW)

UPDATES_PER_SEC = 50
WALK_CYCLE_TIME = 2000


l1_coxa = ServoConfig(servo2040.SERVO_1, 80, -80, 0)
l1_femur = ServoConfig(servo2040.SERVO_2, -47, 90, 45, True)
l1_tibia = ServoConfig(servo2040.SERVO_3, 70, -70, 80, False)
leg = Leg(l1_coxa, l1_femur, l1_tibia, coxa_len=40, femur_len=65, tibia_len=90)
start_time = ticks_ms()


walk_cycle_points = [
    [100, 50, -40],
    [100, -50, -40],
    [100, 0, 0]
]
def walk_cycle(t):
    if (t < .5):
        t = t * 2
        pos_x = interpolation.lerp(walk_cycle_points[0][0], walk_cycle_points[1][0], t)
        pos_y = interpolation.lerp(walk_cycle_points[0][1], walk_cycle_points[1][1], t)
        pos_z = interpolation.lerp(walk_cycle_points[0][2], walk_cycle_points[1][2], t)
    else:
        t = (t - .5) * 2
        pos_x = interpolation.quad_bez(walk_cycle_points[1][0], walk_cycle_points[2][0], walk_cycle_points[0][0], t)
        pos_y = interpolation.quad_bez(walk_cycle_points[1][1], walk_cycle_points[2][1], walk_cycle_points[0][1], t)
        pos_z = interpolation.quad_bez(walk_cycle_points[1][2], walk_cycle_points[2][2], walk_cycle_points[0][2], t)
    
    return pos_x, pos_y, pos_z

circle_center_point = [100, 0, -40]
def circle_pattern(t):
    x = circle_center_point[0] + 50 * math.cos(t)
    y = circle_center_point[1] + 50 * math.sin(t)
    z = circle_center_point[2]
    return x, y, z

sleep(1)
while not USER_BUTTON.raw():
    ticks = ticks_ms()
    elapsed = (ticks - start_time) % WALK_CYCLE_TIME
    cycle_t = elapsed / WALK_CYCLE_TIME  # Normalize time to 0 → 1

    # walk_cycle(cycle_t)
    point = circle_pattern(cycle_t * 2 * math.pi)
    leg.set_position(*point)

    sleep(1 / UPDATES_PER_SEC)

leg.zero_servos()
