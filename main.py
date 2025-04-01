from hexapod.path_drawing import walk_cycle
from hexapod.interpolation import lerp_3d
from hexapod.body import Body
from hexapod.leg import Leg, ServoConfig
from hexapod.utils import Coord3D

from pimoroni import Button
from servo import Servo, servo2040

from time import sleep, ticks_ms

USER_BUTTON = Button(servo2040.USER_SW)

UPDATES_PER_SEC = 40
WALK_CYCLE_TIME = 3000

leg_dimensions = { 'coxa_len': 40, 'femur_len': 65, 'tibia_len': 90 }
hexapod = Body(
    Leg(ServoConfig(servo2040.SERVO_1, 30, -60, -5, True), 
        ServoConfig(servo2040.SERVO_2, 60, -45, -26, True), 
        ServoConfig(servo2040.SERVO_3, 10, 180, 112),
        **leg_dimensions),
    Leg(ServoConfig(servo2040.SERVO_4, 80, -80, 0, True), 
        ServoConfig(servo2040.SERVO_5, -90, 90, -20, True), 
        ServoConfig(servo2040.SERVO_6, 70, -70, 85, False), 
        **leg_dimensions),
    Leg(ServoConfig(servo2040.SERVO_7, 80, -80, 0), 
        ServoConfig(servo2040.SERVO_8, -90, 90, -20, True), 
        ServoConfig(servo2040.SERVO_9, 70, -70, 85, False),
        **leg_dimensions),
    Leg(ServoConfig(servo2040.SERVO_10, -30, 60, 2),
        ServoConfig(servo2040.SERVO_11, 90, -40, -20, True), 
        ServoConfig(servo2040.SERVO_12, 70, -70, 85, False),
        **leg_dimensions),
    Leg(ServoConfig(servo2040.SERVO_4, 80, -80, 0), 
        ServoConfig(servo2040.SERVO_5, -90, 90, -20, True), 
        ServoConfig(servo2040.SERVO_6, 70, -70, 85, False), 
        **leg_dimensions),
    Leg(ServoConfig(servo2040.SERVO_1, 80, -80, 0), 
        ServoConfig(servo2040.SERVO_2, -90, 90, -20, True), 
        ServoConfig(servo2040.SERVO_3, 70, -70, 85, False),
        **leg_dimensions)
)

hexapod.set_legs_active([True, False, False, False, False, False])

start_time = ticks_ms()
    
sleep(1)
try:
    while not USER_BUTTON.raw():
        ticks = ticks_ms()
        elapsed = (ticks - start_time) % WALK_CYCLE_TIME
        cycle_t = elapsed / WALK_CYCLE_TIME  # Normalize time to 0 → 1

        hexapod.walk(cycle_t)
        sleep(1 / UPDATES_PER_SEC)

except KeyboardInterrupt:
    print("Program interrupted.")


