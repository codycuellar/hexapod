from hexapod.body import Body
from hexapod.leg import Leg, Servo

from pimoroni import Button
from servo import servo2040, ServoCluster

from time import sleep, ticks_ms

USER_BUTTON = Button(servo2040.USER_SW)

UPDATES_PER_SEC = 40
WALK_CYCLE_TIME = 1500

leg_dimensions = { 'coxa_len': 40, 'femur_len': 65, 'tibia_len': 90 }

cluster = ServoCluster(0, 0, list(range(servo2040.SERVO_1, servo2040.SERVO_18+1)))
hexapod = Body(
    Leg('Right Front',
        Servo('Coxa', cluster, servo2040.SERVO_1, 30, -60, -5, True), 
        Servo('Femur', cluster, servo2040.SERVO_2, 60, -45, -26, True), 
        Servo('Tibia', cluster, servo2040.SERVO_3, 10, 180, 108),
        **leg_dimensions),
    Leg('Right Center',
        Servo('Coxa', cluster,  servo2040.SERVO_4, 45, -45, -6, True), 
        Servo('Femur', cluster, servo2040.SERVO_5, 60, -45, -30, True), 
        Servo('Tibia', cluster, servo2040.SERVO_6, 10, 180, 105), 
        **leg_dimensions),
    Leg('Right Back',
        Servo('Coxa', cluster, servo2040.SERVO_7, 60, -30, 0, True), 
        Servo('Femur', cluster, servo2040.SERVO_8, 60, -45, -20, True), 
        Servo('Tibia', cluster, servo2040.SERVO_9, 10, 180, 90),
        **leg_dimensions),
    Leg('Left Front',
        Servo('Coxa', cluster, servo2040.SERVO_10, 60, -30, 2, True),
        Servo('Femur', cluster, servo2040.SERVO_11, 60, -45, -22), 
        Servo('Tibia', cluster, servo2040.SERVO_12, 10, 180, 90, True),
        **leg_dimensions),
    Leg('Left Center',
        Servo('Coxa', cluster, servo2040.SERVO_13, 45, -45, -5, True), 
        Servo('Femur', cluster, servo2040.SERVO_14, 60, -45, -25), 
        Servo('Tibia', cluster, servo2040.SERVO_15, 10, 180, 90, True), 
        **leg_dimensions),
    Leg('Left Back',
        Servo('Coxa', cluster, servo2040.SERVO_16, 30, -60, 0, True), 
        Servo('Femur', cluster, servo2040.SERVO_17, 60, -45, -20), 
        Servo('Tibia', cluster, servo2040.SERVO_18, 10, 180, 90, True),
        **leg_dimensions)
)

# hexapod.set_legs_active([True, True, True, True, True, False])

start_time = ticks_ms()
    
sleep(1)
try:
    while not USER_BUTTON.raw():
        ticks = ticks_ms()
        elapsed = (ticks - start_time) % WALK_CYCLE_TIME
        cycle_t = elapsed / WALK_CYCLE_TIME  # Normalize time to 0 → 1

        hexapod.shimmy(cycle_t)
        sleep(1 / UPDATES_PER_SEC)

except KeyboardInterrupt:
    print("Program interrupted.")


