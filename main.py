from hexapod.body import Body
from hexapod.utils import Transform3D, Coord, Vector
from hexapod.leg import Leg, Servo

from pimoroni import Button
from servo import servo2040

from time import sleep, ticks_ms

USER_BUTTON = Button(servo2040.USER_SW)
WALK_CYCLE_TIME = 1500

leg_dimensions = {"coxa_len": 40, "femur_len": 65, "tibia_len": 90}

cluster = Servo.create_cluster(list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1)))
hexapod = Body(
    Leg(
        "Right Front",
        Servo("Coxa", cluster, servo2040.SERVO_1, 30, -60, -5, True),
        Servo("Femur", cluster, servo2040.SERVO_2, 60, -45, -26, True),
        Servo("Tibia", cluster, servo2040.SERVO_3, 10, 180, 108),
        Transform3D(Coord(48.5, 90.5, 0), 60),
        **leg_dimensions,
    ),
    Leg(
        "Right Center",
        Servo("Coxa", cluster, servo2040.SERVO_4, 45, -45, -6, True),
        Servo("Femur", cluster, servo2040.SERVO_5, 60, -45, -30, True),
        Servo("Tibia", cluster, servo2040.SERVO_6, 10, 180, 105),
        Transform3D(Coord(102.5, 3.5, 0)),
        **leg_dimensions,
    ),
    Leg(
        "Right Back",
        Servo("Coxa", cluster, servo2040.SERVO_7, 60, -30, 0, True),
        Servo("Femur", cluster, servo2040.SERVO_8, 60, -45, -20, True),
        Servo("Tibia", cluster, servo2040.SERVO_9, 10, 180, 90),
        Transform3D(Coord(54, -87.5, 0), -60),
        **leg_dimensions,
    ),
    Leg(
        "Left Front",
        Servo("Coxa", cluster, servo2040.SERVO_10, 60, -30, 2, True),
        Servo("Femur", cluster, servo2040.SERVO_11, 60, -45, -22),
        Servo("Tibia", cluster, servo2040.SERVO_12, 10, 180, 90, True),
        Transform3D(Coord(-48.5, 90.5, 0), 120),
        **leg_dimensions,
    ),
    Leg(
        "Left Center",
        Servo("Coxa", cluster, servo2040.SERVO_13, 45, -45, -5, True),
        Servo("Femur", cluster, servo2040.SERVO_14, 60, -45, -25),
        Servo("Tibia", cluster, servo2040.SERVO_15, 10, 180, 90, True),
        Transform3D(Coord(-102.5, 3.5, 0), 180),
        **leg_dimensions,
    ),
    Leg(
        "Left Back",
        Servo("Coxa", cluster, servo2040.SERVO_16, 30, -60, 0, True),
        Servo("Femur", cluster, servo2040.SERVO_17, 60, -45, -20),
        Servo("Tibia", cluster, servo2040.SERVO_18, 10, 180, 90, True),
        Transform3D(Coord(-54, -87.5, 0), -120),
        **leg_dimensions,
    ),
    initial_position=Transform3D(Coord(0, 0, 30)),
    update_frequency=1 / 50,
)

start_time = ticks_ms()

try:
    hexapod.go_to_home()
    hexapod.update_velocity(Vector(0.1, 0, 0))
    while not USER_BUTTON.raw():
        hexapod.update()
        sleep(hexapod.update_frequency)

except KeyboardInterrupt:
    print("Program interrupted.")
