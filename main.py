from hexapod.path_drawing import walk_cycle
from hexapod.interpolation import lerp_3d
from hexapod.leg import Leg, ServoConfig
from hexapod.utils import Coord3D

from pimoroni import Button
from servo import Servo, servo2040

from time import sleep, ticks_ms

USER_BUTTON = Button(servo2040.USER_SW)

UPDATES_PER_SEC = 30
WALK_CYCLE_TIME = 2000

leg_dimensions = { 'coxa_len': 40, 'femur_len': 65, 'tibia_len': 90 }


leg1 = Leg(ServoConfig(servo2040.SERVO_7, 80, -80, 0), 
           ServoConfig(servo2040.SERVO_8, -90, 90, -20, True), 
           ServoConfig(servo2040.SERVO_9, 70, -70, 85, False),
           leg_rotation_offset=-60,
           leg_translation_offset=Coord3D(25, 25, 0),
           **leg_dimensions)

leg2 = Leg(ServoConfig(servo2040.SERVO_4, 80, -80, 0), 
           ServoConfig(servo2040.SERVO_5, -90, 90, -20, True), 
           ServoConfig(servo2040.SERVO_6, 70, -70, 85, False), 
           **leg_dimensions)

leg3 = Leg(ServoConfig(servo2040.SERVO_1, 80, -80, 0), 
           ServoConfig(servo2040.SERVO_2, -90, 90, -20, True), 
           ServoConfig(servo2040.SERVO_3, 70, -70, 85, False),
           leg_rotation_offset=60,
           leg_translation_offset=Coord3D(25, -25, 0),
           **leg_dimensions)

start_time = ticks_ms()

walk_cycle_points = [
    Coord3D(100, 20, -60),
    Coord3D(100, -20, -60),
    Coord3D(100, 0, 25)
]


def reset():
    leg1.zero_servos()  # Example, set to a neutral position
    leg2.zero_servos()
    leg3.zero_servos()
    sleep(1)
    leg1.zero_servos()  # Example, set to a neutral position
    leg2.zero_servos()
    leg3.zero_servos()
    
sleep(1)
try:
    while not USER_BUTTON.raw():
        # leg1.zero_servos()
        # leg2.zero_servos()
        # leg3.zero_servos()

        ticks = ticks_ms()
        elapsed = (ticks - start_time) % WALK_CYCLE_TIME
        cycle_t = elapsed / WALK_CYCLE_TIME  # Normalize time to 0 → 1

        point = walk_cycle(cycle_t, *walk_cycle_points)
        point_offset = walk_cycle((cycle_t + 0.5) % 1, *walk_cycle_points)

        leg1.set_position(point)
        leg2.set_position(point_offset)
        leg3.set_position(point)
        
        sleep(1 / UPDATES_PER_SEC)
except KeyboardInterrupt:
    reset()
    print("Program interrupted. Servos stopped.")

reset()
