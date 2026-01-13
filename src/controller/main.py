# from pimoroni import Button
from servo import ServoCluster, servo2040

import sys, select, time
import gc


gc.collect()

time.sleep(1)
print("Servo2040 online")


poll = select.poll()
poll.register(sys.stdin, select.POLLIN)


cluster_1_pins = list(p for p in range(servo2040.SERVO_1, servo2040.SERVO_6 + 1))
cluster_2_pins = list(p for p in range(servo2040.SERVO_1, servo2040.SERVO_6 + 1))
cluster_3_pins = list(p for p in range(servo2040.SERVO_1, servo2040.SERVO_6 + 1))

clusters = {
    "1": ServoCluster(0, 0, cluster_1_pins),
    "2": ServoCluster(0, 1, cluster_2_pins),
    "3": ServoCluster(0, 1, cluster_3_pins),
}


pin_mappings = {}

for pin in cluster_1_pins:
    pin_mappings[str(pin)] = "1"

for pin in cluster_1_pins:
    pin_mappings[str(pin)] = "2"

for pin in cluster_1_pins:
    pin_mappings[str(pin)] = "3"


def get_cluster(pin_num: str) -> ServoCluster:
    cluster_num = pin_mappings[pin_num]
    return clusters[cluster_num]


while True:
    if poll.poll(0):
        line = sys.stdin.readline().strip()
        angles_raw = line.split(",")
        for angle_raw in angles_raw:
            pin, angle = angle_raw.split(":")
            cluster = get_cluster(pin)
            cluster.value(int(pin), float(angle))
            cluster_num = pin_mappings[pin]
            print("moving servo", pin, "in cluster", cluster, "with angle", angle)
