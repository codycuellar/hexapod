import gc
import sys
import uselect
from servo import ServoCluster, servo2040


def main():
    gc.collect()

    # Use ONE cluster to save all hardware resources
    pins = list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
    cluster = ServoCluster(0, 0, pins)

    spoll = uselect.poll()
    spoll.register(sys.stdin, uselect.POLLIN)

    print("servo connected")

    while True:
        if spoll.poll(0):
            command = sys.stdin.readline().strip()
            if not command:
                continue

            try:
                angles_raw = command.split(",")
                for angle_raw in angles_raw:
                    if ":" not in angle_raw:
                        continue
                    pin, angle = angle_raw.split(":")
                    cluster.value(int(pin), float(angle))
            except Exception as e:
                # Log to serial so you see it on your PC immediately
                print("RUNTIME_ERROR:", e)
                log_to_file("Runtime: " + str(e))


def log_to_file(error_msg):
    try:
        with open("log.txt", "a") as f:  # "a" for append so you don't lose old logs
            f.write(str(error_msg) + "\n")
    except:
        # If we can't write to file (e.g. no memory), at least we tried
        pass


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # This catches "Fatal" crashes that stop the whole script
        print("FATAL_ERROR:", e)
        log_to_file("Fatal: " + str(e))
