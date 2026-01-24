from enum import Enum


class LogLevel(Enum):
    INFO = "INFO"
    WARN = "WARNING"
    CRIT = "CRITICAL"
    FATAL = "FATAL"


def log_to_file(level: LogLevel, error_msg):
    try:
        with open("log.txt", "a") as f:  # "a" for append so you don't lose old logs
            f.write(str(level) + ": " + str(error_msg) + "\n")
    except:
        # If we can't write to file (e.g. no memory), at least we tried
        pass
