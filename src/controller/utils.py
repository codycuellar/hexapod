class LogLevel:
    INFO = 0
    WARN = 1
    CRIT = 2
    FATAL = 3


def clear_log():
    with open("log.txt", "w") as f:
        f.write("")


def log_to_file(level: int, error_msg):
    l = "INFO"
    if level == LogLevel.WARN:
        l = "WARN"
    elif level == LogLevel.CRIT:
        l = "CRITICAL"
    elif level == LogLevel.FATAL:
        l = "FATAL"

    try:
        with open("log.txt", "a") as f:  # "a" for append so you don't lose old logs
            f.write(l + ": " + str(error_msg) + "\n")
    except:
        # If we can't write to file (e.g. no memory), at least we tried
        pass
