#!/usr/bin/python


def parse_command(commands):
    for c in commands:
        if "start" in c:
            return "START_TEST"
        if "stop" in c:
            return "STOP_TEST"

    return None
