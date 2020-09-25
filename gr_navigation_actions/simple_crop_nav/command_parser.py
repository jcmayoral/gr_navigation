#!/usr/bin/python


def parse_command(commands):
    for c in commands:
        if "start" in c:
            return "START_TEST"
        if "stop" in c:
            return "STOP_TEST"
        if "next" in c:
            return "NEXT_TEST"

    return None

def parse_command(command, False):
    if "start" in command:
        return "START_TEST"
    if "stop" in command:
        return "STOP_TEST"
    if "next" in command:
        return "NEXT_TEST"

    return None
