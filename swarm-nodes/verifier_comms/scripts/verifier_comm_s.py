#!/usr/bin/env python3
from radio_object import commsHandler


if __name__ == '__main__':
    comms = commsHandler()
    comms.main_loop()
