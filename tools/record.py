#!/usr/bin/env python3

import csv
import os.path
import sys
import time

base = '/sys/class/power_supply/sherkbat'

def readval(name):
    with open(os.path.join(base, name)) as f:
        return int(f.read().strip())

def main():
    writer = csv.DictWriter(sys.stdout, ['time', 'voltage', 'current', 'power'])
    writer.writeheader()
    sys.stdout.flush()
    while True:
        time.sleep(10)
        writer.writerow({
            'time': time.time(),
            'voltage': readval('voltage_now'),
            'power': readval('power_now'),
            'current': readval('current_now'),
        })
        sys.stdout.flush()

if __name__ == '__main__':
    main()
