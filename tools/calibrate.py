#!/usr/bin/env python3

import argparse
import bisect
import collections
import csv
import dataclasses
import datetime
import math
import os
import statistics
import sys

@dataclasses.dataclass
class Data:
    time: float            # seconds
    voltage: float         # uV
    power: float           # uW
    current: float = None  # uA

    # calculated by integrating
    charge: float = None   # uAh
    energy: float = None   # uWh
    capacity: float = None # 0.0 - 1.0

    def __post_init__(self):
        # FIXME remove once current is provided
        if self.current is None:
            # careful, uA = 1_000_000 * uW / uV
            self.current = 1_000_000.0 * self.power / self.voltage

    def copy(self):
        return dataclasses.replace(self)

def read_data(files):
    data = []
    for fname in files:
        with open(fname) as f:
            reader = csv.DictReader(f)
            for row in reader:
                data.append(Data(**{k: float(v) for k, v in row.items()}))

    return data

def integrate(data):
    prev = data[0]
    prev.charge = 0.0
    prev.energy = 0.0
    for prev, d in zip(data, data[1:]):
        dt = (d.time - prev.time) / (60.0 * 60.0)
        aI = 0.5 * (d.current + prev.current)
        aW = 0.5 * (d.power + prev.power)
        d.charge = prev.charge + aI * dt
        d.energy = prev.energy + aW * dt

    last = data[-1]
    for d in data:
        d.capacity = 1.0 - d.energy / last.energy

def smooth(data, interval):
    window = collections.deque()
    for i, d in enumerate(data):
        window.append(d)

        while abs(d.capacity - window[0].capacity) > interval:
            window.popleft()

        d = d.copy()
        d.voltage = statistics.mean(x.voltage for x in window)
        d.charge = statistics.mean(x.charge for x in window)
        d.energy = statistics.mean(x.energy for x in window)
        d.capacity = statistics.mean(x.capacity for x in window)
        data[i] = d

    while window:
        first = window[0]
        first.voltage = statistics.mean(x.voltage for x in window)
        first.charge = statistics.mean(x.charge for x in window)
        first.energy = statistics.mean(x.energy for x in window)
        first.capacity = statistics.mean(x.capacity for x in window)
        data.append(window.popleft())

def simplify(data, tolerance):
    newdata = [data[0], data[-1]]
    newkeys = [-d.voltage for d in newdata]

    while True:
        worst = None
        worsterr = tolerance
        worsti = None
        for d in data:
            i = bisect.bisect(newkeys, -d.voltage)
            if newkeys[i - 1] == -d.voltage:
                # do not make a non-function
                continue
            puti = i
            if i == 0:
                # extrapolate back
                i += 1
            if i == len(newdata):
                #extrapolate forward
                i -= 1
            a = newdata[i - 1]
            b = newdata[i]
            dV = b.voltage - a.voltage
            dC = b.capacity - a.capacity
            predicted = a.capacity + (d.voltage - a.voltage) * dC / dV
            err = abs(predicted - d.capacity)
            if err > worsterr:
                worst = d
                worsterr = err
                worsti = puti

        if worst is None:
            break

        newdata.insert(worsti, worst)
        newkeys.insert(worsti, -worst.voltage)

    data[:] = newdata

def plot(data, simple):
    import matplotlib.pyplot as plt

    last = data[-1]
    Vs = [d.voltage / 1_000_000.0 for d in data]
    Cs = [d.capacity for d in data]

    Vs_simp = [d.voltage / 1_000_000.0 for d in simple]
    Cs_simp = [d.capacity for d in simple]

    plt.plot(Vs, Cs, label='raw')
    plt.plot(Vs_simp, Cs_simp, label='simplified')
    plt.xlabel('voltage')
    plt.ylabel('capacity')
    plt.legend()

    plt.show()

def asciiplot(data, width=70, height=20):
    hmargin = len("100% | ")
    vmargin = len("v1 -")
    plot_width = width - hmargin
    plot_height = height - vmargin
    plot = [[' '] * plot_width for _ in range(plot_height)]

    # decide on ticks
    vmin = min(d.voltage for d in data)
    vmax = max(d.voltage for d in data)
    def hpos(v):
        h = int(plot_width * (v - vmin) / (vmax - vmin))
        if h == plot_width:
            h -= 1
        return h
    def vpos(c):
        v = int(plot_height * c)
        if v == plot_height:
            v -= 1
        return v
    ticks = list(range(math.ceil(vmin / 1_000_000.0), math.floor(vmax / 1_000_000.0) + 1))
    tickpos = [hpos(t * 1_000_000) for t in ticks]

    # do plotting
    for d in data:
        h = hpos(d.voltage)
        v = vpos(d.capacity)
        plot[v][h] = '~'

    # render plot into lines
    rendered = []
    for i, line in enumerate(plot):
        if i == 0:
            label = "  0% | "
        elif i == len(plot) - 1:
            label = "100% | "
        else:
            label = "     | "
        rendered.append(label + ''.join(line))

    rendered.reverse()

    # bottom axis
    rendered.append("     +-" + '-' * plot_width)

    # tick marks
    rendered.append("       " + ''.join(['|' if i in tickpos else ' ' for i in range(plot_width)]))

    # tick labels
    labels = ''
    while len(labels) < plot_width:
        i = len(labels)
        if i in tickpos:
            j = tickpos.index(i)
            labels += '{}'.format(int(ticks[j]))
        labels += ' '
    rendered.append("       " + labels)

    # bottom axis label
    label = 'voltage'
    centering = (plot_width - len(label)) / 2.0
    label = ' ' * math.floor(centering) + label + ' ' * math.ceil(centering)
    rendered.append("       " + label)

    # sanity check
    assert(all([len(l) == width for l in rendered]))
    assert(len(rendered) == height)

    return rendered

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs='+', help='the recorded data to calibrate with')
    parser.add_argument('--plot', action='store_true', help='plot the resulting discharge profile')
    parser.add_argument('--interval', type=float, help='smoothing interval, in capacity', default=0.05)
    parser.add_argument('--tolerance', type=float, help='capacity error tolerance when simplifying', default=0.001)
    parser.add_argument('--decimate', type=int, help='decimation factor for data, for faster results', default=1)
    args = parser.parse_args()

    #print("reading...", file=sys.stderr)
    data = read_data(args.files)
    data = data[::args.decimate]
    #print("integrating...", file=sys.stderr)
    integrate(data)
    simple = [d.copy() for d in data]
    #print("smoothing...", file=sys.stderr)
    smooth(simple, interval=args.interval)
    #print("simplifying...", file=sys.stderr)
    simplify(simple, tolerance=args.tolerance)

    def printboth(*args):
        if os.fstat(0) != os.fstat(1):
            # only print to stderr if output is redirected
            print(*args, file=sys.stderr)
        if args and args != ('',):
            print('  ', *args)
        else:
            print()

    print('/* calibration table generated {}'.format(datetime.datetime.now().isoformat()))

    # stats
    printboth('')
    last = data[-1]
    printboth('table represented within tolerance {:.1f}% by {} points'.format(args.tolerance * 100.0, len(simple)))
    printboth('')
    printboth('capacity: {:.2f}\tAh'.format(last.charge / 1_000_000.0))
    printboth('          {:.2f}\tWh'.format(last.energy / 1_000_000.0))

    # table
    printboth('')
    for line in asciiplot(simple):
        printboth(line.rstrip())

    print(' */')

    def v(x):
        return int(round(x))

    print('')
    print('#define SHERKBAT_VOLTAGE_MIN {}'.format(v(min(d.voltage for d in simple))))
    print('#define SHERKBAT_VOLTAGE_MAX {}'.format(v(max(d.voltage for d in simple))))
    print('#define SHERKBAT_CHARGE_FULL {}'.format(v(simple[-1].charge)))
    print('#define SHERKBAT_ENERGY_FULL {}'.format(v(simple[-1].energy)))
    print('')

    print('static const struct sherkbat_table_entry sherkbat_table[] = {')
    for d in sorted(simple, key=lambda d: -d.voltage):
        print('    {')
        print('        .capacity = {},'.format(v(d.capacity * 1000)))
        print('        .voltage = {},'.format(v(d.voltage)))
        print('        .charge = {},'.format(v(simple[-1].charge - d.charge)))
        print('        .energy = {},'.format(v(simple[-1].energy - d.energy)))
        print('    },')
    print('};')

    if args.plot:
        plot(data, simple)

if __name__ == '__main__':
    main()
