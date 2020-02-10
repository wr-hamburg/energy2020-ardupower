import serial
import struct
import time
import signal
import sys
import argparse
import io

"""
 *  ArduPower v2 Reference Data Collector and Benchmarking Code
 *  Code by Daniel Guenther Bremer
 *  Part of bachelor's thesis "Optimizing ArduPower"
"""

SERIAL_INTERFACE = "/dev/ttyACM0"

# global variable for the serial interface
ser = None
outtarget = None


# function to handle quitting the program with Ctrl+C
# closes the serial connection, stops the measurement and resets the input buffer
def ctrlc_handler(sigcode, frame):
    global ser
    print('Aborting!')
    ser.write(struct.pack("!b", 0x30))
    ser.flush()
    ser.reset_input_buffer()
    ser.close()
    print('Stopped measurement and closed Serial Connection!')
    # Close file if any
    if outtarget is not sys.stdout:
        outtarget.write("Aborted!")
        outtarget.close()


def collector(args, file):
    global ser
    voltages = []
    types = []
    channels = []

    if file is None:
        outtarget = sys.stdout
    else:
        outtarget = io.open(file, "w")

    try:  # open the serial connection
        ser = serial.Serial(port=SERIAL_INTERFACE,
                            baudrate=115200,
                            bytesize=serial.SIXBITS,
                            timeout=None)
    except serial.SerialException as e:
        print('Serial Connection Problem: %r' % e)
        exit(-1)

    time.sleep(3)  # wait for Arduino reboot
    ser.write(struct.pack("!b", 0x32))  # send '2' to get config
    outtarget.write("reading configuration\n")
    tmp = struct.unpack("!b", ser.read(1))  # read the count of connected probes
    n_probes = tmp[0]
    outtarget.write("n_probes = " + str(n_probes) + '\n')

    volt = 0
    for i in range(n_probes):  # read the probe configurations
        tmp = struct.unpack("!b", ser.read(1))
        types.append(tmp[0])
        tmp = struct.unpack("!b", ser.read(1))
        if tmp[0] is 1:
            volt = 12;
        elif tmp[0] is 2:
            volt = 5;
        elif tmp[0] is 3:
            volt = 3.3
        elif tmp[0] is 4:
            volt = 230
        else:
            outtarget.write('Unknown voltage on probe ' + str(i) + '\n')
            volt = 0
        voltages.append(volt)
        tmp = struct.unpack("!b", ser.read(1))
        channels.append(tmp[0])

    for a in range(n_probes):  # print configuration
        outtarget.write('Channel ' + str(channels[a]) + '@' + str(voltages[a]) + 'V, type ' + str(types[a]) + '\n')
    time.sleep(5)
    ser.write(struct.pack("!b", 0x31))  # enable measurement

    var = True
    value = 0x0000
    data = 0x00

    # Start sync, search for 1 in MSB
    while var:
        tmp = struct.unpack("!b", ser.read(1))
        data = tmp[0]
        if data & 0b100000 is 0b100000:
            outtarget.write('Synched - starting data collection\n')
            var = False
            # ignore first data set
            for i in range(n_probes - 1):
                tmp = struct.unpack("!b", ser.read(1))
        else:
            outtarget.write('Searching for next sync bit - dropping block\n')
            continue

        index = 0

    line = ""

    while True:  # start reading loop
        line = str(time.time()) + ';' + str(index) + ';'
        index = index + 1
        for i in range(n_probes):  # read a whole iteration of data collection
            tmp = struct.unpack("!b", ser.read(1))
            if i == 0 and tmp[0] & 0b100000 != 0b100000:
                # Connection out of sync!
                outtarget.write("Out of sync!\n")
                while True:  # read until synched again
                    tmp = struct.unpack("!b", ser.read(1))
                    if tmp[0] & 0b100000:
                        # Found sync
                        break

            data = tmp[0] & 0b011111  # save HIGH bits
            tmp = struct.unpack("!b", ser.read(1))
            value = (data << 5) | tmp[0]  # recover value
            if args.type is 'p':  # print the power consumption values
                if types[i] is 1:  # calculate power consumption for 20A sensors
                    value = (-10.0 * ((value * 5) / 1024.0 - 2.5) * voltages[i])
                else:  # calculate power for 40A sensor
                    value = (-20.0 * ((value * 5) / 1024.0 - 2.5) * voltages[i])
            line = line + "{:.2f};".format(value)
        if args.output is 'n':
            continue
        outtarget.write(line+'\n')


def main():
    signal.signal(signal.SIGINT, ctrlc_handler)
    parser = argparse.ArgumentParser(description="This script collects values provided by ArduPower v2.",
                                     formatter_class=argparse.RawTextHelpFormatter)

    outputgroup = parser.add_argument_group("Required Arguments")
    outputgroup.add_argument("-o", "--output",
                             choices=["c", "f", "n"],
                             help="Provide output method.\n"
                                  + "\tc - output to stdout\n"
                                  + "\tf - output to file. Requires -f"
                                  + "\tn - no output (load simulation only)",
                             required=True)
    outputgroup.add_argument("-t", "--type",
                             choices=["p", "r"],
                             help="Set value type of output.\n"
                                  + "\tp - power draw\n"
                                  + "\tr - raw ADC values",
                             required=True)
    parser.add_argument("-f", "--file", help="Parameter is only required if '-o f' is used and ignored otherwise.\n"
                                             + "\tProvide a path to a file that should contain the captured values.")

    args = parser.parse_args()

    # Check if file is provided when using '-o f'/'--output f'
    if args.output == "f" and args.file is None:
        parser.error("When using '-o f' the parameter '-f /path/to/file' has to be specified!")

    if args.output is 'c' or args.output is 'n':
            print('Starting data collection - printing to stdout')
            collector(args, None)
    elif args.output is 'f':
            print('Starting data collection - writing to ' + args.file)
            collector(args, args.file)
    sys.exit(-1)


if __name__ == "__main__":
    main()
