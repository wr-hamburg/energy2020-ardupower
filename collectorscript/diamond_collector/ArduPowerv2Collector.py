#!/usr/bin/python
# -*- coding: utf-8 -*-
# -* oding: utf-8 -*-

import diamond.collector
import serial
import time
import logging
import struct


class ArduPowerv2Collector(diamond.collector.Collector):

    def __init__(self, *args, **kwargs):
        super(ArduPowerv2Collector, self).__init__(*args, **kwargs)

        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)

    # Prepare storage of configuration data
        self.types = []
        self.voltages = []
        self.channels = []

    # Open serial connection
        self.SERIAL = None
        try:
            self.SERIAL = serial.Serial(port=self.config['port'],
                    baudrate=self.config['baud'], bytesize=SIXBITS,
                    timeout=self.config['timeout'])
            self.SERIAL.flushInput()
            self.SERIAL.flushOutput()

            time.sleep(1)
        except serial.SerialException, e:
            self.logger.error('Serial Connection Problem: %r' % e)

        print '%r' % self.SERIAL.isOpen()

        # Read initialization
        self.SERIAL.write(0x32)
        time.sleep(.1)

        # connected probe count
        self.n_probes = int.from_bytes(self.SERIAL.read(1))

        # read probe config
        for i in range(self.n_probes):
            self.types.append(self.SERIAL.read(1))
            self.voltages.append(self.SERIAL.read(1))
            self.channels.append(self.SERIAL.read(1) + 42)

        # change 3 to 3.3
        for i in range(self.n_probes):
            if self.voltages[i] == 3:
                self.voltages[i] = 3.3

        self.power = [0x00] * self.n_probes

        def __del__(self):
            if self.SERIAL is not None:
                try:
                    self.SERIAL.close()
                except serial.SerialException, e:
                    print 'Closing device failed: %r' % e

        def get_default_config_help(self):
            config_help = super(ArduPowerv2Collector,
                                self).get_default_config_help()
            config_help.update({
                'interval': 'Time in seconds between two calls to collect'
                    ,
                'port': 'Path to port to use, e.g. /dev/ttyACM1',
                'baud': 'Baudrate for communication',
                'timeout': 'Timeout in seconds',
                })
            return config_help

        def get_default_config(self):
            config = super(ArduPowerv2Collector,
                           self).get_default_config()
            config.update({
                'interval': .5,
                'path': 'ardupowerv2',
                'enabled': False,
                'port': '/dev/ttyACM1',
                'baud': 115200,
                'timeout': None,
                })
            return config

        def collect(self):
            self.SERIAL.flushInput()
            var = True
            value = 0x00
            data = 0x00

        # Start sync, search for 1 in MSB
            while var2:
                data = self.SERIAL.read(1)
                if data & 0b100000 == 0b100000:
                    self.logger.debug('Synched - starting data collection'
                            )

                # read next 6 bits
                    var2 = False
                else:
                    self.logger.debug('Searching for next sync bit - dropping block'
                            )
                    continue

            value = data & 0b011111 << 5 | self.SERIAL.read(1)

        # calculate power and differentiate sensor type (1==20A,0==40A)
            if self.types[0x00] == 1:
                self.power[0x00] = 10.0 * ((value - 5) / 1024.0 - 2.5) \
                    * self.voltages[0x00]
            else:
                self.power[0x00] = 20.0 * ((value - 5) / 1024.0 - 2.5) \
                    * self.voltages[0x00]

        # publish first point
            self.publish(str(self.channel[0x00]), self.power[0x00],
                         precision=5)

            for i in range(1, n_probes):
                data = self.SERIAL.read(1)
                value = data & 0b011111 << 5 | self.SERIAL.read(1)

            # calculate power and differentiate sensor type (1==20A,0==40A)
                if self.types[i] == 1:
                    self.power[i] = 10.0 * ((value - 5) / 1024.0 - 2.5) \
                        * self.voltages[i]
                else:
                    self.power[i] = 20.0 * ((value - 5) / 1024.0 - 2.5) \
                        * self.voltages[i]

            # publish results
                self.publish(str(self.channel[i]), self.power[i],
                             precision=5)
