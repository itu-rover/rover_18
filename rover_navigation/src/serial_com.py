#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import os
import sys
import threading
import serial
import socket
from glob import glob
from settings import *

DEBUG = True

utility_names = {"SC": "Sensor Controller",
                 "R": "Robotic Arm",
                 "PR": "Prob",
                 "M":  "Motor Controller"}


class SerialNode(object):
    def __init__(self):
        self.ports = glob('/dev/ttyACM*') + glob('/dev/ttyUSB*')
        self.client = None
        self.msg_interval = WRITE_INTERVAL
        self.is_connected = False
        self.is_lost = False
        self.is_first = True
        self.configure_server()
        self.configure()

    def configure_server(self):
        self.server = socket.socket()
        self.server.bind((HOST, PORT))
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.listen(5)


    def configure(self):
        self.serials = []
        self.utilities = {}
        self.msg = None
        self.is_writing = False
        self.is_reading = False
        self.is_repeating = False
        time.sleep(2)
        self.add_serials()
        self.add_utilities()

    def read_data(self):
        sc, pr = None, None

        try:
            sc = self.utilities["SC"]
        except KeyError:
            self.is_reading = False

        try:
            pr = self.utilities["PR"]
        except KeyError:
            self.is_reading = False

        if sc or pr:
            self.is_reading = True

        while self.is_reading:
            pr_data, sc_data, serial_data = None, None, None

            try:
                if sc:
                    sc_data = sc.readline()

                if pr:
                    pr_data = pr.readline()

                try:
                    if sc_data and pr_data:
                        serial_data = sc_data.split()[0] + pc_data
                    elif pr_data and not sc_data:
                        serial_data = pr_data
                    elif sc_data and not pr_data:
                        serial_data = sc_data

                    if self.client:
                        try:
                            self.client.send(serial_data)
                        except Exception as e:
                            pass

                except Exception as e:
                    print("13", e)
            except OSError:
                pass
            except serial.SerialException:
                self.is_reading = False

                if sc:
                    sc.close()
                    print("sc closed")

                if pr:
                    pr.close()
                    print("pr closed")
            except Exception:
                pass
        print("reading thread ended!")

    def write_data(self):
        sc, mc, ra = None, None, None

        if "M" in self.utilities:
            mc = self.utilities["M"]

        if "SC" in self.utilities:
            sc = self.utilities["SC"]

        if "R" in self.utilities:
            ra = self.utilities["R"]

        if mc or sc or ra:
            self.is_writing = True

        while self.is_writing:
            self.msg, sc_msg = None, "0,0"

            if self.client:
                try:
                    self.msg = self.client.recv(1024)

                    if self.msg == '' and not self.is_lost:
                        sc.write("lost\n")
                        self.is_lost = True
                        print("lost")
                        self.client.close()
                        self.client = None
                except Exception as e:
                    print(e)

            if self.msg:
                try:
                    self.msg = self.msg.split("/")
                except Exception as e:
                    continue
                    

                try:
                    if mc:
                        mc.write(self.msg[0] + "\n")
                        print("written to mc", self.msg[0])

                    if ra:
                        ra.write(self.msg[1] + "\n")
                except serial.SerialException:
                    break
                except Exception as e:
                    print("14", e)

                if sc:
                    try:
                        sc_msg = self.msg[2].split("\r\n")[0]

                        if sc_msg in ('go', 'delete', 'stop') or \
                           (sc_msg != "0,0" and sc_msg[-1].isdigit()) or \
                           sc_msg[0] == "T":
                            sc.write(sc_msg + '\n')
                            print("written to sc", sc_msg)
      
                    except Exception as e:
                        pass

        print("writing thread ended!")

    def add_serials(self):
        print("adding serials...")
        for port in self.ports:
            try:
                self.serials.append(serial.Serial(port, 115200))
            except Exception as e:
                print("serial error", e)

        if not self.serials:
            print("no serials")

    def add_utilities(self):
        print("adding utilities...")
        serial_no = 0
        num_serials = len(self.serials)
        time.sleep(1)
        current = time.time()

        while serial_no != num_serials:
            try:
                if time.time() - current > 25:
                    break

                utility = self.serials[serial_no]
                utility.timeout = 5
                utility.write("AFAF0000AF8003020000920D0A".decode("hex"))
                time.sleep(0.5)
                if utility.inWaiting():
                    serial_data = utility.readline().split("\r\n")
                    serial_data = serial_data[0].split(",")
                    utility_name = str(serial_data[0])

                    if utility_name in utility_names:
                        print(utility_name + " Identified")

                        self.utilities[utility_name] = utility

                        utility.flushInput()
                        utility.flushOutput()

                        serial_no += 1
                else:
                    serial_no += 1
            except serial.SerialException:
                pass
            except OSError:
                pass
            except Exception:
                pass

    def run_server(self):
        try:
            while True:
                try:
                    self.client, addr = self.server.accept()
                    print('Got connection from', addr)
                    sc = None

                    if not self.is_first:
                        try:
                            sc = self.utilities["SC"]
                        except KeyError:
                            sc = None

                        if sc and self.is_lost:                            
                            try:
                                sc.write("ok\n")
                                self.is_lost = False
                                print("ok")
                            except Exception as e:
                                print("15", e)

                    self.is_first = False
                except Exception as e:
                    print("16", e)
        except Exception as e:
            print("17", e)

    def start_server(self):
        try:
            server_thread = threading.Thread(
                target=self.run_server,
                args=())
            server_thread.daemon = True
            server_thread.start()
        except Exception as e:
            print(e)

        self.is_connected = True

    def start_reading(self):
        try:
            reading_thread = threading.Thread(
                target=self.read_data,
                args=())
            reading_thread.daemon = True
            reading_thread.start()
        except Exception as e:
            print(e)

    def start_writing(self):
        try:
            writing_thread = threading.Thread(
                target=self.write_data,
                args=())
            writing_thread.daemon = True
            writing_thread.start()
        except Exception as e:
            print(e)

    def start_checking(self):
        try:
            check_thread = threading.Thread(
                target=self.check_client,
                args=())
            check_thread.daemon = True
            check_thread.start()
        except Exception as e:
            print(e)

    def check_client(self):
        while True:
            if self.client:
                try:
                    is_data = self.client.send("")
                    if not is_data: 
                        self.client.close()
                        self.client = None

                        try:
                            sc = self.utilities["SC"]
                        except KeyError:
                            sc = None

                        if sc and not self.is_lost:
                            try:
                                sc.write("lost\n")
                                self.is_lost = True
                                print("lost")
                            except Exception as e:
                                print("16", e)

                except Exception as e:
                    sc = None
                    self.client.close()
                    self.client = None

                    try:
                        sc = self.utilities["SC"]
                    except KeyError:
                        sc = None

                    if sc and not self.is_lost:
                        try:
                            sc.write("lost\n")
                            self.is_lost = True
                            print("lost")
                        except Exception as e:
                            print("17", e)

    def run(self):
        print("running")
        is_checking = True

        try:
            self.start_reading()
            self.start_writing()
            #self.start_checking()

            if not self.is_connected:
                self.start_server()

            while is_checking:
                self.current_ports = glob('/dev/ttyACM*') + glob('/dev/ttyUSB*')
                self.change_serial = False

                try:
                    sc = self.utilities["SC"]
                except KeyError:
                    sc = None

                if self.current_ports != self.ports:
                    self.ports = self.current_ports
                    self.change_serial = True

                if not self.change_serial:
                    for utility_name in self.utilities:
                        if not self.utilities[utility_name].isOpen():
                            self.change_serial = True
                            break

                if not self.change_serial:
                    for ser in self.serials:
                        try:
                            time.sleep(1)
                            if ser.inWaiting() and not ser in self.utilities.values():
                                self.change_serial = True
                                break
                        except Exception as e:
                            print("11", e)

                if not self.is_lost and not DEBUG:
                    try:
                        result = os.system("ping -c 1 192.168.1.30")

                        if result != 0:
                            sc.write("lost\n")
                            self.is_lost = True
                    except Exception as e:
                        print(e)

                if self.change_serial:
                    print("changing...")

                    self.change_serial = False
                    self.configure()
                    self.start_reading()
                    self.start_writing()
        except serial.SerialException:
            self.configure()
            self.run()
        except OSError:
            self.configure()
            self.run()
        except Exception as e:
            print("19", e)
            print("Exiting...")
            sys.exit()

