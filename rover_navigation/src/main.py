#!/usr/bin/env python
# -*- coding: utf-8 -*-

from serial_com import SerialNode
from settings import *


def main():
    serial_node = SerialNode()
    serial_node.run()

if __name__ == "__main__":
    main()
