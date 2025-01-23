#!/bin/bash

sudo /var/snap/rosbot/common/patch_serial_port.sh
sudo rosbot.stop
sleep 4
sudo rosbot.flash