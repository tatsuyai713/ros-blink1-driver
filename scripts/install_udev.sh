#!/bin/bash

if [[ "$(whoami)" != "root" ]]; then
    echo "Please run this as root."
    exit 1
fi


echo "Moving to /etc/udev/rules.d/51-blink1.rules"
cp ./51-blink1.rules /etc/udev/rules.d/51-blink1.rules
echo "Reloading udev"
udevadm control --reload

