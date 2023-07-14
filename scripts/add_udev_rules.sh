#!/usr/bin/env bash

set -e
set -u
set -o pipefail

readonly udev_path="/etc/udev/rules.d/40-rplidar.rules"

echo "writing udev rules to $udev_path"

cat <<EOT | sudo tee $udev_path > /dev/null

# rplidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", KERNELS=="1-1.3", MODE:="0777", SYMLINK+="rplidar"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", KERNELS=="1-1.4", MODE:="0777", SYMLINK+="zigbeebridge"

EOT

sudo udevadm control --reload-rules && sudo udevadm trigger
echo "Done"
