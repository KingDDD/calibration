#!/bin/bash

mldb wait-for-device #do
mldb root
mldb remount
mldb rm /data/output.txt
echo -e "while true; do\nsleep 1\ncat /dev/extsync_pin >> /data/output.txt\ndone" > capture.sh
chmod +x capture.sh
mldb push capture.sh /data/
rm capture.sh
mldb shell
