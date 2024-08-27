 #!/bin/sh
# Bind the USBCAN device

ip link set can_kvaser type can bitrate 500000   #500kHz
ip link set can_kvaser txqueuelen 1024
ip link set can_kvaser up

ip link set can_candleLight type can bitrate 1000000   #1MHz
ip link set can_candleLight txqueuelen 1024
ip link set can_candleLight up
