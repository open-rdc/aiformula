 #!/bin/sh
# Bind the USBCAN device

ip link set can0 type can bitrate 1000000   #500kHz
ip link set can0 txqueuelen 1024
ip link set can0 up
