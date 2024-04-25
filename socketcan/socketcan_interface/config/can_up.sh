 #!/bin/sh
# Bind the USBCAN device

ip link set can0 type can bitrate 500000    #500kHz
ip link set can0 up
ip link set can0 txqueuelen 1000
