Arduino library for talking to the Cell Top Monitors via the shield for the MEGA.
There is a picture of the shield in the main repository along with a sketch to get you started.

Originally comms was based around a 6 byte datagram, 2 bytes for voltage, 2 bytes for temperature, and 2 bytes for forward error correction. Since the CTMs can only transmit 255 bytes this limited to maximum number of cells to 41.
The revised v2 comms, separates the voltage and temperature data into 3 byte datagrams (the master would normally request voltage data and every so often request temperature data.) This allows the maximum number of cells to increase to 84.
