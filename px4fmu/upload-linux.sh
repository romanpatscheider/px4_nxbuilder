cd ~/pixhawk/px4_nxbuilder/px4fmu
~/pixhawk/px4_bootloader/px_mkfw.py --board_id 5 > prototype.px4
~/pixhawk/px4_bootloader/px_mkfw.py --prototype prototype.px4 --image ~/pixhawk/px4_nxbuilder/build/px4fmu/nuttx/nuttx.bin > nuttx.px4
~/pixhawk/px4_bootloader/px_uploader.py nuttx.px4 --port /dev/ttyACM0
