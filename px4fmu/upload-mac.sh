../../px4_bootloader/px_mkfw.py --board_id 5 > prototype.px4
../../px4_bootloader/px_mkfw.py --prototype prototype.px4 --image ~/src/px4_nxbuilder/build/px4fmu/nuttx/nuttx.bin > nuttx.px4
../../px4_bootloader/px_uploader.py nuttx.px4 --port /dev/tty.usbmodemDEM1
