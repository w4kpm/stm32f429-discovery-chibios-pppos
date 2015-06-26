#! /bin/bash
openocd -f interface/stlink-v2.cfg -f target/stm32f4x_stlink.cfg \
	-c init \
	-c "reset halt; flash probe 0; flash write_image erase build/ch.hex 0; reset run; shutdown"
