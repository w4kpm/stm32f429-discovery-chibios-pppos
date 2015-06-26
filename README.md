# stm32f429-discovery-chibios-pppos
just a dump of a sample project that connects to pppd - largely based on the one that wizhippo did here on github

The difference is that you should be able to use this with very little changes from a fresh checkout of Chibios

You should only need to :

1. check out Chibios 
2. extract the ext/lwip-1.4.1_patched.7z file
3. Make sure that the path to Chibios is correct in the Makefile - Mine is ../../ChibiOS-RT
4. make
5. ./load.sh (this assumes that you have opeocd installed as well as a usb port hooked up to your STM32F429-DISCOVERY)

I am also going with the understanding that you have all the gcc-arm-eabi stuff installed for building as well

Under UBUNTU I think that the buildessential package and the gcc-arm-none-eabi packages will get you fixed up - 
but don't hold me to it.

Oh yes, . . . 

The main PPP serial link is SD1 connected to PB6 and PB7 - the other serial link that I used for some debugging is SD3 connected to PB10 and PB11 - both are at 460800 for the time being and seem to work well there.




Finally - you need to set up pppd according to the instructions here http://www.dalbert.net/?m=201503
