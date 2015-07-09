2015-7-8
I uploaded a video of the older code here:
https://www.youtube.com/watch?v=EpdJi0NMHLo
I plan on uploading something demonstrating the newer code later.

* Copy Block still not working 

* Raw tile not working 

2015-7-6
update:

* This project is being turned into a VNC client.

* The serial links have changed - 

* The logging is now on PA0(tx) and PA1(rx)

* the PPP is now on PA9(tx) and PA10(rx)

* Important - the VNC server must not have a password, and it only goes and looks at the default 172.30.1.98:5901

* Updating is *REALLY* Slow - As in barely functional at 460800 - I suspect a delay in the lwip implementation (I'm getting 1.2Kbytes / sec - I think I should be getting more like 46Kbytes/sec at that speed). I am going to try raw sockets and see how it works.

* Raw (vnc) updates are not currently doing anything. 

* Code is a mess - don't expect that to change too soon - I'm more worried about function. 



# stm32f429-discovery-chibios-pppos
just a dump of a sample project that connects to pppd - largely based on the one that wizhippo did here on github

The difference is that you should be able to use this with very little changes from a fresh checkout of Chibios

You should only need to :

1. check out Chibios 
2. extract the ext/lwip-1.4.1_patched.7z file
3. Make sure that the path to Chibios is correct in the Makefile - Mine is ../../ChibiOS-RT
4. make
5. ./load.sh (this assumes that you have opeocd installed as well as a usb port hooked up to your STM32F429-DISCOVERY - yes this will remove the software that came with the device. I didn't back it up so I don't know how to do that off the top of my head. - be careful if you want to keep it)


I am also going with the understanding that you have all the gcc-arm-eabi stuff installed for building as well

Under UBUNTU I think that the buildessential package and the gcc-arm-none-eabi packages will get you fixed up - 
but don't hold me to it.

Oh yes, . . . 

The main PPP serial link is SD1 connected to PB6 and PB7 - the other serial link that I used for some debugging is SD3 connected to PB10 and PB11 - both are at 460800 for the time being and seem to work well there.




Finally - you need to set up pppd according to the instructions here http://www.dalbert.net/?m=201503
