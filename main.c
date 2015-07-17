/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "chprintf.h"
#include "shell.h"
#include "sysinfo.h"
#include "sdram.h"

#include "lwip/tcpip.h"
#include <string.h>
#include "ppp/ppp.h"
#include "stm32f4xx.h"
#include "miniz.c"
#include "vnc-logo.h"

mutex_t SD4mtx;
//#include "lwipthread.h"

#define TFT_COPY_BUFFER_START 0xD0200000
#define TFT_BUFFER_START 0xD0000000
#define TFT_BUFFER_WIDTH 800
#define TFT_BUFFER_HEIGHT 480
#define TFT_PIXEL_SIZE 2

#define XFER_BUFFER 0xD0300000


#define PKT_BUFFER_START 0xD0100000
#define PKT_BUFFER_LEN   0x00100000
uint8_t *bufferstart;
uint8_t *bufferend;
uint16_t numbytes;
z_stream infstream;

static uint8_t txbuf[2];
static uint8_t rxbuf[2];
static char text[255];
struct tcp_pcb *gpcb;
int tcp_connected;
int tcp_sent_data;



/*
 * Endpoints to be used for USBD2.
 */
#define USBD2_DATA_REQUEST_EP           1
#define USBD2_DATA_AVAILABLE_EP         1
#define USBD2_INTERRUPT_REQUEST_EP      2

/*
 * Serial over USB Driver structure.
 */

static SerialUSBDriver SDU2;

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x5740,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD2_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD2_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD2_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(38),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  0x0040,
  0x0040,
  &ep1instate,
  &ep1outstate,
  2,
  NULL
};

/**
 * @brief   IN EP2 state.
 */
static USBInEndpointState ep2instate;

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  &ep2instate,
  NULL,
  1,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();

    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, USBD2_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USBD2_INTERRUPT_REQUEST_EP, &ep2config);

    /* Resetting the state of the CDC subsystem.*/
    sduConfigureHookI(&SDU2);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * USB driver configuration.
 */
static const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  sduRequestsHook,
  NULL
};

/*
 * Serial over USB driver configuration.
 */
static const SerialUSBConfig serusbcfg = {
  &USBD2,
  USBD2_DATA_REQUEST_EP,
  USBD2_DATA_AVAILABLE_EP,
  USBD2_INTERRUPT_REQUEST_EP
};






static const SPIConfig std_spicfg1 = {
  NULL,
  GPIOC,                                                        /*port of CS  */
  2,                                                /*pin of CS   */
  SPI_CR1_CPOL|SPI_CR1_CPHA| \
  SPI_CR1_SPE|SPI_CR1_MSTR|SPI_CR1_BIDIMODE ,
  NULL
};

/*
The interface to the LCD controller appears to be bidirectional - and you have to do some stuff to set it up before it will take parallel info from the lcd controller from STM32 - it may be best to work on connecting to VNC server and speed issues before messing with the screwy SPI stuff with this display.
*/


void init_spi()
{
//  palSetPadMode(GPIOA, CS, PAL_MODE_OUTPUT_PUSHPULL);
    // palSetPadMode(GPIOA, CS2, PAL_MODE_OUTPUT_PUSHPULL);
    // palSetPadMode(GPIOB, CK, PAL_MODE_OUTPUT_PUSHPULL);
    // palSetPadMode(GPIOB, MISO, PAL_MODE_INPUT_PULLDOWN);
    //palSetPadMode(GPIOB, MOSI, PAL_MODE_OUTPUT_PUSHPULL);
    //palSetPad(GPIOA,CS);
    //palSetPad(GPIOA,CS2);
    // palClearPad(GPIOB,CK);

}
uint8_t spi_read(device,location)
{
  spiStart(&SPID5,device);
  spiSelect(&SPID5);
  txbuf[0] = 0x80 | location;
  spiSend(&SPID5,1,&txbuf);
  spiReceive(&SPID5,1,&rxbuf);
  spiUnselect(&SPID5);
  spiStop(&SPID5);
  return(rxbuf[0]);
}
void spi_write(device,location,data)
{
  spiStart(&SPID5,device);
  spiSelect(&SPID5);
  txbuf[0] = 0x00 | location;
  txbuf[1] = data;
  spiSend(&SPID5,2,&txbuf);
  spiUnselect(&SPID5);
  spiStop(&SPID5);
}






void TFTLCD_Init(void);
void SDRAM_Init(void);
/* stolen from a version of sys_arch I found on wizhippo's github 

https://github.com/wizhippo/stm32f4-chibios-lwip-pppos.git
*/
u32_t sys_jiffies(void) {
	static u32_t jiffies = 0;
#if OSAL_ST_FREQUENCY == 1000
	jiffies += (u32_t)osalOsGetSystemTimeX();
#elif (OSAL_ST_FREQUENCY / 1000) >= 1 && (OSAL_ST_FREQUENCY % 1000) == 0
	jiffies += ((u32_t)osalOsGetSystemTimeX() - 1) / (OSAL_ST_FREQUENCY / 1000) + 1;
#elif (1000 / OSAL_ST_FREQUENCY) >= 1 && (1000 % OSAL_ST_FREQUENCY) == 0
	jiffies += ((u32_t)osalOsGetSystemTimeX() - 1) * (1000 / OSAL_ST_FREQUENCY) + 1;
#else
	jiffies += (u32_t)(((u64_t)(osalOsGetSystemTimeX() - 1) * 1000) / OSAL_ST_FREQUENCY) + 1;
#endif

	return jiffies;
}



static SerialConfig uartCfg =
{
    460800,// bit rate
    //460800,// bit rate
    0,
    0,
    0,
};


static SerialConfig uartCfg2 =
{
    460800,// bit rate
    0,
    0,
    0,
};


/*
 * Application entry point.
 */


static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  int pass = 0;
  chRegSetThreadName("Thread1");

  while (TRUE) {
    palSetPad(GPIOG, 13);       /* Green.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOG, 13);     /* Green.  */
    chThdSleepMilliseconds(500);
  }

	return MSG_OK;
}


#ifndef SOCK_TARGET_HOST
#define SOCK_TARGET_HOST  "172.30.1.98"
#endif

#ifndef SOCK_TARGET_PORT
#define SOCK_TARGET_PORT  5901
#endif



/*
 * Shell related
 */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

static const ShellCommand shell_commands[] = {
	SYSINFO_SHELL_COMMANDS,
	{NULL, NULL}
};

uint8_t  *vncbuffer;
int vncsocket;

void log_data(char* text)
{

    chprintf((BaseSequentialStream*)&SD4,text);
    chprintf((BaseSequentialStream*)&SDU2,text);
}

void log_data_num(char* text,int data)
{

    chprintf((BaseSequentialStream*)&SD4,text,data);
    chprintf((BaseSequentialStream*)&SDU2,text,data);
}


void ensure_bytes(uint16_t numb)
{
    // I had problems with recv not returning the number of bytes that 
    // I requested - I am assuming that recv also returns when it hits the 
    // end of a packet, not neccecerily when it gets the number of bytes requested.
    // In order to get the required number of bytes, I loop until I have read everything
    // I asked for.
    int currentbytes;
    uint16_t readbytes,neededbytes;
    uint8_t *buffpos;

    readbytes =0;
    neededbytes = numb;
    buffpos = vncbuffer;
    while (readbytes < numb)
	{
	    //chThdSleepMilliseconds(10);
	    //log_data_num("we need %d \r\n",neededbytes);
	    //log_data_num("we have %d \r\n",readbytes);
	    currentbytes=rx((uint8_t *)buffpos+readbytes,neededbytes);
	    if (currentbytes < 0)
		{
		    log_data_num("we got an error reading %d \r\n",currentbytes);
		    return;
		}
	    //log_data_num("we read %d \r\n",currentbytes);
	    readbytes += currentbytes;
	    neededbytes -= currentbytes;
	}	
}










void set_16(uint8_t *buffer, uint16_t data)
{    
    // Endianess differs - we need to swap the bytes

    int x;
    for (x=0;x<2;x++)
	*(buffer+x) = 0xFF & (data >> 8*(1-x));
}

uint8_t read_8(void)
{    
    // Endianess differs - we need to swap the bytes


    uint8_t data;
    int numbytes;
    data = 0;
    numbytes=rx(&data,1);
    if (numbytes != 1)
	log_data_num("read_8 Incorrect length\r\n",numbytes);
    return data;
}



uint16_t read_16(void)
{    
    // Endianess differs - we need to swap the bytes


    uint16_t data;
    int numbytes;
    ensure_bytes(2);
    data = data = vncbuffer[1];
    data = data | vncbuffer[0] << 8;
    return data;
}



uint16_t read_pixel(void)
{    
    // Endianess differs - we need to swap the bytes


    uint16_t data;
    int numbytes;
    ensure_bytes(2);
    data = *(uint16_t *) vncbuffer;
    return data;
}



uint32_t read_32(void)
{    
    // Endianess differs - we need to swap the bytes


    uint32_t data;
    int numbytes;
    data = 0;
    ensure_bytes(4);
    data = data | vncbuffer[3];
    data = data | vncbuffer[2] << 8;
    data = data | vncbuffer[1] << 16;
    data = data | vncbuffer[0] << 24;
    return data;
}



void set_32(uint8_t *buffer, uint32_t data)
{    
    // Endianess differs - we need to swap the bytes
    int x;
    for (x=0;x<4;x++)
	*(buffer+x) = 0xFF & (data >> 8*(3-x));
}



uint32_t translate_color(uint32_t color)
{
    // color as delivered is backwards from what we expect - 
    // so we translate - hopefully I can change this sometime
    uint32_t newcolor;
    newcolor = (color &  0x0000f800) ;
    newcolor |= ((color & 0x00fc0000) >> 13);
    newcolor |= ((color & 0xf8000000) >> 27);
    return newcolor;
}




void read_all_data(char* str,uint16_t len)
{
    /* reads all bytes left in queue - NB - it looks like lwip breaks 
       at the end of each remote transition, so even though this completes, 
       there may be another packet waiting */
    int x;
    int numread;
    

    if (numbytes < len)
	{
	  log_data_num("only %d bytes - will block \r\n",numbytes);
	    return;
	}
    numread=rx(vncbuffer,len);
    log_data_num("%s Data Dump:\r\n    ",str);
    for (x=0;x<numread;x++)
      log_data_num("0x%X ",vncbuffer[x]);

    log_data("\r\n");


}



void read_copy_rect(int xpos, int ypos, int width, int height)
{
    uint16_t sourcex,sourcey;
    sourcex = read_16();
    sourcey = read_16();
    //    sprintf(text,"copy-rect. source %d,%d dest: %d,%d size:%d,%d\r\n",sourcex,sourcey,xpos,ypos,height,width);
    //log_data(text);

    // we copy twice - once to a copy buffer - then from the copy buffer back to the live buffer
    // this is because it was causing problems with the DMA - it would copy to portions that 
    // still had not been copied yet.  - it's actually pretty fast.

    DMA2D->CR = 0; // mode =  memory to memory
    DMA2D->OPFCCR = 0x2; // set rgb565 mode
    DMA2D->FGPFCCR = 0x2; // set rgb565 mode
  
    DMA2D->OMAR = TFT_COPY_BUFFER_START + (TFT_BUFFER_WIDTH * sourcey * TFT_PIXEL_SIZE) + sourcex* TFT_PIXEL_SIZE; // memory start 
    DMA2D->OOR = TFT_BUFFER_WIDTH - width;  // output offset (determines offset of next line)

    DMA2D->FGMAR = TFT_BUFFER_START + (TFT_BUFFER_WIDTH * sourcey *TFT_PIXEL_SIZE) + sourcex*TFT_PIXEL_SIZE; // memory start 
    DMA2D->FGOR = TFT_BUFFER_WIDTH - width;  // output offset (determines offset of next line)


    DMA2D->NLR = (uint32_t) ((width << 16) | height); //width and height 
    DMA2D->CR |= 1;  // start operation
    while (DMA2D->CR & DMA2D_CR_START) {
    }


    DMA2D->CR = 0; // mode =  memory to memory
    DMA2D->OPFCCR = 0x2; // set rgb565 mode
    DMA2D->FGPFCCR = 0x2; // set rgb565 mode
  
    DMA2D->OMAR = TFT_BUFFER_START + (TFT_BUFFER_WIDTH * ypos * TFT_PIXEL_SIZE) + xpos* TFT_PIXEL_SIZE; // memory start 
    DMA2D->OOR = TFT_BUFFER_WIDTH - width;  // output offset (determines offset of next line)

    DMA2D->FGMAR = TFT_COPY_BUFFER_START + (TFT_BUFFER_WIDTH * sourcey *TFT_PIXEL_SIZE) + sourcex*TFT_PIXEL_SIZE; // memory start 
    DMA2D->FGOR = TFT_BUFFER_WIDTH - width;  // output offset (determines offset of next line)


    DMA2D->NLR = (uint32_t) ((width << 16) | height); //width and height 
    DMA2D->CR |= 1;  // start operation
    while (DMA2D->CR & DMA2D_CR_START) {
    }


}


void read_raw_rect(int x,int y, int w, int h)
{
    // this just converts each incoming byte and 
    // directly updates the framebuffer.
    int bytecount;

    //    uint32_t incoming_pixel;
    uint16_t pixel;
    int q,r;
    //log_data_num("Raw Rect: reading %d bytes\r\n",(w * h * 4));
    ensure_bytes((w * h * 2));
    for (q = 0; q<w; q++)
	for(r=0;r<h; r++)
	    {
	      
	      pixel = *(uint16_t*)(vncbuffer+((r*w+q)*2));
	      //pixel=translate_color(incoming_pixel);
	      *(uint16_t*) (TFT_BUFFER_START + (((y+r)*TFT_BUFFER_WIDTH)+x+q)*TFT_PIXEL_SIZE) = pixel &0xffff;
	    }
	
}



void read_zlib_rect(int x,int y, int w, int h)
{
    // WARNING - this doesn't really work right -
    // in the case that your framebuffer is 800 bytes and 
    // you are just updating 600 bytes, it doesn't skip the stuff that isn't
    // updated resulting in a skewed image. 

    // to work right, it needs to do something like what the raw rect 
    // above does. If it were re-written a bit the raw code above could work
    // for both 

    // Oh - and the vnc logo at the start will probably goof up the
    // zlib stream, so it probably needs to be re-started prior to 
    // connecting to vnc

    
    // this just converts each incoming byte and 
    // directly updates the framebuffer.
    int bytecount;

    //    uint32_t incoming_pixel;
    int16_t pixel;
    uint32_t pixelcount;
    uLong uncompress_size;
    int q,r;
    
    pixelcount = read_32();
    sprintf(text,"zlib Rect: (%d,%d,%d,%d) reading %d bytes\r\n",x,y,w,h,pixelcount);
    log_data(text);
    ensure_bytes(pixelcount);

    infstream.avail_in = pixelcount; // size of input
    infstream.next_in = vncbuffer; // input char array
    infstream.avail_out = w*h*TFT_PIXEL_SIZE;
    infstream.next_out = TFT_BUFFER_START + ((y*TFT_BUFFER_WIDTH)+x)*TFT_PIXEL_SIZE;


    log_data("got bytes\r\n");
    pixel = inflate(&infstream,Z_NO_FLUSH);
    log_data_num("after uncompress %d\r\n",pixel);
	
}





uint8_t clipit (int currentpos, int maxpos)
{
    int currentx;
    currentx = currentpos*16;
    if ((currentx + 16) < maxpos)
	return 16;
    else
	return (16 - ( (currentx + 16) - maxpos));	    
}

// these may need to be kept between runs 
// the protocol only sends them once and then reuses them.
// I'm not sure if this is true between frame requeses,
// I made them global just in case.
uint32_t hextile_bg_color;
uint32_t hextile_fg_color;


void fill_rect(uint32_t color, int x, int y, int width, int height,int xlate) 
{
    DMA2D->CR = 3 << 16; // mode =  register to memory
    DMA2D->OPFCCR = 0x2; // set rgb565 mode
    if (xlate ==0)
	DMA2D->OCOLR = color;
    else
	DMA2D->OCOLR = translate_color(color);
  
    DMA2D->OMAR = TFT_BUFFER_START + (TFT_BUFFER_WIDTH * y *TFT_PIXEL_SIZE) + x*TFT_PIXEL_SIZE; // memory start 
    DMA2D->OOR = TFT_BUFFER_WIDTH - width;  // output offset (determines offset of next line)
    DMA2D->NLR = (uint32_t) ((width << 16) | height); //width and height 
    DMA2D->CR |= 1;  // start operation
    while (DMA2D->CR & DMA2D_CR_START) {
    }

}


void process_colored_rects(int rectCount, int startX, int startY)
{
    int count;
    uint32_t currentcolor;
    uint8_t xy,wh;
    uint8_t x,y,h,w;
    //log_data_num("start colored rects %d\r\n",rectCount);
    for (count = 0;count < rectCount; count++)
	{
	    //	    log_data_num("colored Rect #%d\r\n",count);
	    currentcolor = read_pixel();
	    xy = read_8();
	    wh = read_8();
	    y = xy & 0xf;
	    x = (xy & 0xf0) >> 4;
	    h = wh & 0xf;
	    w = (wh & 0xf0) >> 4;
	    fill_rect(currentcolor,startX + x,startY+y,w+1,h+1,0);
	}
}
	    
void process_foreground_rects(uint32_t fgcolor,int rectCount, int startX, int startY)
{
    int count;

    uint8_t xy,wh;
    uint8_t x,y,h,w;
    //log_data_num("start foreground rects %d\r\n",rectCount);
    for (count = 0;count < rectCount; count++)
	{
	    //	    log_data_num("foreground Rect #%d\r\n",count);
	    xy = read_8();
	    wh = read_8();
	    y = xy & 0xf;
	    x = (xy & 0xf0) >> 4;
	    h = wh & 0xf;
	    w = (wh & 0xf0) >> 4;
	    fill_rect(fgcolor,startX + x,startY+y,w+1,h+1,0);
	}
}
	    



void read_hextile_rect(int x,int y, int width, int height)
{
    int y2,x2;

    uint8_t subencoding,raw,background,foreground,anysubrects,coloredrects;
    int newwidth,newheight;
    int subrect_count;
    for (y2=0;y2<height/16.0;y2++)
	for(x2=0;x2<width/16.0;x2++)
	    {
		//chThdSleepMilliseconds(10);
		subencoding = read_8();
		raw = subencoding & 1;
		background = subencoding & 2;
		foreground = subencoding & 4;
		anysubrects = subencoding & 8;
		coloredrects = subencoding & 16;
		//sprintf(text,"subencoding %X raw:%x bg:%x fg:%x asr:%x cr:%x\r\n",subencoding,
		//	raw,background,foreground,anysubrects,coloredrects);
		//log_data(text);
		newwidth = clipit(x2,width);
		newheight = clipit(y2,height);
		if (raw)
		    {
			//sprintf(text,"starting raw rect pos: %d,%d size:%d,%d\r\n",x+16*x2,y+16*y2,newwidth,newheight);
			//log_data(text);
			read_raw_rect(x+16*x2,y+16*y2,newwidth,newheight);
		    }
		else
		    {
			if (background)
			    {
				
				hextile_bg_color = read_pixel();
				//log_data_num("new bg color %X\r\n",hextile_bg_color);			    
			    }
			if (foreground)
			    {
				hextile_fg_color = read_pixel();
				//log_data_num("new fg color %X\r\n",hextile_fg_color);			    }
			    }
			if (anysubrects)
			    {
				subrect_count = read_8();
				//log_data_num("subrec Count %d\r\n",subrect_count);
			    }
			// fill bg rect here 
		        fill_rect(hextile_bg_color,x2*16+x,y2*16+y,newwidth,newheight,0);
			//  I think these are mutually exclusive.
			if (coloredrects)
			    process_colored_rects(subrect_count,x2*16+x,y2*16+y);
			else
			    if (anysubrects)
				process_foreground_rects(hextile_fg_color,subrect_count,x2*16+x,y2*16+y);
		    }
		
		
		
	    }
	    
}



//int startlog;

void read_fb_rect(void)
{
    uint16_t xpos,ypos,height,width;
    uint32_t encoding;

    xpos = read_16();
    ypos = read_16();
    width = read_16();
    height = read_16();
    encoding = read_32();
    if (encoding == 6)
	{
	    sprintf(text,"New Rect %d %d %d %d %d\r\n",xpos,ypos,height,width,encoding);
	    log_data(text);
	    }
    switch (encoding)
	{
	case 0: 
	    read_raw_rect(xpos,ypos,width,height);
	    break;
	case 1:
	    read_copy_rect(xpos,ypos,width,height);
	    break;
        case 5:
	    read_hextile_rect(xpos,ypos,width,height);
	    break;
        case 6:
	    read_zlib_rect(xpos,ypos,width,height);
	    break;

	default:
	    log_data_num("Got unexpected encoding %d\r\n",encoding);
	}

	    
	
}



void process_frame_response()
{
    uint8_t message,padding;
    int x;
    uint16_t numrects;
    message = read_8();
    if (message == 0)
	{
	    padding = read_8();
	    numrects = read_16();
	    //log_data_num("number of high level rects %d\r\n",numrects);
	    for (x=0;x<numrects;x++)
		{
		    //	    log_data_num("high level rect # %d\r\n",x);
		    read_fb_rect();
		    //chThdSleepMilliseconds(10);
		}
	}
    else
	{
	    log_data_num("Unexpected message (%d) -- (I was expecting frame response, but got something else):\r\n",message);
	}
    
    //    read_all_data("Doing process_frame_response");
    //lwip_close(vncsocket);
}

void read_pixel_format_data(void)
{
    uint32_t strLength;
    log_data_num("width: %d\r\n",read_16()); 
    log_data_num("height: %d\r\n",read_16()); 
    log_data_num("bpp: %d\r\n",read_8()); 
    log_data_num("depth: %d\r\n",read_8()); 
    log_data_num("big-endian: %d\r\n",read_8()); 
    log_data_num("true-color: %d\r\n",read_8()); 
    log_data_num("red-max: %d\r\n",read_16()); 
    log_data_num("green-max: %d\r\n",read_16()); 
    log_data_num("blue-max: %d\r\n",read_16()); 
    log_data_num("red-shift: %d\r\n",read_8()); 
    log_data_num("green-shift: %d\r\n",read_8()); 
    log_data_num("blue-shift: %d\r\n",read_8());
    rx(vncbuffer,3); // per the protocol, there is 
                                         // three bytes of padding
    strLength= read_32();
    ensure_bytes(strLength);
    vncbuffer[strLength] = 0;
    log_data("name:\r\n     ");
    log_data(vncbuffer);
    log_data("\r\n");

}



void set_pixel_format_data(void)
{
  int len;
  vncbuffer[0] = 0;
  vncbuffer[4] = 16; // 16 bpp
  vncbuffer[5] = 16; // 16 bit depth
  vncbuffer[6] = 0; //little endian
  vncbuffer[7] = 1; // true color
  set_16(vncbuffer+8,0x1f); // set red to be 5 bits
  set_16(vncbuffer+10,0x3f); // set g to be 6 bits
  set_16(vncbuffer+12,0x1f); // set blue to be 5 bits
  vncbuffer[14] = 11; // shift red 11
  vncbuffer[15] = 5;  // shift green 5
  vncbuffer[16] = 0;  // shift blue 0
  len = send(vncbuffer,20);
  if (len != 20)
	log_data("pixel information Incorrect length\r\n");

}




void framebuffer_request(uint16_t x, 
			 uint16_t y, 
			 uint16_t width, 
			 uint16_t height, 
			 uint8_t incremental)
{
    int len;
    uint16_t *midptr;
    uint32_t *longptr;

    //    read_all_data("Begin FB Request");
    
    vncbuffer[0] = 3; // message type
    vncbuffer[1] = incremental;
    set_16(vncbuffer+2,x);
    set_16(vncbuffer+4,y);
    set_16(vncbuffer+6,width);
    set_16(vncbuffer+8,height);

    len = send(vncbuffer,10);
    if (len != 10)
	log_data("framebuffer Incorrect length\r\n");
    process_frame_response();
}



void set_encodings()
{
    int x;
    
    vncbuffer[0] = 2; // message
    vncbuffer[1] = 0; // padding
    set_16(vncbuffer+2,4); // we're using 4 encodings
    set_32(vncbuffer+4,5); // hextile
    set_32(vncbuffer+8,6); // zlib
    set_32(vncbuffer+12,1); // copyrect
    set_32(vncbuffer+16,0); // raw

    // - only to print out raw bytes and make sure things were 
    // in correct order.

    //chprintf((BaseSequentialStream*)&SD4,"Encoding:\r\n    ");
    //for (x=0;x<16;x++)
    //	chprintf((BaseSequentialStream*)&SD4,"0x%X ",vncbuffer[x]);
    //chprintf((BaseSequentialStream*)&SD4,"\r\n");
    x = send(vncbuffer,20);
    if (x != 20)
	log_data("set Encodings Incorrect length\r\n");
	

}

char tcpbuff[255];


uint8_t *tcpipbuffer;



int send(uint8_t *buffer,int len)
{
    err_t err;
    tcp_sent_data = 0;
    err = tcp_write(gpcb,buffer,len,0);
    if (err == ERR_OK)
	err=0;
	//chprintf((BaseSequentialStream*)&SD4,"tcp_write= OK\r\n");
    else
      log_data_num("tcp_write= %d\r\n",err);
    while (tcp_sent_data ==0)
	{
	    //	    chprintf((BaseSequentialStream*)&SD4,"o");
	    chThdSleepMilliseconds(10);

	    
	}
    return len;
}

int rx(uint8_t* buffer,int len)
{
    int retval;
    retval = len;
    // block forever
    
    while (numbytes < len)
	{
	  log_data("x");

	    chThdSleepMilliseconds(10);
	}

    memcpy(buffer,bufferstart,len);
    bufferstart += len;
    numbytes -= len;
    if (bufferstart == bufferend)
	{
	    bufferstart = bufferend = PKT_BUFFER_START;
	}
    return retval;
    
}


int connect_vnc(void){

    err_t err;
    read_all_data("Server Response",12);
    sprintf(tcpbuff,"RFB 003.003\n"); // send the version we would like to get 
    send(tcpbuff,12);
    tcpbuff[0] = 1;
    send(tcpbuff,1); // share flag - allow sharing
    read_all_data("Security Code",4);    
    read_pixel_format_data();    
    set_pixel_format_data();
    set_encodings();



}

err_t tcp_recv_callback(void* arg, struct tcp_pcb *tpcb,struct pbuf *p,err_t err)
 {
     int x;
     struct pbuf *initp;
     int total_rx = 0;
     initp = p;
     while (p != NULL)
	 {
	     //chprintf((BaseSequentialStream*)&SD4,"pbuf(%x) RX %d (total = %d) bytes - next Pbuf = %X bufferend =%X numbytes =%d \r\n",p,p->len,p->tot_len,p->next,bufferend,numbytes);
	     memcpy(bufferend,p->payload,p->len);
	     bufferend += p->len;
	     numbytes += p->len;
	     total_rx += p->len;
	     p = p->next;	  
	 }
     pbuf_free(initp);
     tcp_recved(gpcb,total_rx);
     return ERR_OK;
 }



 err_t tcp_connect_callback(void* arg, struct tcp_pcb *tpcb,err_t err)
 {
   log_data("got to callback \r\n");
     if (err == ERR_OK)
	 {
	     tcp_connected = 1;
	     log_data("Connected!\r\n");

	 }
     else
	 {
	   log_data_num("connection error: %d\r\n",err);

	 }
     return ERR_OK;

}

err_t tcp_sent_callback(void *arg,
		       struct tcp_pcb *tpcb,
		       u16_t len)
{
    //chprintf((BaseSequentialStream*)&SD4,"sent: %d bytes\r\n",len);  
    tcp_sent_data = 1;
    return ERR_OK;
}




static THD_WORKING_AREA(waVncThread, 4096 );
static THD_FUNCTION(VncThread, arg) {

	int ret;
	int pass;
	int x;
	struct ip_addr dest;
	err_t error;

	tcp_connected = 0;
	bufferstart = PKT_BUFFER_START;
	bufferend = PKT_BUFFER_START;
	numbytes = 0;

	IP4_ADDR(&dest, 172, 30, 1, 98);
	//	struct sockaddr_in sa;

	int recsize;
	//	socklen_t cli_addr_len;
	//	struct sockaddr_in cli_addr;

	//	socklen_t fromlen;

	chRegSetThreadName("VncThread");
	log_data("VNC Client Starting \r\n");


	gpcb = tcp_new();	
	tcp_sent(gpcb,tcp_sent_callback);
	tcp_recv(gpcb,tcp_recv_callback);
	error = tcp_connect(gpcb,&dest,SOCK_TARGET_PORT,tcp_connect_callback);

	if (error != ERR_OK)
	  log_data_num("error colling connect %d \r\n",error);
	else
	  log_data("tcp_connect OK \r\n");

	while (tcp_connected == 0)
	    {
	      log_data("-");

		chThdSleepMilliseconds(100);
		if (chThdShouldTerminateX())
		  return TRUE;
	    }
	    

	connect_vnc();
	framebuffer_request(0,0,800,480,0);
	while (!chThdShouldTerminateX())
	    {
	      //log_data("****************************************\r\n");
//		//read_all_data("getting ready to read");
		//chprintf((BaseSequentialStream*)&SD4,".");
		//chThdSleepMilliseconds(1000);
		framebuffer_request(0,0,800,480,1);

	    }
	return TRUE;
}

/*
static THD_WORKING_AREA(waEchoServerThread, 2048 );
static THD_FUNCTION(EchoServerThread, arg) {
	(void) arg;
	int pass = 0;
  
	int sock,newsockfd;
	struct sockaddr_in sa;
	char buffer[1024];
	int recsize;
	socklen_t cli_addr_len;
	struct sockaddr_in cli_addr;

	socklen_t fromlen;

	chRegSetThreadName("EchoServerThread");
	chprintf((BaseSequentialStream*)&SD4,"Shell Server Starting \r\n");

	sock = lwip_socket(AF_INET,  SOCK_STREAM, IPPROTO_TCP);
	if (sock == -1) {
	    chprintf((BaseSequentialStream*)&SD4,"Closing A - no socket \r\n");

		return MSG_RESET;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	sa.sin_port = PP_HTONS(26);
	fromlen = sizeof(sa);

	if (lwip_bind(sock, (struct sockaddr *) &sa, sizeof(sa)) == -1) {
	    chprintf((BaseSequentialStream*)&SD4,"Closing B - no BIND \r\n");
		lwip_close(sock);
		return MSG_RESET;
	}
	if (lwip_listen(sock, 1) < 0) {
		lwip_close(sock);
		return MSG_RESET;
	}

	while (!chThdShouldTerminateX()) {
	    chprintf((BaseSequentialStream*)&SD4,"Attempting to rx\r\n");
	    newsockfd = lwip_accept(sock, (struct sockaddr *) &cli_addr,
				    &cli_addr_len);
	    if (newsockfd < 0) {
		break;
	    }
	    chprintf((BaseSequentialStream*)&SD4,"Accepted\r\n");
	    chThdSleepMilliseconds(100);
	    while (!chThdShouldTerminateX()) {

		recsize = lwip_recvfrom(newsockfd, buffer, sizeof(buffer), 0,
				(struct sockaddr *) &sa, &fromlen);
		chprintf((BaseSequentialStream*)&SD4,"recsize %d - fromlen %d\r\n",recsize,fromlen);
		if (recsize < 0) {
			break;
		}
		pass ++;
		if (pass%2 ==0)
		    {
			palClearPad(GPIOG, 14);
		    }
		else
		    {
			palSetPad(GPIOG, 14);
		    }

		lwip_sendto(newsockfd, (void *) buffer, recsize, 0, (struct sockaddr *) &sa,
				fromlen);
	    }
	    lwip_close(sock);
	}

	lwip_close(sock);

	return MSG_OK;
}

*/
/*
 * TCP Shell server thread
 */
/*
static THD_WORKING_AREA(waShellServerThread, 512);
static THD_FUNCTION(ShellServerThread, arg) {
	(void) arg;

	int sockfd, newsockfd;
	socklen_t cli_addr_len;
	struct sockaddr_in serv_addr, cli_addr;

	//	SocketStream sbp;
	ShellConfig shell_cfg;
	thread_t *shelltp;

	chRegSetThreadName("ShellServerThread");
	chprintf((BaseSequentialStream*)&SD4,"Shell Server Starting \r\n");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = PP_HTONS(25);

	sockfd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sockfd < 0)
		return MSG_RESET;

	if (lwip_bind(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		lwip_close(sockfd);
		return MSG_RESET;
	}

	if (lwip_listen(sockfd, 1) < 0) {
		lwip_close(sockfd);
		return MSG_RESET;
	}

	cli_addr_len = sizeof(cli_addr);

	while (!chThdShouldTerminateX()) {
		newsockfd = lwip_accept(sockfd, (struct sockaddr *) &cli_addr,
				&cli_addr_len);
		if (newsockfd < 0) {
			break;
		}

		ssObjectInit(&sbp, newsockfd);
		shell_cfg.sc_channel = (BaseSequentialStream*) &sbp;
		shell_cfg.sc_commands = shell_commands;

		shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO+2);
		chThdWait(shelltp);

		lwip_close(newsockfd);
	}

	if (lwip_shutdown(sockfd, SHUT_RDWR) < 0) {
		// oops
	}
	lwip_close(sockfd);

	return MSG_OK;
}

*/
static THD_WORKING_AREA(waShellServerThread2, 512);
static THD_FUNCTION(ShellServerThread2, arg) {
	(void) arg;


	ShellConfig shell_cfg;
	thread_t *shelltp;

	chRegSetThreadName("ShellServerThread2");

	shell_cfg.sc_channel = (BaseSequentialStream*) &SD4;
	shell_cfg.sc_commands = shell_commands;

	shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		chThdWait(shelltp);


	return MSG_OK;
}

/*
 * Handle link status events
 */
static void ppp_linkstatus_callback(void *ctx, int errCode, void *arg) {
	(void) arg;

	volatile int *connected = (int*)ctx;
	chprintf((BaseSequentialStream*)&SD4,"callback %x\r\n",errCode);
	chThdSleepMilliseconds(100);

	if (errCode == PPPERR_NONE) {
		*connected = 1;
	} else {
		*connected = 0;
	}
}





uint32_t Current_color;




int main(void) {
    //  unsigned i;
  thread_t *echoServerThread = 0;
  thread_t *shellServerThread = 0;
  thread_t *shellServerThread2 = 0;
  //  startlog = 0;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  uint32_t i;
  halInit();
  chSysInit();


  infstream.zalloc = Z_NULL;
  infstream.zfree = Z_NULL;
  infstream.opaque = Z_NULL;
  inflateInit(&infstream);

  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(8));    
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(8));
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
      RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
      RCC_AHB1ENR_GPIOGEN;

  sdStart(&SD1, &uartCfg);
  sdStart(&SD4, &uartCfg2);


  SDRAM_Init();
  TFTLCD_Init();





  //for(i = 0xD0000000; i < 0xD0000000 + 0xBB800; i += 2)
  //  *(uint16_t *)i = 0x0700;

  //fill_rect(0x0000,0,0,800,480,0);

  //fill_rect(0xffff,100,200,100,100,0);
  //fill_rect(0x0700,200,100,100,100,0);
  //fill_rect(0x001f,200,200,100,100,0);
  //fill_rect(0xf000,100,100,100,100,0);



  infstream.avail_in = vnclogo_length; // size of input
  infstream.next_in = &vnclogo; // input char array
  infstream.avail_out = 800*480*TFT_PIXEL_SIZE;
  infstream.next_out = TFT_BUFFER_START;
  inflate(&infstream,Z_NO_FLUSH);



  chMtxObjectInit(&SD4mtx);
  sduObjectInit(&SDU2);
  sduStart(&SDU2, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);




  palSetPadMode(GPIOG, 13, PAL_MODE_OUTPUT_PUSHPULL); 
  palSetPadMode(GPIOG, 14, PAL_MODE_OUTPUT_PUSHPULL );
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;


  vncbuffer = XFER_BUFFER;

 

  printf("test from printf \r\n");
  chprintf((BaseSequentialStream*)&SD4,"After start thread\r\n");
  chThdSleepMilliseconds(100);


  palSetPad(GPIOG, 14);

  chprintf((BaseSequentialStream*)&SD4,"After set pad Init\r\n");
  chThdSleepMilliseconds(100);


  tcpip_init(NULL, NULL);
  chprintf((BaseSequentialStream*)&SD4,"After tcp Init\r\n");
  chThdSleepMilliseconds(100);



  pppInit();
  chprintf((BaseSequentialStream*)&SD4,"After PPP Init\r\n");
  chThdSleepMilliseconds(100);




  shellInit();
  chprintf((BaseSequentialStream*)&SD4,"After Shell Init\r\n");
  chThdSleepMilliseconds(100);
  chThdSetPriority(PPP_THREAD_PRIO + 1);

  chprintf((BaseSequentialStream*)&SD4,"After set Priority\r\n");
  chThdSleepMilliseconds(100);

	while (TRUE) {
		volatile int connected = 0;
		log_data("before ppp open\r\n");
		//check on deallocating pbuf 
		//check to see if moving buffers works - check mempools
		//check to see if it happens afte two cycles if we use 10 as the timeout or 100
		int pd = pppOverSerialOpen((BaseSequentialStream*)&SD1, ppp_linkstatus_callback,
					   (int*) &connected);

		if (pd < 0) {
		  log_data_num("pd eror %d\r\n",pd);
			chThdSleep(MS2ST(100));
			continue;
		}
		log_data("After ppp open\r\n");
		chThdSleepMilliseconds(100);


		// Wait for initial connection
		int timeout = 0;

		while (connected < 1) {
		  log_data("Connection Wait\r\n");
		  chThdSleep(MS2ST(500));
		  if(timeout++ > 5) {  // If we waited too long restart connection
		    log_data("Close Connection - too long\r\n");
		    chThdSleepMilliseconds(100);
		    pppClose(pd);
		    log_data("After Close Connection - too long\r\n");
		    break;
		  }
			
		}
		log_data_num("entering check stable connection %d\r\n",connected);

		// Make sure connection is stable
		while (connected < 5) {
			chThdSleep(MS2ST(100));
			log_data("connected\r\n");
			if (connected == 0) { // reset by pppThread while waiting for stable connection
			  log_data("Close Connection - not stable\r\n");
			  chThdSleepMilliseconds(100);
			    
			  pppClose(pd);
			  break;
			}
			connected++;
		}

		log_data_num("After check connection stable %d\r\n",connected);
		// Run server threads
	
		echoServerThread = chThdCreateStatic(waVncThread,
						     sizeof(waVncThread), NORMALPRIO + 2, VncThread, NULL);

		//		echoServerThread = chThdCreateStatic(waEchoServerThread,
		//						     sizeof(waEchoServerThread), NORMALPRIO + 1, EchoServerThread, NULL);
		//		shellServerThread = chThdCreateStatic(waShellServerThread,
		//						      sizeof(waShellServerThread), NORMALPRIO + 1, ShellServerThread,
		//						NULL);

		/*		shellServerThread2 = chThdCreateStatic(waShellServerThread2,
				sizeof(waShellServerThread2), NORMALPRIO + 1, ShellServerThread2,
				NULL);*/
		chThdSleep(MS2ST(100));
		while (connected > 0) {
			chThdSleep(MS2ST(200));
		}

		// Tear down threads
		log_data("Terminate Thread\r\n");
		chThdTerminate(echoServerThread);
		log_data("Wait Thread\r\n");
		chThdWait(echoServerThread);
		log_data("should Loop\r\n");
		//		chThdTerminate(shellServerThread);
		//chThdWait(shellServerThread);
	}

  return 0;
}



