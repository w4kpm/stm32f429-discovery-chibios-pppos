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


//#include "lwip/sockets.h"
#include "lwip/tcpip.h"

//#include "socketstreams.h"

#include "ch.h"
#include "hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "chprintf.h"
#include "shell.h"
#include "sysinfo.h"
#include "sdram.h"
//#include "lwip/sockets.h"
#include <string.h>
#include "ppp/ppp.h"
#include "stm32f4xx.h"
mutex_t SD4mtx;
//#include "lwipthread.h"

#define PKT_BUFFER_START 0xD0100000
#define PKT_BUFFER_LEN   0x00100000
uint8_t *bufferstart;
uint8_t *bufferend;
uint16_t numbytes;

static uint8_t txbuf[2];
static uint8_t rxbuf[2];
static char text[255];
struct tcp_pcb *gpcb;
int tcp_connected;
int tcp_sent_data;
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


#define LCD_WIDTH	 800
#define LCD_HEIGHT	480

static SerialConfig uartCfg =
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

char vncbuffer[2048]; // shouldn't ever get more than 1024
int vncsocket;

void log_data(char* text)
{
    return;
    chprintf((BaseSequentialStream*)&SD4,text);
}

void log_data_num(char* text,int data)
{

    chprintf((BaseSequentialStream*)&SD4,text,data);
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
    buffpos = &vncbuffer;
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
    data = data | vncbuffer[1];
    data = data | vncbuffer[0] << 8;
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





void read_all_data(char* str,uint16_t len)
{
    /* reads all bytes left in queue - NB - it looks like lwip breaks 
       at the end of each remote transition, so even though this completes, 
       there may be another packet waiting */
    int x;
    int numread;

    if (numbytes < len)
	{
	    chprintf((BaseSequentialStream*)&SD4,"only %d bytes - will block \r\n",numbytes);
	    return;
	}
    numread=rx(&vncbuffer,len);
    chprintf((BaseSequentialStream*)&SD4,"%s Data Dump:\r\n    ",str);
    for (x=0;x<numread;x++)
	chprintf((BaseSequentialStream*)&SD4,"0x%X ",vncbuffer[x]);
    chprintf((BaseSequentialStream*)&SD4,"\r\n");


}



void read_copy_rect(int xpos, int ypos, int width, int height)
{
    uint16_t sourcex,sourcey;
    char text[255];
    sourcex = read_16();
    sourcey = read_16();
    //sprintf(text,"copy-rect. source %d,%d dest: %d,%d size:%d,%d\r\n",sourcex,sourcey,xpos,ypos,height,width);
    //log_data(text);
}


void read_raw_rect(int x,int y, int w, int h)
{
    int bytecount;
    char text[255];
    log_data_num("Raw Rect: reading %d bytes\r\n",(w * h * 4));
    ensure_bytes((w * h * 4));
	
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
//void fill_rect(uint32_t color, int x, int y, int width, int height)
//{
//    char text[255];
    //sprintf(text,"Fill Rect %X pos:%d,%d size:%d,%d\r\n",color,x,y,height,width);
    //log_data(text);
//}


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
void fill_rect(uint32_t color, int x, int y, int width, int height,int xlate) 
{
    DMA2D->CR = 3 << 16;
    DMA2D->OPFCCR = 0x2;
    if (xlate ==0)
	DMA2D->OCOLR = color;
    else
	DMA2D->OCOLR = translate_color(color);
  
    DMA2D->OMAR = 0xD0000000 + (800 * y *2) + x*2;
    DMA2D->OOR = LCD_WIDTH - width;
    DMA2D->NLR = (uint32_t) ((width << 16) | height);
    DMA2D->CR |= 1;
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
	    currentcolor = read_32();
	    xy = read_8();
	    wh = read_8();
	    y = xy & 0xf;
	    x = (xy & 0xf0) >> 4;
	    h = wh & 0xf;
	    w = (wh & 0xf0) >> 4;
	    fill_rect(currentcolor,startX + x,startY+y,w+1,h+1,1);
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
	    fill_rect(fgcolor,startX + x,startY+y,w+1,h+1,1);
	}
}
	    



void read_hextile_rect(int x,int y, int width, int height)
{
    int y2,x2;
    char text[255];
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
				
				hextile_bg_color = read_32();
				//log_data_num("new bg color %X\r\n",hextile_bg_color);			    
			    }
			if (foreground)
			    {
				hextile_fg_color = read_32();
				//log_data_num("new fg color %X\r\n",hextile_fg_color);			    }
			    }
			if (anysubrects)
			    {
				subrect_count = read_8();
				//log_data_num("subrec Count %d\r\n",subrect_count);
			    }
			// fill bg rect here 
		        fill_rect(hextile_bg_color,x2*16+x,y2*16+y,newwidth,newheight,1);
			//  I think these are mutually exclusive.
			if (coloredrects)
			    process_colored_rects(subrect_count,x2*16+x,y2*16+y);
			else
			    if (anysubrects)
				process_foreground_rects(hextile_fg_color,subrect_count,x2*16+x,y2*16+y);
		    }
		
		
		
	    }
	    
}





void read_fb_rect(void)
{
    uint16_t xpos,ypos,height,width;
    uint32_t encoding;
    char text[255];
    xpos = read_16();
    ypos = read_16();
    height = read_16();
    width = read_16();
    encoding = read_32();
    //sprintf(text,"New Rect %d %d %d %d %d\r\n",xpos,ypos,height,width,encoding);
    //log_data(text);
    switch (encoding)
	{
	case 0: 
	    read_raw_rect(xpos,ypos,height,width);
	    break;
	case 1:
	    read_copy_rect(xpos,ypos,height,width);
	    break;
        case 5:
	    read_hextile_rect(xpos,ypos,height,width);
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
    rx(&vncbuffer,3); // per the protocol, there is 
                                         // three bytes of padding
    strLength= read_32();
    ensure_bytes(strLength);
    vncbuffer[strLength] = 0;
    log_data("name:\r\n     ");
    log_data(vncbuffer);
    log_data("\r\n");

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

    len = send(&vncbuffer,10);
    if (len != 10)
	log_data("framebuffer Incorrect length\r\n");
    process_frame_response();
}



void set_encodings()
{
    int x;
    
    vncbuffer[0] = 2; // message
    vncbuffer[1] = 0; // padding
    set_16(vncbuffer+2,3); // we're using 3 encodings
    set_32(vncbuffer+4,5); // hextile
    set_32(vncbuffer+8,1); // copyrect
    set_32(vncbuffer+12,0); // raw

    // - only to print out raw bytes and make sure things were 
    // in correct order.

    //chprintf((BaseSequentialStream*)&SD4,"Encoding:\r\n    ");
    //for (x=0;x<16;x++)
    //	chprintf((BaseSequentialStream*)&SD4,"0x%X ",vncbuffer[x]);
    //chprintf((BaseSequentialStream*)&SD4,"\r\n");
    x = send(&vncbuffer,16);
    if (x != 16)
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
	chprintf((BaseSequentialStream*)&SD4,"tcp_write= %d\r\n",err);
    while (tcp_sent_data ==0)
	{
	    //	    chprintf((BaseSequentialStream*)&SD4,"o");
	    chThdSleepMilliseconds(1);

	    
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
	    //chprintf((BaseSequentialStream*)&SD4,"x");

	    chThdSleepMilliseconds(1);
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
     chprintf((BaseSequentialStream*)&SD4,"got to callback \r\n");
     if (err == ERR_OK)
	 {
	     tcp_connected = 1;
	     chprintf((BaseSequentialStream*)&SD4,"Connected!\r\n");
	 }
     else
	 {
	     chprintf((BaseSequentialStream*)&SD4,"connection error: %x\r\n",err);
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




static THD_WORKING_AREA(waVncThread, 2048 );
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
	chprintf((BaseSequentialStream*)&SD4,"VNC Client Starting \r\n");

	gpcb = tcp_new();	
	tcp_sent(gpcb,tcp_sent_callback);
	tcp_recv(gpcb,tcp_recv_callback);
	error = tcp_connect(gpcb,&dest,SOCK_TARGET_PORT,tcp_connect_callback);

	if (error != ERR_OK)
	    chprintf((BaseSequentialStream*)&SD4,"error colling connect %d \r\n",error);
	else
	    chprintf((BaseSequentialStream*)&SD4,"tcp_connect OK \r\n");
	while (tcp_connected == 0)
	    {
		chprintf((BaseSequentialStream*)&SD4,"-");
		chThdSleepMilliseconds(100);
	    }
	    

	//	vncsocket = lwip_socket(AF_INET,  SOCK_STREAM, IPPROTO_TCP);
	//if (vncsocket == -1) {
	//    chprintf((BaseSequentialStream*)&SD4,"Closing A - no socket \r\n");

	//		return MSG_RESET;
	//	}

	//	memset(&sa, 0, sizeof(sa));
//	sa.sin_family = AF_INET;
//	sa.sin_port = PP_HTONS(SOCK_TARGET_PORT);
//	sa.sin_addr.s_addr = inet_addr(SOCK_TARGET_HOST);
//	fromlen = sizeof(sa);
//	ret = lwip_connect(vncsocket, (struct sockaddr*)&sa, sizeof(sa));
//	chprintf((BaseSequentialStream*)&SD4,"Returned '%d' \r\n",ret);
	connect_vnc();
	//chThdSleepMilliseconds(5000);
		framebuffer_request(0,0,800,480,0);
	while (TRUE)
	    {
//		log_data("****************************************\r\n");
//		//read_all_data("getting ready to read");
		//chprintf((BaseSequentialStream*)&SD4,".");
		//chThdSleepMilliseconds(1000);
		framebuffer_request(0,0,800,480,1);

	    }
	
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

	shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO+2);
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

  chMtxObjectInit(&SD4mtx);
  sdStart(&SD1, &uartCfg);
  sdStart(&SD4, &uartCfg);

  palSetPadMode(GPIOG, 13, PAL_MODE_OUTPUT_PUSHPULL); 
  palSetPadMode(GPIOG, 14, PAL_MODE_OUTPUT_PUSHPULL );
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;

  SDRAM_Init();
  
  TFTLCD_Init();
  //  chThdSleepMilliseconds(1000);

  //for(i = 0xD0000000; i < 0xD0000000 + 0xBB800; i += 2)
  //  *(uint16_t *)i = 0x0700;
  fill_rect(0x0000,0,0,800,480,0);

  fill_rect(0xffff,100,200,100,100,0);
  fill_rect(0x0700,200,100,100,100,0);
  fill_rect(0x001f,200,200,100,100,0);
  fill_rect(0xf000,100,100,100,100,0);


  fill_rect(0xffffff,300,200,100,100,1);
  fill_rect(0xff0000,400,100,100,100,1);
  fill_rect(0x00ff00,400,200,100,100,1);
  fill_rect(0x0000ff,300,100,100,100,1);





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

		int pd = pppOverSerialOpen(&SD1, ppp_linkstatus_callback,
				(int*) &connected);

		if (pd < 0) {
			chThdSleep(MS2ST(100));
			continue;
		}
		chprintf((BaseSequentialStream*)&SD4,"After ppp open\r\n");
		chThdSleepMilliseconds(100);


		// Wait for initial connection
		int timeout = 0;

		while (connected < 1) {
			chThdSleep(MS2ST(500));
			if(timeout++ > 10) {  // If we waited too long restart connection
			    chprintf((BaseSequentialStream*)&SD4,"Close Connection - too long\r\n");
			    chThdSleepMilliseconds(100);

				pppClose(pd);
				break;
			}
			
		}
		chprintf((BaseSequentialStream*)&SD4,"entering connection \r\n");

		// Make sure connection is stable
		while (connected < 5) {
			chThdSleep(MS2ST(100));
			printf("Connected\r\n");
			if (connected == 0) { // reset by pppThread while waiting for stable connection
			    chprintf((BaseSequentialStream*)&SD4,"Close Connection - not stable\r\n");
			    chThdSleepMilliseconds(100);

				pppClose(pd);
				break;
			}
			connected++;
		}

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

		while (connected > 0) {
			chThdSleep(MS2ST(200));
		}

		// Tear down threads
		chThdTerminate(echoServerThread);
		chThdWait(echoServerThread);
		//		chThdTerminate(shellServerThread);
		//chThdWait(shellServerThread);
	}

  return 0;
}



