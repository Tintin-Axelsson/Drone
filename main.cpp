#include <fcntl.h> //open()
#include <unistd.h> //write()
#include <string>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <iostream>

#define BAUD 100000

int main(int argc, char *argv[])
{
	int fd;
	struct termios2 ntio;
	unsigned char tx_buffer[20];
    	unsigned char *p_tx_buffer;
	unsigned int channel[16];

	for(int i = 0; i < 16; i++)
		channel[i] = 993;
		

    	fd = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);
    	ioctl(fd, TCGETS2, &ntio);

    	ntio.c_cflag &= ~CBAUD;		//Has to be done to set non-standard BAUD
    	ntio.c_cflag |= BOTHER | CREAD;

	ntio.c_cflag |= PARENB;		//Parity enable
	ntio.c_cflag &= ~PARODD;
	ntio.c_cflag |= CSTOPB;		//Dual Stop

    	ntio.c_ispeed = BAUD;	//Setting BAUD
    	ntio.c_ospeed = BAUD;
    	ioctl(fd, TCSETS2, &ntio);
    

	int counter = 0;
	channel[2] = 300;
	channel[5] = 300;

	while (true) {
	p_tx_buffer = &tx_buffer[0];
	
	if(counter == 1000) {
		channel[5] = 1800;
		std::cout<<"Channel 5 set 1800"<< std::endl;
	}
	else if(counter == 2000) {
		channel[2] = 900;
		std::cout<<"Channel 2 set 900"<< std::endl;
	}
	counter++;

	*p_tx_buffer++ = (uint8_t) 0x0F; //SBUS header 
	*p_tx_buffer++ = (uint8_t) ((channel[0] & 0x07FF));
	*p_tx_buffer++ = (uint8_t) ((channel[0] & 0x07FF)>>8 | (channel[1] & 0x07FF)<<3);
        *p_tx_buffer++ = (uint8_t) ((channel[1] & 0x07FF)>>5 | (channel[2] & 0x07FF)<<6);
        *p_tx_buffer++ = (uint8_t) ((channel[2] & 0x07FF)>>2);
        *p_tx_buffer++ = (uint8_t) ((channel[2] & 0x07FF)>>10 | (channel[3] & 0x07FF)<<1);
        *p_tx_buffer++ = (uint8_t) ((channel[3] & 0x07FF)>>7 | (channel[4] & 0x07FF)<<4);
        *p_tx_buffer++ = (uint8_t) ((channel[4] & 0x07FF)>>4 | (channel[5] & 0x07FF)<<7);
        *p_tx_buffer++ = (uint8_t) ((channel[5] & 0x07FF)>>1);
        *p_tx_buffer++ = (uint8_t) ((channel[5] & 0x07FF)>>9 | (channel[6] & 0x07FF)<<2);
        *p_tx_buffer++ = (uint8_t) ((channel[6] & 0x07FF)>>6 | (channel[7] & 0x07FF)<<5);
        *p_tx_buffer++ = (uint8_t) ((channel[7] & 0x07FF)>>3);
        *p_tx_buffer++ = (uint8_t) ((channel[8] & 0x07FF));
        *p_tx_buffer++ = (uint8_t) ((channel[8] & 0x07FF)>>8 | (channel[9] & 0x07FF)<<3);
        *p_tx_buffer++ = (uint8_t) ((channel[9] & 0x07FF)>>5 | (channel[10] & 0x07FF)<<6);
        *p_tx_buffer++ = (uint8_t) ((channel[10] & 0x07FF)>>10 | (channel[11] & 0x07FF)<<1);
        *p_tx_buffer++ = (uint8_t) ((channel[11] & 0x07FF)>>7 | (channel[12] & 0x07FF)<<4);
        *p_tx_buffer++ = (uint8_t) ((channel[12] & 0x07FF)>>4 | (channel[13] & 0x07FF)<<7);
        *p_tx_buffer++ = (uint8_t) ((channel[13] & 0x07FF)>>1);
        *p_tx_buffer++ = (uint8_t) ((channel[13] & 0x07FF)>>9 | (channel[14] & 0x07FF)<<2);
        *p_tx_buffer++ = (uint8_t) ((channel[14] & 0x07FF)>>6 | (channel[15] & 0x07FF)<<5);
        *p_tx_buffer++ = (uint8_t) ((channel[15] & 0x07FF)>>3);
        *p_tx_buffer++ = (uint8_t) 0x00;	//Flags
        *p_tx_buffer++ = (uint8_t) 0x00;	//Footer
	



	if(fd != -1) {
    		int count = write(fd, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));
		if (count < 0)
			std::cout << "UART TX error" << std::endl;
	}
	usleep(2900);
	}

    return 0;
}



