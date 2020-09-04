#include <inttypes.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>			//Used for UART
#include <iostream>




using namespace std;

//Uart stuff
int uart0_filestream = -1;

unsigned int channel[16];




int main() {


    	uart0_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
    	if (uart0_filestream == -1)
        	cout <<("Error - Unable to open UART.  Ensure it is not in use by another application\n");

    	struct termios options;
    	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B100000 | CS8 | CLOCAL | CREAD; // Set baud
    	options.c_iflag = IGNPAR;
    	options.c_oflag = 0;
    	options.c_lflag = 0;
    	tcflush(uart0_filestream, TCIFLUSH);
    	tcsetattr(uart0_filestream, TCSANOW, &options);




    	unsigned char tx_buffer[27];
    	unsigned char *p_tx_buffer;

	while (true){

	p_tx_buffer = &tx_buffer[0];

	*p_tx_buffer++ = (uint8_t) 0x0F;
	*p_tx_buffer++ = (uint8_t) ((channel[0] & 0x07FF));
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
        *p_tx_buffer++ = (uint8_t) ((channel[10] & 0x07FF)>>2);
        *p_tx_buffer++ = (uint8_t) ((channel[10] & 0x07FF)>>10 | (channel[11] & 0x07FF)<<1);
        *p_tx_buffer++ = (uint8_t) ((channel[11] & 0x07FF)>>7 | (channel[12] & 0x07FF)<<4);
        *p_tx_buffer++ = (uint8_t) ((channel[12] & 0x07FF)>>4 | (channel[13] & 0x07FF)<<7);
        *p_tx_buffer++ = (uint8_t) ((channel[13] & 0x07FF)>>1);
        *p_tx_buffer++ = (uint8_t) ((channel[13] & 0x07FF)>>9 | (channel[14] & 0x07FF)<<2);
        *p_tx_buffer++ = (uint8_t) ((channel[14] & 0x07FF)>>6 | (channel[15] & 0x07FF)<<5);
        *p_tx_buffer++ = (uint8_t) ((channel[15] & 0x07FF)>>3);
	*p_tx_buffer++ = (uint8_t) 0x0F;
	*p_tx_buffer++ = (uint8_t) 0x00;

    	if (uart0_filestream != -1) {
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));
		if (count < 0)
			cout <<"UART TX error"<< endl;
    	}
	usleep(70000);
	}
  	return 0;
}
