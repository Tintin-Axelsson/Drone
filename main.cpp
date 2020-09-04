#include <inttypes.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <iostream>

using namespace std;

//Uart stuff
int uart0_filestream = -1;



/* write SBUS packets */
void write(uint16_t* channels) {
    static uint8_t packet[25];
    packet[0] = (uint8_t) 0x0F;
    if (channels)
    {
        packet[1] = (uint8_t) ((channels[0] & 0x07FF));
        packet[2] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
        packet[3] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
        packet[4] = (uint8_t) ((channels[2] & 0x07FF)>>2);
        packet[5] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
        packet[6] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
        packet[7] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
        packet[8] = (uint8_t) ((channels[5] & 0x07FF)>>1);
        packet[9] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
        packet[10] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
        packet[11] = (uint8_t) ((channels[7] & 0x07FF)>>3);
        packet[12] = (uint8_t) ((channels[8] & 0x07FF));
        packet[13] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
        packet[14] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);
        packet[15] = (uint8_t) ((channels[10] & 0x07FF)>>2);
        packet[16] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
        packet[17] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
        packet[18] = (uint8_t) ((channels[12] & 0x07FF)>>4 | (channels[13] & 0x07FF)<<7);
        packet[19] = (uint8_t) ((channels[13] & 0x07FF)>>1);
        packet[20] = (uint8_t) ((channels[13] & 0x07FF)>>9 | (channels[14] & 0x07FF)<<2);
        packet[21] = (uint8_t) ((channels[14] & 0x07FF)>>6 | (channels[15] & 0x07FF)<<5);
        packet[22] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    }
    // flags
    packet[23] = 0x00;
    // footer
    packet[24] = (uint8_t) 0x00;
}

unsigned int channel[16];

int main() {
	channel[0] = 2047;
    	uart0_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    	if (uart0_filestream == -1)
    	{
        	//ERROR - CAN'T OPEN SERIAL PORT
        	cout <<("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    	}
    	struct termios options;
    	tcgetattr(uart0_filestream, &options);
    	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;	// Set baud rate
    	options.c_iflag = IGNPAR;
    	options.c_oflag = 0;
    	options.c_lflag = 0;
    	tcflush(uart0_filestream, TCIFLUSH);
    	tcsetattr(uart0_filestream, TCSANOW, &options);

    	unsigned char tx_buffer[20];
    	unsigned char *p_tx_buffer;


	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = (uint8_t) 0x0F;
	*p_tx_buffer++ = (uint8_t) ((channel[0] & 0x07FF));
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'o';

    	if (uart0_filestream != -1) {
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));
		if (count < 0)
			cout <<"UART TX error"<< endl;
    	}
  	return 0;
}
