// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int main(){
	// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
	int USB = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if ( tcgetattr ( USB, &tty ) != 0 ) {
   		std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}
	cfsetospeed (&tty, B115200);
	cfsetispeed (&tty, B115200);
	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  0;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( USB, TCIFLUSH );
	if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
	   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
	}
	unsigned char cmd[3];
	uint16_t a = 10,b = 10;
	int32_t l,r;
	char s[2];	
	unsigned char read_buf[4];
	while(true){
		std::cin.ignore();
		std::cin.getline(s,1);
		cmd[0] = (0x03f8 & a) >> 3;
		cmd[1] = ((a & 0x7) << 5);
		cmd[1] |= (0x03e0 & b) >> 5;
		cmd[2] = ((b & 0x1f) << 3);
		// cmd[0] = 0xff;
		// cmd[1] = 0;
		// cmd[2] = 0;

		write( USB, cmd, sizeof(cmd) );
		// int num_bytes = read(USB, &read_buf, 4);
		// if (num_bytes == 8) {
		// l = ((int32_t)read_buf[3] << 24) | ((int32_t)read_buf[2] << 16) | ((int32_t)read_buf[1] << 8) | ((int32_t)read_buf[0]);
		// printf("%d\n", l);
		// }
		// printf("1.Read %i bytes. Received message: %d", num_bytes, read_buf[5]);
	}
}