//
// CS 431 - Lab 03 Server Skeleton
// PC/Linux (Provided)
//

#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "pc_crc16.h"
#include "lab03.h"

int main(int argc, char* argv[])
{
	double troll_pct=0;		// Perturbation % for the troll (if needed)
	int ifd,ofd,i,N,troll=0;	// Input and Output file descriptors (serial/troll)
	char str[MSG_BYTES_MSG],opt;	// String input
	char c;
	int ack = 0;
	int cnt = 0;
	int mCrc,attempts=0;
	int errno = -1;
	struct termios oldtio, tio;	// Serial configuration parameters
	int VERBOSE = 0;		// Verbose output - can be overriden with -v

	// Command line options
	while ((opt = getopt(argc, argv, "t:v")) != -1) {
		switch (opt) {
			case 't':	troll = 1; 
					troll_pct = atof(optarg);
					break;
			case 'v':	VERBOSE = 1; break;
			default: 	break;
		}
	}

	printf("CS431 - Lab 03 Server\n(Enter a message to send.  Type \"quit\" to exit)\n");


	//
	// WRITE ME: Open the serial port (/dev/ttyS0) read-write
	//
	ifd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK);//O_RDWR);
	if (ifd == -1){
		printf("Error Reading/Writing TTYS0");
		return -1;
	}
	printf("file descriptor opened\n");


	// Start the troll if necessary
	if (troll)
	{
		// Open troll process (lab03_troll) for output only
		FILE * pfile;		// Process FILE for troll (used locally only)
		char cmd[128];		// Shell command

		snprintf(cmd, 128, "./lab03_troll -p%f %s", troll_pct, (VERBOSE) ? "-v" : "");

		pfile = popen(cmd, "w");
		if (!pfile) { perror("lab03_troll"); exit(-1); }
		ofd = fileno(pfile);
	}
	else ofd = ifd;		// Use the serial port for both input and output
	printf("done trolling\n");


	//
 	// WRITE ME: Set up the serial port parameters and data format
	//
	tcgetattr(ifd, &oldtio); // read default config
	//memcpy ( &tio, &oldtio, sizeof(struct termios) );
 
	// change config here
	//tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
	//tio.c_cflag |= B9600;
	//tio.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        //tio.c_cflag &= ~CSTOPB;
	//tcgetattr(ifd, &oldtio);
	tio.c_cflag 	= B9600 | CS8 | CLOCAL;
	tio.c_iflag 	= 0;
	tio.c_oflag 	= 0;
	tio.c_lflag 	= 0;
	tcflush(ifd, TCIFLUSH);
	tcsetattr(ifd, TCSANOW, &tio);
 
	/*if (tcsetattr (ifd, TCSANOW, &tio) != 0)
	{
	    fprintf (stderr, "error %d from tcsetattr", errno);
	    return -1;
	}*/
	printf("new config set\n");


	int dfid = fopen("hexdump","w");
	while(1)
	{
		cnt = 0;
		//
		// WRITE ME: Read a line of input (Hint: use fgetc(stdin) to read each character)
		//
		while (1){
			c = fgetc(stdin);
			if (c == '\n')
				break;
			str[cnt++] = c;
		}
		str[cnt] = NULL;
		printf("%s\n",str);
		

		if (strcmp(str, "quit") == 0) break;

		//
		// WRITE ME: Compute crc (only lowest 16 bits are returned)
		//
		mCrc = pc_crc16(str, cnt);
		printf("%d", mCrc);
	
		ack = 0;
		while (!ack)
		{
			printf("Sending (attempt %d)...\n", ++attempts);

			
			// 
			// WRITE ME: Send message
			//
			write (ifd, MSG_START,  MSG_BYTES_START);
			write (ifd, mCrc, MSG_BYTES_CRC);
			write (ifd, sizeof(str), MSG_BYTES_MSG_LEN);
			write (ifd, str, sizeof(str));

			write (dfid, MSG_START,  MSG_BYTES_START);
			write (dfid, mCrc, MSG_BYTES_CRC);
			write (dfid, sizeof(str), MSG_BYTES_MSG_LEN);
			write (difd, str, sizeof(str));
 
			printf("SENT MESSAGE\n");

		
			printf("Message sent, waiting for ack... ");

			
			//
			// WRITE ME: Wait for MSG_ACK or MSG_NACK
			//
			read(ifd, c, 1);
			printf("receive char %d\n",c);
			if (c == MSG_ACK)
				ack = 1;
			else if (c == MSG_NACK)
				ack = 0;
			

			printf("%s\n", ack ? "ACK" : "NACK, resending");
		}
		printf("\n");
	}



	//
	// WRITE ME: Reset the serial port parameters
	//
	if (tcsetattr (ifd, TCSANOW, &oldtio) != 0)
	{
	    fprintf (stderr, "error %d from tcsetattr", errno);
	    return -1;
	}


	// Close the serial port
	close(ifd);
	close(dfid);
	
	return 0;
}
