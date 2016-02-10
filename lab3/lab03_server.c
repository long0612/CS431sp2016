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
	


	//
 	// WRITE ME: Set up the serial port parameters and data format
	//



	while(1)
	{

		//
		// WRITE ME: Read a line of input (Hint: use fgetc(stdin) to read each character)
		//

		if (strcmp(str, "quit") == 0) break;

		//
		// WRITE ME: Compute crc (only lowest 16 bits are returned)
		//
	
		while (!ack)
		{
			printf("Sending (attempt %d)...\n", ++attempts);

			
			// 
			// WRITE ME: Send message
			//

		
			printf("Message sent, waiting for ack... ");

			
			//
			// WRITE ME: Wait for MSG_ACK or MSG_NACK
			//


			printf("%s\n", ack ? "ACK" : "NACK, resending");
		}
		printf("\n");
	}



	//
	// WRITE ME: Reset the serial port parameters
	//



	// Close the serial port
	close(ifd);
	
	return 0;
}
