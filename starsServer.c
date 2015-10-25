/*
    C socket server example, handles multiple clients using threads
*/
 
#include<stdio.h>
#include<string.h>    //strlen
#include<stdlib.h>    //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include<errno.h>
#include<unistd.h>    //write
#include<pthread.h> //for threading , link with lpthread
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define PI 3.14159265

#define TRUE    (1==1)
#define FALSE   (!TRUE)

#define STAR_HUBS               20

#define SPI_CHAN                0
#define NUM_TIMES               100
#define MAX_SIZE                (1024*1024)

#define WRITE_COMMAND           0b100101L
#define CONTROL_BITS            0b10110L

static int myFd ;

//  Machine State:
//  [0..7]  	Program
//  [8..15] 	Brightness 
//  [16..23]	Sound (?)
//  [24..31]	Unassigned

long machineState = 1;
long stateCtr = 0;

int StarsBrightness[180] = {1,1,1,3,3,3,1,0,0,0,2,0,3,3,2,2,1,0,0,0,2,1,1,2,0,1,0,0,0,1,1,0,0,1,2,2,5,3,1,0,0,0,2,1,2,1,2,2,0,1,2,0,0,1,0,0,0,0,0,0,1,0,1,2,1,3,2,0,0,3,0,2,1,3,0,3,0,1,1,1,2,0,1,0,1,0,2,0,0,2,0,0,1,0,0,0,1,0,1,0,1,0,3,3,0,1,0,0,1,0,0,0,1,1,0,0,2,1,0,1,1,0,0,0,0,0,1,0,0,2,0,0,3,1,1,2,0,1,0,2,1,0,0,1,2,2,1,1,1,1,0,2,0,1,1,1,0,1,1,1,1,0,2,0,0,1,1,1,0,0,0,0,2,1,1,3,3,0,0,1};
double StarsX[180] = {290.810811,317.055556,329.264706,349.5,365.833333,380.756098,371.5,368.083333,391.088235,395.724138,356.5,342.078947,301.815789,296.571429,321.027027,289.088235,261.925926,289.724138,216.178571,245.625,237.540541,237.611111,190.53125,226.8,197.113636,222.5,271.222222,159.333333,171.129032,261.7,257.282051,244.533333,213,202.114286,184.147059,199.942857,137.071429,128.864865,172.25,197.230769,193.071429,231.05,108.909091,116.5,126.5,125.571429,135.638889,97.548387,120.852941,126.633333,115.214286,261.190476,219.925,284.5,257,165.5,119.833333,168.897436,158.925,158.684211,175.027778,177.617647,150.575758,134.081081,159.916667,166.2,191.828571,180.818182,170.5,188.181818,173.65625,167.5,250.631579,207.606061,232.65625,260.5,249,267.357143,277.857143,225.871795,208.925,206.5,214.837838,194,298.571429,297.368421,293.611111,280.833333,279.571429,275.756098,269.5,289,273.515152,283,302.5,305.5,374.214286,357.44186,334.166667,306.71875,322.789474,339.777778,301.756098,315.368421,324.5,304.833333,332.935484,340,385,388,388,391.837209,399.871795,418.380952,441.424242,419.128205,398.702703,375.357143,374.363636,355.073171,447.944444,431.5,419.5,395.794118,407.366667,395.5,386.28125,391.037037,406.789474,409.419355,422.885714,435.925926,454.571429,461.5,454.805556,474.5,458.545455,472.925926,487.029412,478.128205,496.74359,495.580645,494.34375,511.909091,473.243243,480.228571,465.818182,464.5,459.871795,468.090909,525.5,485.15625,520.5,508.424242,531.027778,500.903226,396.928571,404.152174,421.925,404.333333,455.909091,412.205882,433.8,457.903226,439.5,424.171429,379.857143,425,386.424242,357.02439,366.071429,357.717949,390.463415,294.282051,334.459459,367.815789,326,343,372.315789,302.142857};
double StarsY[180] = {33.756757,74.805556,111,114.5,120.5,100,70.5,79.972222,130.029412,93.586207,54.5,60.552632,129.315789,105.771429,100.162162,63.794118,35.407407,30.413793,50.607143,53.15625,68.918919,76.861111,86.1875,121.925,221.795455,165.5,237.333333,190.222222,139.129032,156.175,138.051282,127.288889,126.892857,116.885714,79.470588,125.571429,288.809524,258.702703,255,244.846154,280.190476,243.075,257.30303,266.36,297.5,300.771429,302.083333,316.419355,379.382353,394.633333,388.5,336.071429,281.65,324.705882,294.410256,297.5,323.5,334.641026,343.2,335.184211,383.083333,372.147059,372.424242,374.459459,405.972222,395.075,441,442.181818,414.5,367.181818,364,362.5,411.078947,423.515152,437.375,458.5,399,356.095238,467.214286,364.871795,366.2,387.294118,375.405405,376,380.771429,375.5,369.138889,444.566667,396.228571,366,429.5,423.3,452.393939,461.5,459,457.5,440.5,420.55814,433.805556,367.1875,309.894737,320.083333,392,382.078947,386.5,404.5,443.580645,448.735294,372.756098,398,382.5,359.55814,366.871795,341.095238,338.060606,329.128205,346.864865,323.904762,378.909091,362.292683,398.194444,405.5,396.5,408.088235,414.633333,417.4,409.8125,431.814815,430.894737,464.129032,442.114286,444.407407,385.228571,401,415.055556,410.5,424.090909,429.407407,411.617647,382.128205,377.25641,404.451613,394.46875,371.30303,224.243243,247.571429,307.181818,324.5,358.871795,366.30303,333.705882,348.625,327,301.424242,252.083333,229.677419,271.809524,290.565217,281.8,255,244.69697,308.911765,199.075,227.677419,180.5,158,150.285714,226.243902,231.424242,205.585366,223.809524,238.897436,206.121951,302.897436,163.081081,203.684211,249.589744,246.5,283.684211,157.285714};
int indexOffset = 60;

//the thread functions
void *connection_handler(void *);
void *star_driver();

void spiSetup (int speed)
{
  if ((myFd = wiringPiSPISetup (SPI_CHAN, speed)) < 0)
  {
    fprintf (stderr, "Can't open the SPI bus: %s\n", strerror (errno)) ;
    exit (EXIT_FAILURE) ;
  }
}

int main(int argc , char *argv[])
{
    int socket_desc , client_sock , c , *new_sock;
    struct sockaddr_in server , client;

    char consoleMessage[2000];
    int j;
    for (j=0; j<180; j++) {
	sprintf(consoleMessage, "%d\tX:%f\tY:%f\tB:%d",j,StarsX[j],StarsY[j],StarsBrightness[j]);
        puts(consoleMessage);
    }


    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( 8889 );

    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        //print the error message
        perror("bind failed. Error");
        return 1;
    }
    puts("bind done");

    //  Starting lighting thread
    pthread_t lights_thread;
    if( pthread_create( &lights_thread , NULL ,  star_driver, NULL) < 0)
        {
            perror("could not create star thread");
            return 1;
        }


    //Listen
    listen(socket_desc , 3);

    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);

    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);
    while( (client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)) )
    {
        puts("Connection accepted");

        pthread_t sniffer_thread;
        new_sock = malloc(1);
        *new_sock = client_sock;

        if( pthread_create( &sniffer_thread , NULL ,  connection_handler , (void*) new_sock) < 0)
        {
            perror("could not create thread");
            return 1;
        }

        //Now join the thread , so that we dont terminate before the thread
        //pthread_join( sniffer_thread , NULL);
        puts("Handler assigned");
    }

    if (client_sock < 0)
    {
        perror("accept failed");
        return 1;
    }

    return 0;
}

/*
 * This will handle connection for each client
 * */
void *connection_handler(void *socket_desc)
{
    //Get the socket descriptor
    int sock = *(int*)socket_desc;
    int read_size;
    char *message , client_message[2000];

    //Send some messages to the client
    message = "Greetings! I am your connection handler\n";
    write(sock , message , strlen(message));

    message = "Now type something and i shall repeat what you type \n";
    write(sock , message , strlen(message));

    //Receive a message from client
    while( (read_size = recv(sock , client_message , 2000 , 0)) > 0 )
    {
        //Send the message back to client
        //write(sock , client_message , strlen(client_message));
	if ((client_message[5] == 0x65) & (client_message[4] == 0x7A)) 
		{
		machineState = (machineState & 0xFFFFFF00) | client_message[3];
		machineState = (machineState & 0xFFFF00FF) | (client_message[2]<<8);
		machineState = (machineState & 0xFF00FFFF) | (client_message[1]<<16);
		machineState = (machineState & 0x00FFFFFF) | (client_message[0]<<24);
		stateCtr = 0;
		sprintf(client_message, "Setting to %lX\n", machineState);
		puts(client_message);
		}  else {
		sprintf(client_message,"%X\t%X\n",(client_message[4]),(client_message[5]));
		}
        write(sock , client_message , strlen(client_message));
	
    }

    if(read_size == 0)
    {
        puts("Client disconnected");
        fflush(stdout);
    }
    else if(read_size == -1)
    {
        perror("recv failed");
    }

    //Free the socket pointer
    free(socket_desc);

    return 0;
}

void *star_driver() {
	#include <stdio.h>
	#include <stdlib.h>
	#include <unistd.h>
	#include <stdint.h>
	#include <string.h>
	#include <errno.h>
	#include <time.h>
	#include <math.h>
	#include <wiringPi.h>
	#include <wiringPiSPI.h>

	unsigned char *myData ;
	int bg_brightness, fg_brightness, brightness, header_bits, size;
	float const_b1, const_b2, const_b3;
	// Constellations
	//
	// 0 - No constellation
	// 1 - Cygnus
	// 2 - Lacerta
	// 3 - Andromeda
	// 4 - Perseus
	// 5 - Cephus
	// 6 - Cassiopeia
	// 7 - Hercules
	// 8 - Draco
	// 9 - Ursa Minor
	// 10 - Camelopardalis
	// 11 - Auriga
	// 12 - Canes Venatici
	// 13 - Ursa Major
	// 14 - Leo Minor
	// 15 - Lynx
	int constellations[16][100] = {  // First value will be number of stars in constellation
		{20, 66, 69, 65, 67, 64, 63,74,73, 228, 229, 232, 238, 161, 162, 178, 174, 177, 180, 165, 237},
		{8, 198, 196, 200, 204, 201, 193, 212, 208},
		{10, 170, 187, 184, 186, 185, 188, 189, 191, 192, 190},
		{15, 157, 168, 167, 159, 155, 156, 154, 139, 136, 153, 148, 151, 133, 135, 134},
		{22, 137, 142, 140, 141, 143, 144, 130, 122, 131, 132, 119, 120, 118, 121, 126, 125, 129, 123, 124, 109, 110, 111},
		{5, 177, 169, 180, 178, 161},
		{6, 164, 160, 163, 147, 150, 138},
		{2, 215, 216},
		{15, 205, 205, 221, 224, 219, 218, 217, 220, 228, 223, 225, 226, 227, 235, 240},
		{7, 229, 233, 236, 231, 232, 238, 237},
		{6, 115, 113, 102, 87, 101, 116},
		{8, 99, 97, 98, 105, 106, 107, 108, 103},
		{2, 71, 72},
		{20, 63, 64, 65, 66, 62, 61, 76, 78, 73, 74, 75, 81, 82, 84, 92, 91, 90, 86, 96, 94},
		{3, 77, 79, 80},
		{6, 100, 85, 88, 89, 83, 95}
		};
	double raindrops[10][184]; // Center X, Center Y, Girth, Start Time, Star Radii
	int raindropCounter = 0;
	int raindropFrequency;
	double starRadius;
	int b, t, ctr = 0;
	int i, j, k;
	int const_number = 0, last_const_number = 0;
	int dayCode = 0x0103FF40;
	int nightCode = 0x00010140;
	int dayNightFade = 40000;

  	if ((myData = malloc (MAX_SIZE)) == NULL)
	  {
    		fprintf (stderr, "Unable to allocate buffer: %s\n", strerror (errno)) ;
    		exit (EXIT_FAILURE) ;
  	  }
	spiSetup(1000000);
	t = 0;
	while ((machineState & 0xFF) > 0) {
		delay(10);
		stateCtr++;
		//printf("%lX\n", machineState);
		bg_brightness = (machineState & 0x00007F00) >> 8;
		switch ((machineState & 0xF0)>> 4) {
			case 0x1:
				//  Standard Mode
				brightness =    (machineState & 0x00FF0000) >> 16;
				fg_brightness = (machineState & 0xFF000000) >> 24;
				ctr = 0;
  				for (j = 0; j < STAR_HUBS; j++) {
   					header_bits = (WRITE_COMMAND << 26) | (CONTROL_BITS << 21) | (bg_brightness << 14) | (bg_brightness << 7) | bg_brightness;
					myData[0+j*28] =  (header_bits & 0xFF000000) >> 24;
   					myData[1+j*28] =  (header_bits & 0x00FF0000) >> 16;
   					myData[2+j*28] =  (header_bits & 0x0000FF00) >> 8;
   					myData[3+j*28] =  (header_bits & 0x000000FF);
					for (i = 0; i < 12; i++) {
						b = (int) pow(2,(double) 1+brightness + brightness*sin (2 * PI * t / 1000 + (double) (ctr % 10) * PI * 2 / 10 ))-1;
						//b = (int) pow(2,(double) brightness + (double) StarsBrightness[ctr] / 5.0 * (double) fg_brightness);
						//b = (int) pow(2,(double) brightness + (double) StarsBrightness[ctr-indexOffset]);
						//b = (int) pow(2,(double) brightness + (double) StarsBrightness[ctr-indexOffset]);
					        myData[i*2+5+j*28] = (b & 0x00FF);
					        myData[i*2+4+j*28] = (b & 0xFF00) >> 8;
					        ctr++;
   					}
  				}
  				size = 28 * STAR_HUBS;
  				if (wiringPiSPIDataRW (SPI_CHAN, myData, size) == -1) {
				        printf("Fail!\n");
			  	}
				break;
			case 0x2:
				//  Constellation Mode
				fg_brightness = (machineState & 0xFF000000) >> 24;
				brightness = (machineState & 0x0000FF0000) >> 16;
				const_b1 = (float) brightness / 16.0;
				if (stateCtr < 100) {
					const_b2 = const_b1 + (float) fg_brightness / 16.0 / (100.0 / (float) stateCtr);
					const_b3 = const_b1 + (float) fg_brightness / 16.0 / (100.0 / (100.0 - (float) stateCtr));
				} else {
					const_b2 = const_b1 + (float) fg_brightness / 16.0;
					const_b3 = const_b1;
					last_const_number = const_number;
				}
				const_number = (machineState & 0xF);
		    		//fprintf (stderr, "Constellation: %d\t%d\n", const_number,stateCtr) ;
				ctr = 1;
  				for (j = 0; j < STAR_HUBS; j++) {
   					header_bits = (WRITE_COMMAND << 26) | (CONTROL_BITS << 21) | (bg_brightness << 14) | (bg_brightness << 7) | bg_brightness;
					myData[0+j*28] =  (header_bits & 0xFF000000) >> 24;
   					myData[1+j*28] =  (header_bits & 0x00FF0000) >> 16;
   					myData[2+j*28] =  (header_bits & 0x0000FF00) >> 8;
   					myData[3+j*28] =  (header_bits & 0x000000FF);
					for (i = 0; i < 12; i++) {
						b = (int) pow(2,const_b1);
						for (k = 0; k < constellations[last_const_number][0]; k++) { 
							if (ctr == constellations[last_const_number][k+1]) b = (int) pow(2,const_b3);
						}
						for (k = 0; k < constellations[const_number][0]; k++) { 
							if (ctr == constellations[const_number][k+1]) b = (int) pow(2,const_b2);
					    		//fprintf (stderr, "Star Finder:: %d\t%d\n", ctr,constellations[const_number][k+1]) ;
						}
					        myData[i*2+5+j*28] = (b & 0x00FF);
					        myData[i*2+4+j*28] = (b & 0xFF00) >> 8;
					        ctr++;
   					}
  				}
  				size = 28 * STAR_HUBS;
  				if (wiringPiSPIDataRW (SPI_CHAN, myData, size) == -1) {
				        printf("Fail!\n");
			  	}
				break;
			case 0x3:
				//  Rotating Constellation Mode
				fg_brightness = (machineState & 0xFF000000) >> 24;
				brightness = (machineState & 0x0000FF0000) >> 16;
				const_b1 = (float) brightness / 16.0;
				if (stateCtr == 1000) {
					stateCtr = 0;
					const_number = (machineState & 0xF);
					machineState = (machineState & 0xFFFFFFF0) | (const_number + 1);
				}
				if (stateCtr < 100) {
					const_b2 = const_b1 + (float) fg_brightness / 16.0 / (100.0 / (float) stateCtr);
					const_b3 = const_b1 + (float) fg_brightness / 16.0 / (100.0 / (100.0 - (float) stateCtr));
				} else {
					const_b2 = const_b1 + (float) fg_brightness / 16.0;
					const_b3 = const_b1;
					last_const_number = const_number;
				}
				const_number = (machineState & 0xF);
		    		//fprintf (stderr, "Constellation: %d\t%d\n", const_number,(int) stateCtr) ;
				ctr = 1;
  				for (j = 0; j < STAR_HUBS; j++) {
   					header_bits = (WRITE_COMMAND << 26) | (CONTROL_BITS << 21) | (bg_brightness << 14) | (bg_brightness << 7) | bg_brightness;
					myData[0+j*28] =  (header_bits & 0xFF000000) >> 24;
   					myData[1+j*28] =  (header_bits & 0x00FF0000) >> 16;
   					myData[2+j*28] =  (header_bits & 0x0000FF00) >> 8;
   					myData[3+j*28] =  (header_bits & 0x000000FF);
					for (i = 0; i < 12; i++) {
						b = (int) pow(2,const_b1);
						for (k = 0; k < constellations[last_const_number][0]; k++) { 
							if (ctr == constellations[last_const_number][k+1]) b = (int) pow(2,const_b3);
						}
						for (k = 0; k < constellations[const_number][0]; k++) { 
							if (ctr == constellations[const_number][k+1]) b = (int) pow(2,const_b2);
					    		//fprintf (stderr, "Star Finder:: %d\t%d\n", ctr,constellations[const_number][k+1]) ;
						}
					        myData[i*2+5+j*28] = (b & 0x00FF);
					        myData[i*2+4+j*28] = (b & 0xFF00) >> 8;
					        ctr++;
   					}
  				}
  				size = 28 * STAR_HUBS;
  				if (wiringPiSPIDataRW (SPI_CHAN, myData, size) == -1) {
				        printf("Fail!\n");
			  	}
				break;
			case 0xFF:
				//  Raindrop Mode
				raindropFrequency = (machineState & 0xFF000000) >> 24;
				brightness = (machineState & 0x7F0000) >> 16;
				if (stateCtr == 1) { 			// initialize control array
					for (j = 0; j < 10; j++) {
						raindrops[j][0] = 0;
						raindrops[j][1] = 0;
						raindrops[j][2] = 0;
						raindrops[j][3] = 0;
					}
				}
				if (rand() % (raindropFrequency * 1000) == 0) {  // add raindrop to queue
					raindrops[raindropCounter][0] = (double) ((rand() % 300) + 200);
					raindrops[raindropCounter][1] = (double) ((rand() % 300) + 200);
					raindrops[raindropCounter][2] = (double) 1;
					raindrops[raindropCounter][3] = (double) stateCtr;
					for (j = 0; j < 180; j++) {		// Setup radial positions with respect to centre
						raindrops[raindropCounter][4+j] = sqrt((StarsX[j] - raindrops[raindropCounter][0])*(StarsX[j] - raindrops[raindropCounter][0]) + (StarsY[j] - raindrops[raindropCounter][1])*(StarsY[j] - raindrops[raindropCounter][1]));
					}
					raindropCounter++;
				}
				for (j = 0; j < 10; j++) {			//  Check for expired drops
					if ((stateCtr - raindrops[j][3]) > 1000) {
						raindrops[j][0] = 0;
						raindrops[j][1] = 0;
						raindrops[j][2] = 0;
						raindrops[j][3] = 0;
					}
				}
  				for (j = 0; j < STAR_HUBS; j++) {
   					header_bits = (WRITE_COMMAND << 26) | (CONTROL_BITS << 21) | (bg_brightness << 14) | (bg_brightness << 7) | bg_brightness;
					myData[0+j*28] =  (header_bits & 0xFF000000) >> 24;
   					myData[1+j*28] =  (header_bits & 0x00FF0000) >> 16;
   					myData[2+j*28] =  (header_bits & 0x0000FF00) >> 8;
   					myData[3+j*28] =  (header_bits & 0x000000FF);
					for (i = 0; i < 12; i++) {
        					b = (int) pow(2,(double) 8+brightness + brightness*sin (2 * PI * t / 1000 + (double) (ctr % 5) * PI * 2 / 5 ))-1;
						for (j = 0; j < 10; j++) {			//  Loop over raindrops
							if (raindrops[j][0] != 0) {
								starRadius = (double) ((stateCtr - raindrops[j][3]) / 10.0);
								b += 40000*pow(2,-1*(starRadius - raindrops[j][ctr+4-indexOffset])*(starRadius - raindrops[j][ctr+4-indexOffset])/2/raindrops[j][2]/raindrops[j][2]);
							}
						}

					        myData[i*2+5+j*28] = (b & 0x00FF);
					        myData[i*2+4+j*28] = (b & 0xFF00) >> 8;
					        ctr++;
   					}
  				}
  				size = 28 * STAR_HUBS;
  				if (wiringPiSPIDataRW (SPI_CHAN, myData, size) == -1) {
				        printf("Fail!\n");
			  	}
				break;
			case 0x4:
				//  Dance  Mode
				t = t + 10;
				delay(10);
				brightness = (machineState & 0x7F0000) >> 16;
  				for (j = 0; j < STAR_HUBS; j++) {
   					header_bits = (WRITE_COMMAND << 26) | (CONTROL_BITS << 21) | (bg_brightness << 14) | (bg_brightness << 7) | bg_brightness;
					myData[0+j*28] =  (header_bits & 0xFF000000) >> 24;
   					myData[1+j*28] =  (header_bits & 0x00FF0000) >> 16;
   					myData[2+j*28] =  (header_bits & 0x0000FF00) >> 8;
   					myData[3+j*28] =  (header_bits & 0x000000FF);
					for (i = 0; i < 12; i++) {
        					b = (int) pow(2,(double) 8+brightness + brightness*sin (2 * PI * t / 1000 + (double) (ctr % 5) * PI * 2 / 5 ))-1;
					        myData[i*2+5+j*28] = (b & 0x00FF);
					        myData[i*2+4+j*28] = (b & 0xFF00) >> 8;
					        ctr++;
   					}
  				}
  				size = 28 * STAR_HUBS;
  				if (wiringPiSPIDataRW (SPI_CHAN, myData, size) == -1) {
				        printf("Fail!\n");
			  	}
				break;
			case 0x5:
				//  Dance  Mode, shifting brightness
				//  Will interpolate all fields
				t = t + 10;
				delay(10);
				if (stateCtr > dayNightFade) {
					brightness = (nightCode & 0x7F0000) >> 16;
					bg_brightness = (nightCode & 0x00007F00) >> 8;
				} else {
					brightness =    ((dayCode & 0x7F0000) >> 16) - (int) ((double) (((dayCode & 0x7F0000) >> 16) - ((nightCode & 0x7F0000) >> 16)) * (log((double) stateCtr) / log((double) dayNightFade)));
					bg_brightness = ((dayCode & 0x007F00) >> 8) - (int) ((double) (((dayCode & 0x007F00) >> 8) - ((nightCode & 0x007F00) >> 8)) * (log((double) stateCtr) / log((double) dayNightFade)));
					//fprintf(stderr,"%l\t%l\t%l\t%l\n",stateCtr,dayNightFade,brightness,bg_brightness);
					//fprintf (stderr, "Dusking: %d\t%d\t%d\n", brightness,bg_brightness,(int) stateCtr) ;
				}
  				for (j = 0; j < STAR_HUBS; j++) {
   					header_bits = (WRITE_COMMAND << 26) | (CONTROL_BITS << 21) | (bg_brightness << 14) | (bg_brightness << 7) | bg_brightness;
					myData[0+j*28] =  (header_bits & 0xFF000000) >> 24;
   					myData[1+j*28] =  (header_bits & 0x00FF0000) >> 16;
   					myData[2+j*28] =  (header_bits & 0x0000FF00) >> 8;
   					myData[3+j*28] =  (header_bits & 0x000000FF);
					for (i = 0; i < 12; i++) {
        					b = (int) pow(2,(double) 8+brightness + brightness*sin (2 * PI * t / 1000 + (double) (ctr % 5) * PI * 2 / 5 ))-1;
					        myData[i*2+5+j*28] = (b & 0x00FF);
					        myData[i*2+4+j*28] = (b & 0xFF00) >> 8;
					        ctr++;
   					}
  				}
  				size = 28 * STAR_HUBS;
  				if (wiringPiSPIDataRW (SPI_CHAN, myData, size) == -1) {
				        printf("Fail!\n");
			  	}
				break;
		}
	}
	return 0;
}
