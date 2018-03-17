/********************************************
* mpu6050.c : How to interface with the MPU6050
* Gyro/Accleromter module from Deal Extreme
* http://www.dx.com/p/mpu6050-serial-6-axis-accelerometer-gyroscope-module-kalman-filtering-angle-output-for-arduino-414210
*
* (c) 2016 Mike Field <hamster@snap.net.nz>
********************************************/

/* Standard headers */
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <termios.h>

/************************
* Error counters
************************/
static unsigned int imuErrorsTimeout    = 0;
static unsigned int imuErrorsChksum     = 0;
static unsigned int imuErrorsPacketType = 0;

/***********************************************
* Decode the acceleration packet
***********************************************/
void processAcceleration(unsigned char *buffer) {
  float x,y,z;
  short int i;
  struct timeval t;
  gettimeofday(&t,NULL);

  /* Full scale is +/- 16 G ( 1 G = 9.8 m/s) */
  i = (buffer[3] << 8) + buffer[2]; x = i/(32768.0/16/9.8);
  i = (buffer[5] << 8) + buffer[4]; y = i/(32768.0/16/9.8);
  i = (buffer[7] << 8) + buffer[6]; z = i/(32768.0/16/9.8);
#if 1
  printf("%i.%06i Acceleration %7.2f %7.2f %7.2f  (%7.2f)\n",
         (int)t.tv_sec, (int)t.tv_usec,
          x,y,z, sqrt(x*x+y*y+z*z));

#endif
}

/***********************************************
* Decode the rotation rate packet
***********************************************/
void processRotation(unsigned char *buffer) {
  float x,y,z;
  short int i;
  struct timeval t;
  gettimeofday(&t,NULL);

  /* Full scale is 2000 degrees per second */
  i = (buffer[3] << 8)+buffer[2]; x = i/(32768.0/2000);
  i = (buffer[5] << 8)+buffer[4]; y = i/(32768.0/2000);
  i = (buffer[7] << 8)+buffer[6]; z = i/(32768.0/2000);
#if 1
  printf("%i.%06i Rotation     %7.2f %7.2f %7.2f\n",
         (int)t.tv_sec, (int)t.tv_usec,
         x, y, z);
#endif
}

/***********************************************
* Decode the attitude rate packet
***********************************************/
void processAttitude(unsigned char *buffer) {
  float roll, pitch, yaw;
  short int i;
  struct timeval t;
  gettimeofday(&t,NULL);

  /* Full scale is +/- 180 degrees per second */
  i = (buffer[3] << 8)+buffer[2]; roll  = i/(32768.0/180);
  i = (buffer[5] << 8)+buffer[4]; pitch = i/(32768.0/180);
  i = (buffer[7] << 8)+buffer[6]; yaw   = i/(32768.0/180);

#if 1
  printf("%i.%06i Attitude     %7.2f %7.2f %7.2f\n",
         (int)t.tv_sec, (int)t.tv_usec,
         roll, pitch, yaw);
#endif
}

/***********************************************
* Process the next character from the sensor
***********************************************/
void processIMUchar(unsigned char c) {
  static unsigned char buffer[11];
  static unsigned char holdoff = 0;
  static unsigned char inSync = 0;

  unsigned char cksum = 0;
  int i;

  /****************************
  * Move the buffer forward and
  * insert the new character
  ****************************/
  for(i = 0; i < 10; i++) {
    buffer[i] = buffer[i+1];
  }
  buffer[10] = c;

  /***************************************
  * Are we sure that we will not be at the
  * end of packet? If so, just return
  ***************************************/
  if(holdoff > 0)
  {
    holdoff--;
    return;
  }

  /*********************
  * Compute the checksum
  *********************/
  cksum = 0;
  for(i = 0; i < 10; i++) {
    cksum += buffer[i];
  }

  /* If we have an invalid checksum return */
  if(cksum != c) {
    if(inSync) {
       imuErrorsChksum++;
       inSync = 0;
    }
    return;
  }

  /* Packets start with 0x55 */
  if(buffer[0] != 0x55) {
    imuErrorsPacketType++;
    return;
  }

  /* The next byte is the packet type */
  switch(buffer[1]) {
    case 0x51:
        processAcceleration(buffer);
        inSync = 1;
        holdoff = 10;
        return;
    case 0x52:
        processRotation(buffer);
        inSync = 1;
        holdoff = 10;
        return;
    case 0x53:
        processAttitude(buffer);
        inSync = 1;
        holdoff = 10;
        return;
  }

  /* Unknown packet type is an error */
  imuErrorsPacketType++;
}

/******************************************
* Configure the serial connection to just
* pass through the raw data as it arrives,
* and to run at 115200 baud
******************************************/
int configureSerialPort(int f) {
  int flags;
  struct termios cf;

  /* Set the file into non-blocking mode */
  flags = fcntl(f, F_GETFL, 0);
  fcntl(f, F_SETFL, flags | O_NONBLOCK);

  if(tcgetattr(f, &cf) == -1) {
    printf("Unable to get termios details\n");
    return 0;
  }

  if(cfsetispeed(&cf, B115200) == -1 || cfsetospeed(&cf, B115200) == -1) {
    printf("Unable to set speed\n");
    return 0;
  }

  /* Make it a raw stream and turn off software flow control */
  cfmakeraw(&cf);
  cf.c_iflag &= ~(IXON | IXOFF | IXANY);

  if(tcsetattr(f, TCSANOW, &cf) == -1) {
    printf("Unable to set termios details\n");
    return 0;
  }
  return 1;
}

/************************************
* Main program for gyro/accelerometer 
* sensor data processing
************************************/
int main(int argc, char *argv[]) {
  int f;
  static unsigned char backToBackTimeouts = 0;

  if(argc != 2) {
     printf("Must supply file name (eg /dev/ttyAMA0)\n");
     return 0;
  }

  printf("Opening port: %s\n", argv[1]);

  f  = open(argv[1], O_RDONLY);
  if(f == -1) {
     printf("Unable to open file\n");
     return 0;
  }

  if(!configureSerialPort(f)) {
    printf("Can't configure serial port\n");
    return 0;
  }

  while(1) {
      fd_set readfds;
      struct timeval timeout;
      int status;

      /* Set up to wait up to 0.2 secs for data */
      timeout.tv_sec  = 0;
      timeout.tv_usec = 200000;
      FD_ZERO(&readfds);
      FD_SET(f, &readfds);

      /* Try to read from the device */
      status = select(f+1, &readfds, NULL, NULL, &timeout);
      if(status < 0) {
        /* Error while reading! */
        break;
      }

      /* If that failed, then timeout */
      if(status == 0) {
        imuErrorsTimeout++;
        if(backToBackTimeouts == 10)
          break; /* Stop on 10 timeouts */
        backToBackTimeouts++;
      } else {
        /* Process data */
        backToBackTimeouts = 0;

        /* There can only be waiting I/O on the
         * device, but check anyway */
        if(FD_ISSET(f, &readfds)) {
          /* We are in non-blocking mode
           * so we can read more than one
           * character at a time */
          unsigned char buffer[16];
          int n;
          n = read(f,buffer,sizeof(buffer));
          if(n > 0) {
            int i;
            for(i = 0; i < n; i++) {
              processIMUchar(buffer[i]);
            }
          }
        }
      }
  }

  /* Get here if there is a timeout */
  close(f);

  printf("Timout errors %i, chksum errors %i, header errors %i\n",imuErrorsTimeout, imuErrorsChksum, imuErrorsPacketType);
} 
