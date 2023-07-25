#include "serial.h"
#include <stdio.h>
#include <cstring>
#ifdef WIN32
    #include "windows.h"
#else
    #include <fcntl.h>   // Contains file controls like O_RDWR
    #include <errno.h>   // Error integer and strerror() function
    #include <termios.h> // Contains POSIX terminal control definitions
    #include <unistd.h>  // write(), read(), close()
#endif

#include <climits>

int Serial::serial_open(const char com_id[], int baud_rate, int byte_size, int parity, int stop_bits){
#ifdef WIN32
    hCom_ = CreateFile((LPCSTR)com_id,
                      GENERIC_READ | GENERIC_WRITE, // open mode: read & write  
                      0, //独占方式  
                      NULL,
                      OPEN_EXISTING, //打开而不是创建  
                      0, //同步方式  
                      NULL);
    if (hCom_ == (HANDLE)-1) {
      	printf("Error: open serial port fail!");
        return -1;
    }

    // set buffer size
    SetupComm(hCom_, read_buffer_size_, write_buffer_size_);

    // set timeout
    COMMTIMEOUTS TimeOuts; 
    TimeOuts.ReadIntervalTimeout = 1000;
    TimeOuts.ReadTotalTimeoutMultiplier = 0;
    TimeOuts.ReadTotalTimeoutConstant = 0;
    TimeOuts.WriteTotalTimeoutMultiplier = 0;
    TimeOuts.WriteTotalTimeoutConstant = 500;
    SetCommTimeouts(hCom_, &TimeOuts); 

    DCB dcb;
    GetCommState(hCom_, &dcb);
    dcb.BaudRate = baud_rate;
    switch (byte_size) {
        case 8: dcb.ByteSize = byte_size; break;
    }
    switch(parity) {
        case 0: dcb.Parity = NOPARITY; break;
        default: dcb.Parity = NOPARITY;
    }
    switch(stop_bits) {
        case 1: dcb.StopBits = ONESTOPBIT; break;
        case 2: dcb.StopBits = TWOSTOPBITS; break;
        default: dcb.StopBits = ONESTOPBIT;
    }
    if (!SetCommState(hCom_, &dcb))
        return -2;

    // clear the cache
    PurgeComm(hCom_, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
#else
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  hCom_ = open(com_id, O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(hCom_, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return -3;
  }

  switch(parity) {
      case 0: tty.c_cflag &= ~PARENB; break;
      default: tty.c_cflag &= ~PARENB;
  }
  switch(stop_bits) {
      case 1: tty.c_cflag &= ~CSTOPB; break;
      case 2: tty.c_cflag |= CSTOPB; break;
      default: tty.c_cflag &= ~CSTOPB;
  }    
  tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size 
  switch (byte_size) {
      case 8: tty.c_cflag |= CS8; break;
  }
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  // Save tty settings, also checking for error
  if (tcsetattr(hCom_, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return -4;
  }
#endif
  return 0;
}

int Serial::serial_write(BYTE write_buffer[]){
#ifdef WIN32
    COMSTAT ComStat;
    DWORD dwErrorFlags;
    ClearCommError(hCom_, &dwErrorFlags, &ComStat);
#endif
    
#ifdef WIN32
    if (!WriteFile(
#else
    if (!write(
#endif
        hCom_, write_buffer, write_buffer_size_
#ifdef WIN32
        , &write_buffer_size_, NULL
#endif
    )) {
        printf("serial write failed!\n");
        return -1;
    } else {
        return 0;
    }
}


int Serial::serial_close(){
#ifdef WIN32
    return CloseHandle(hCom_);
#else
    return close(hCom_);
#endif
}
