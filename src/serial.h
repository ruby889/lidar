#ifndef SERIAL_H
#define SERIAL_H

#ifdef WIN32
    #include "windows.h"
#endif

// data type
#ifndef BYTE_DEFINE
#define BYTE_DEFINE
    typedef unsigned char       BYTE;
#endif

#ifndef DWORD_DEFINE
#define DWORD_DEFINE
    typedef unsigned long       DWORD;
#endif

class Serial{
    public:
        int serial_open(const char com_id[], int baud_rate=9600, int byte_size=8, int parity=0, int stop_bits=1);
        int serial_write(BYTE write_buffer[]);
        int serial_close();
    private:
        // int serial_port_;
        #ifdef WIN32
            HANDLE hCom_;
        #else
            int hCom_;
        #endif
        DWORD read_buffer_size_ = 10;
        DWORD write_buffer_size_ = 10;
};

#endif
