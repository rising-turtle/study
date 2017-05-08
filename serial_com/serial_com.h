/*
 *  Sep. 7th, 2016 David Z
 *
 *  send data through serial com port 
 *
 * */

#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>

class CSerialCom 
{
  public:
    CSerialCom(); 
    ~CSerialCom(); 

    bool open_serial(char* portname = "/dev/ttyS0"); 
    bool send(char* buf, int len); 
    bool recv(char* buf, int len, char stop_char='#');
    void close_serial();

    int m_fd; 
    struct termios tio; 
    struct termios old_state;
    
};





#endif
