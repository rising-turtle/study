#ifndef SERVER_DATA_DEALER_H
#define SERVER_DATA_DEALER_H

// #include "opencv/cv.h"

class CServerDealer
{
public:
    CServerDealer();
    ~CServerDealer();
    bool handleODO(char* buf, unsigned int len, char* path_);
    bool handleXtion(char* buf, unsigned int len, char* path_);
    bool handleSICK(char* buf, unsigned int len, char* path_);
}; 

#endif
