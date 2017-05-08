#ifndef BIT_CHAR_H
#define BIT_CHAR_H

#include <sstream>
#include <fstream>
#include <string>
#include <stdlib.h>


class bitChar{
  public:
    unsigned char* c;
    int shift_count;
    std::string BITS;

    bitChar();
    void setBITS(std::string _X);
    int insertBits(std::ofstream& outf);
    std::string getBits(unsigned char _X);
    void writeBits(std::ofstream& outf);
    ~bitChar();
};

#endif
