
#include "serial_com.h"


void test_write(CSerialCom& s); 
void test_read(CSerialCom& s); 

const int N = 10; 
const char stop_sign = '#';

int main(int argc, char* argv[])
{
  CSerialCom serial; 
  if(!serial.open_serial("/dev/ttyS4"))
  {
    printf("failed to open serial port"); 
    return -1;
  }

  // test_read(serial); 
  test_write(serial);

  return 0; 
}

void test_write(CSerialCom& s)
{
  float buf[N]; 
  printf("send %d floats: \n", N); 
  for(int i=0; i<N; i++)
  {
    buf[i] = (i+1)*2.4; 
    printf("%f ", buf[i]);
  }
  
  int Len = sizeof(float)*N + 1;
  char* tbuf = new char[Len]; 
  strcpy(tbuf, (char*)buf, Len-1); 
  tbuf[Len-1] = stop_sign; 

  if(s.send(tbuf, Len))
  {
    printf("succeed send %d floats\n", N); 
  }else
  {
    printf("failed send floats!\n");
  }

  delete []tbuf; 
}

void test_read(CSerialCom& s)
{
  float buf[N] = {0};
  int Len = sizeof(float)*N + 1; 
  if(s.recv((char*)buf, Len, stop_sign))
  {
    printf("succeed read %d floats: \n", N);
    for(int i=0; i<N; i++)
      printf("%f ", buf[i]); 
    printf("\n");
  }else
  {
    printf("failed to read %d floats: \n", N); 
  }
  return ;
}



