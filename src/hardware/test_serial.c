#include "serialport.h"
#include <stdio.h>

int main(int argc, char** argv) {
  #ifdef DEBUG
  printf("Debug mode\n");
  #endif

  printf("Opening port...\n"); 
  SerialPort maestro;
  open_serial_port(&maestro, "/dev/ttyACM0");
  printf("Port open with fd: %d\n", maestro.file_descriptor); 
  unsigned const char data[] =     "012341234";
  printf("Sending %d bytes... ", (int)sizeof(data));
  bool result;
  result = write_bytes(&maestro, data, sizeof(data)); 
  if (result) {
    printf("Success\n");
  } else {
    printf("Failure\n");
  }
  printf("Closing port...\n"); 
  close_serial_port(&maestro);
  printf("Port closed.\n"); 
  return 0;
}
