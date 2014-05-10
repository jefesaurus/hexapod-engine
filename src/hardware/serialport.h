#include <stdbool.h>

typedef struct{
  int file_descriptor;
} SerialPort;

void open_serial_port(SerialPort* port, const char* port_name);
void close_serial_port(SerialPort* port);
bool write_bytes(SerialPort* port, const unsigned char* data, unsigned int data_size_bytes);
bool read_bytes(SerialPort* port, unsigned char* data, unsigned int data_size_bytes);
