#include "serialport.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>  

/* The port name can be:
	- Linux : "/dev/ttyACM0"
	- Mac OS X : "/dev/cu.usbmodem00034567"
*/
void open_serial_port(SerialPort* port, const char* port_name) {
  port->file_descriptor = open(port_name, O_RDWR | O_NOCTTY);
  if (port->file_descriptor == -1) {
    perror(port_name);
  }
}

void close_serial_port(SerialPort* port) {
  if (port->file_descriptor != -1) {
    close(port->file_descriptor);
  }
  port->file_descriptor = -1;
}

int open_port(const char* port_name) {
  int fd = open(port_name, O_RDWR | O_NOCTTY);
  if (fd == -1) {
    perror(port_name);
    return -1;
  }
  return fd;
}

bool write_bytes(SerialPort* port, const unsigned char* data, unsigned int data_size_bytes) {
	if (port->file_descriptor == -1) {
		return false;
  }

	// See http://linux.die.net/man/2/write
	int ret = write(port->file_descriptor, data, data_size_bytes);
	if (ret == -1) {
		printf("Error writing. errno=%d\n", errno);
		return false;
	} else if (ret != data_size_bytes) {
		printf("Error writing. Wrote %d bytes instead of %d\n", ret, data_size_bytes);
		return false;
	}

	return true;
}

bool read_bytes(SerialPort* port, unsigned char* data, unsigned int data_size_bytes) {
	if (port->file_descriptor == -1) {
		return false;
  }

	// See http://linux.die.net/man/2/read
	int ret = read(port->file_descriptor, data, data_size_bytes);
	if (ret == -1) {
		printf("Error reading. errno=%d\n", errno);
		return false;
	} else if (ret != data_size_bytes) {
		printf("Error reading. Read %d bytes instead of %d\n", ret, data_size_bytes);
		return false;
	}
	return true;
}
