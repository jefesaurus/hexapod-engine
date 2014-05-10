#include "maestro.h"
#include <stdio.h>

#define OPEN_ERROR -1
#define WRITE_ERROR -2
#define READ_ERROR -3
#define VALUE_ERROR -4

int set_target(Maestro* device, unsigned char channel_num, unsigned short target) {
	if (target < device->global_min || target > device->global_max) {
    printf("Target out of range: %d", target);
    return VALUE_ERROR;
  }

	unsigned char command[4] = { 0x84, channel_num, target & 0x7F, (target >> 7) & 0x7F };
	if (!write_bytes(&device->port, command, sizeof(command))) {
    return WRITE_ERROR;
  }
	return 0;
}
	
int set_speed(Maestro* device,  unsigned char channel_num, unsigned short speed ) {
	unsigned char command[4] = { 0x87, channel_num, speed & 0x7F, (speed >> 7) & 0x7F };
	if (!write_bytes(&device->port, command, sizeof(command))) {
		return WRITE_ERROR;
  }
	return 0;
}

int set_acceleration(Maestro* device, unsigned char channel_num, unsigned char acceleration) {
	unsigned short accelerationAsShort = acceleration;
	unsigned char command[4] = { 0x89, channel_num, accelerationAsShort & 0x7F, (accelerationAsShort >> 7) & 0x7F };
	if (!write_bytes(&device->port, command, sizeof(command))) {
		return WRITE_ERROR;
  }
	return 0;
}

int get_position(Maestro* device,  unsigned char channel_num) {
	unsigned char command[2] = { 0x90, channel_num };
	if (!write_bytes(&device->port, command, sizeof(command))) {
    return WRITE_ERROR;
  }
	unsigned char response[2] = { 0x00, 0x00 };
	if (!read_bytes(&device->port, response, sizeof(response))) {
    return READ_ERROR;
  }
	return response[0] + 256*response[1];
}

// 1 means servos are still moving
int get_moving_state(Maestro* device) {
	unsigned char command = 0x93;
	if (!write_bytes(&device->port, &command, sizeof(command))) {
    return WRITE_ERROR;
  }
	unsigned char response = 0x00;
	if (!read_bytes(&device->port, &response, sizeof(response))) {
    return READ_ERROR;
  }
	return (response == 0x01);
}

int get_errors(Maestro* device) {
	unsigned char command = 0xA1;
	if (!write_bytes(&device->port, &command, sizeof(command))) {
    return WRITE_ERROR;
  }
	unsigned char response[2] = { 0x00, 0x00 };
	if (!read_bytes(&device->port, response, sizeof(response))) {
    return READ_ERROR;
  }
	return (response[0] & 0x7F) + 256 * (response[1] & 0x7F);	// Need to check this code on real errors!
}

int go_home(Maestro* device) {
	unsigned char command = 0xA2;
	if (!write_bytes(&device->port, &command, sizeof(command))) {
    return WRITE_ERROR;
  }
  return 0;
}
