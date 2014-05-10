#include "serialport.h"
typedef struct {
  SerialPort port;
  int num_channels;
  int global_min;
  int global_max;
} Maestro;

// The target is given in units of 0.25탎 
int set_target(Maestro* device, unsigned char channel_num, unsigned short target);

// The speed limit is given in units of (0.25탎)/(10ms)
int set_speed(Maestro* device, unsigned char channel_num, unsigned short speed );

// The acceleration limit is a value from 0 to 255 in units of (0.25탎)/(10ms)/(80ms)	
int set_acceleration(Maestro* device, unsigned char channel_num, unsigned char acceleration);

// For a servo channel, the position is the current pulse width in units of 0.25탎
// For a digital output channel, a position less than 6000 means the line is low, and above 6000 it's high
// For an input channel, the position represents the voltage measure on the channel (see doc)
int get_position(Maestro* device, unsigned char channel_num);
int get_moving_state(Maestro* device);
int get_errors(Maestro* device);

// The "go home" action set the channels to their startup/error state.
// This state is defined on a per-channel. It can either be:
// - ignore: the value is unchanged when we do a "go home". PWM signal is continues to be generated
// - go to: the channel is set to the specified value. Again PWM signal is generated
// - off: the channel is turned off. There's no more PWM signal generated for the channel
int go_home(Maestro* device);
