#include <cstddef>
#include <cstdint>
#include <cstdlib>  
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <signal.h>
#include <vector>
#include <pthread.h>

#include <iostream>
#include <atomic>

const int joy_max_axis = 8;
const int joy_max_buttons = 32;

struct joy_state_vector {
    int16_t axis[joy_max_axis];
    int8_t buttons[joy_max_buttons];
};

enum joystick_status: int {
    joy_stat_connected = 1,
    joy_stat_disconnected = 0
};

class joystick_listener {
public:
    std::atomic<joy_state_vector> state;
    std::atomic_int connected;
    int fd;
    pthread_t listener_pt;

    joystick_listener();
    ~joystick_listener();

    int get_state(joy_state_vector& ret);
    int connect();
    int disconnect();
    int status();
    
};
