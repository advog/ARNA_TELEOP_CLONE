#include "joystick_listener.hpp"
#include <cstring>

/*
** listener thread
*/
struct listener_args {
    int fd;
    std::atomic<joy_state_vector>* state;
    std::atomic_int* connected;
};

void* joystick_listener_thread(void* args) {
    
    listener_args l_args = *(listener_args*)args;
    free(args);

    int fd = l_args.fd;
    std::atomic<joy_state_vector>* state = l_args.state;
    std::atomic_int* connected = l_args.connected;
    
    joy_state_vector local_state = state->load();
    js_event event;
    int ret;
    while(1){
        
        //wait until data
        //TODO-med find out if partial read of an event breaks this
        ret = read(fd, &event, sizeof(event));

        //disconnected
        if(ret <= 0){
            //read error or fd close
            connected->store(joy_stat_disconnected);
            return NULL;
        }

        //edit local_state and store it in state
        if(event.type == 1){
            local_state.buttons[event.number] = event.value;
        }
        else if(event.type == 2){
            local_state.axis[event.number] = event.value;
        }

        state->store(local_state);
    }
}

/*
** Constructor-Destructor
*/
joystick_listener::joystick_listener() {
    connected.store(joy_stat_disconnected);
    state.store(joy_state_vector{});
    fd = -1;
}

joystick_listener::~joystick_listener() {
    if(status() == joy_stat_connected){
	    disconnect();
    }
}

int joystick_listener::connect() {
    
    //set status to disconnected
    connected.store(joy_stat_disconnected);

    //open joystick fd
    fd = open("/dev/input/js0", O_RDONLY);
    if(fd <= 0){ 
        std::cout << "failed to open" << std::endl;
        return joy_stat_disconnected;}
    
    listener_args* l_args = new listener_args;
    l_args->fd = fd;
    l_args->state = &state;
    l_args->connected = &connected;
    pthread_create(&listener_pt, NULL, joystick_listener_thread, (void*)l_args);
    
    connected.store(joy_stat_connected);

    return connected.load();
}

int joystick_listener::disconnect() {
    close(fd);
    pthread_join(listener_pt, NULL);
    return joystick_status::joy_stat_disconnected;
}

int joystick_listener::get_state(joy_state_vector &ret) {
    if(connected.load() == joy_stat_connected){
	    ret = state.load();
      	return joy_stat_connected;
    }
    else{
	    return joy_stat_disconnected;
    }
}

int joystick_listener::status(){
    return connected.load();
}
