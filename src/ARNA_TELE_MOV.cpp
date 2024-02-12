#include <cstddef>
#include <cstdlib>
#include <gtkmm-3.0/gtkmm.h>
#include <gdkmm-3.0/gdkmm.h>

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>
#include <cstdint>
#include <string>
#include <cmath>
#include <fstream>

#include "gtk/gtkcssprovider.h"
#include "json.hpp"
#include "rosbridge_client.hpp"
#include "joystick_listener.hpp"
#include "ARNA_THEORA.hpp"

//
// consts
//

//default entry values
const std::string default_stream_topic = "/image_raw/theora";
std::string default_ros_address = "localhost";
std::string default_ros_port = "9090";

//default topics
const std::string move_vec_topic = "/ARNA_TELEOP_MOV";
const std::string com_topic = "/ARNA_TELEOP_COM";
const std::string depth_topic = "/ARNA_TELEOP_DEPTH";

const int home_command_action = 2;

//
// func declarations
//

//spinners
gint spin_ROS(gpointer data);
std::vector<float> poll_joystick();
std::vector<float> poll_buttons();
gint publish_move_vec(gpointer data);

//connections
int ros_connect(std::string address, int port);
int joystick_connect();

//gui manipulation
void set_ros_label_status(int status);

//gui event callbacks
void press_movement_button(GtkWidget *widget, gpointer data);
void release_movement_button(GtkWidget *widget, gpointer data);
void click_joystick_connect_button(GtkWidget *widget, gpointer data);
void click_ros_connect_button(GtkWidget *widget, gpointer data);
void click_load_joy_config_button(GtkWidget* widget, gpointer data);

//gui management
void quit_app(gpointer data);
void setup_GUI(int argc, char **argv);

//
// data structures
//

//maps axis+activation button -> mov_vector index+gain
struct axis_mapping{
    int active_mode;
    int joy_axis;
    int mov_vector_index;
    float gain;
};

//stores configuration information about joystick
class joy_config{
public:

    //gripper button
    int gripper_button;

    //deadzone
    int16_t deadzone;

    //vector of mappings from (mode, axis) -> mov_vector_index
    std::vector<axis_mapping> axis_map;

    joy_config(){
        clear();
    }

    void clear(){
        gripper_button = 0;
        deadzone = 0;
        axis_map.clear();
    }

    //loads joystick config from file
    int load_config(std::string path){
        
        std::ifstream f(path);
        nlohmann::json data = nlohmann::json::parse(f);

        clear();

        gripper_button = data["gripper_button"];
        deadzone = data["deadzone"];

        for(auto outer = data["mode_map"].begin(); outer != data["mode_map"].end(); ++outer) {
            nlohmann::json& mode_json = *outer;
            
            for (auto inner = mode_json["map"].begin(); inner != mode_json["map"].end(); ++inner) {
                nlohmann::json& mapping_json = *inner;

                axis_mapping tmp = {mode_json["button"], mapping_json["joy_axis"], mapping_json["mov_vector_index"], mapping_json["gain"]};
                axis_map.push_back(tmp);
            }

        }
        return 0;

    }

};

//keeps track of state of application
class app_state{
public:

    int ros_connected;
    int joy_connected;

    app_state(){
        ros_connected = 0;
        joy_connected = 0;
    }
};

//
// GTK widget pointers
//

//gtk general
GtkWidget* window;
GtkWidget* noot;

//base page
GtkWidget* base_layout;
std::vector<GtkWidget*> base_buttons(6);

//arm page
GtkWidget* arm_layout;
std::vector<GtkWidget*> arm_buttons(13);
GtkWidget* depth_label;

//ctrl page
GtkWidget* ctrl_grid;

GtkWidget* joy_connect_button; //joystick config
GtkWidget* joy_config_button;

GtkWidget* ros_connect_button; //ros connect buttons
GtkWidget* ros_connect_add_entry;
GtkWidget* ros_connect_port_entry;
GtkWidget* ros_status_label;

GtkWidget* home_arm_button;

//
// logical globals
//

//move vector {base, arm linear, arm angular, gripper}
//sent to the forwading node on ARNA ~60 times per second
std::vector<float> button_global_mov_vector(10, 0);

//maps from button -> mov_vector index + gain 
std::map<GtkWidget*, std::pair<int, float>> button_map;

//rosbridge networking
rosbridge_lib::rosbridge_client ros;

//joystick
joystick_listener joystick;
joy_config joy_conf;

//application state
app_state state;

//
// spinners
//

//spins ros once
gint spin_ROS(gpointer data){
    (void)data;

    if(!state.ros_connected){return 0;}

    state.ros_connected = (ros.spin_once() != rosbridge_lib::spret_disconnected);
    return state.ros_connected;

}

//deadzone + gain function for joystick values
float inline jhelp(float gain, int16_t val){ 
    
    if(std::abs(val) > joy_conf.deadzone){
        return std::copysign((std::abs(val) - joy_conf.deadzone) / 32768.0f, val) * gain; //magnitude * sign * gain
    }
    else{
        return 0;
    }

}

//polls joystick, returns move vector of joystick inputs
std::vector<float> poll_joystick(){

    static char gripper_prev(0);
    static float gripper_val(0.0);

    //create mov_vector
    std::vector<float> joy_mov_vector(10, 0);

    //get joystick state from listener
    joy_state_vector joy_state;
    state.joy_connected = joystick.get_state(joy_state);

    //update move vector
    if(state.joy_connected == joystick_status::joy_stat_connected){
        
        //get active mode
        int active_mode = -1;
        for (int i = 0; i < joy_max_buttons; i++) {
            if (joy_state.buttons[i] == 1) {
                active_mode = i;
            }
        }

        //enact active mappings
        for (size_t i = 0; i < joy_conf.axis_map.size(); i++) {
            axis_mapping tmp = joy_conf.axis_map[i];
            if(tmp.active_mode == active_mode){
                //mapping is active, use it
                joy_mov_vector[tmp.mov_vector_index] = jhelp(tmp.gain, joy_state.axis[tmp.joy_axis]);
            }
        }

        //gripper control
        char gripper_cur = joy_state.buttons[joy_conf.gripper_button];
        if(gripper_prev == 0 && gripper_cur == 1){
            gripper_val = gripper_val > 0.5 ? 0 : 0.8;
        }
        gripper_prev = joy_state.buttons[joy_conf.gripper_button];
        joy_mov_vector[9] = gripper_val;

    }

    return joy_mov_vector;

}

//polls buttons, doesn't do anything except copy button_mov_vec, just a placeholder
std::vector<float> poll_buttons(){
    return button_global_mov_vector;
}

//
// logic functions
//

void depth_callback(nlohmann::json j){
    std::stringstream ss;
    ss << "depth: " << j["msg"]["data"];
    gtk_label_set_text(GTK_LABEL(depth_label), ss.str().c_str());
}

//publish mov_vector
gint publish_move_vec(gpointer data){
    (void)data;

    //if ros disconnected, stop
    if(!state.ros_connected){return 0;}
    
    //
    std::vector<float> mov_vector(10, 0);

    //get mov vectors
    std::vector<float> joy_mov_vector = poll_joystick();
    std::vector<float> button_mov_vector = poll_buttons();

    //combine them somehow
    mov_vector = joy_mov_vector;
    for (size_t i = 0; i < mov_vector.size(); i++) {
        if (button_mov_vector[i] != 0) {
            mov_vector[i] = button_mov_vector[i];
        }
    }

    //send the combined move vector
    nlohmann::json j;
    j["data"] = mov_vector;
    ros.publish(move_vec_topic, j);
    
    state.ros_connected = ros.send_queue() != rosbridge_lib::send_return::seret_disconnected;
    set_ros_label_status(state.ros_connected);

    return state.ros_connected;

}

int send_command(int command){

    //send the combined move vector
    nlohmann::json j;
    j["data"] = command;
    ros.publish(com_topic, j);

    state.ros_connected = ros.send_queue() != rosbridge_lib::send_return::seret_disconnected;
    set_ros_label_status(state.ros_connected);

    return state.ros_connected;

}

//connects to rosbridge server, schedules spin every 3ms and movement transmit every 50ms
int ros_connect(std::string address, int port){

    // startup rosbridge_client
    ros.cleanup();
    state.ros_connected = ros.connect(port, address) == rosbridge_lib::conn_return::cnret_succes;
    if(!state.ros_connected){
	    ros.cleanup();
	    return 0;
    }

    // advertise movement vector topic
    ros.advertise(move_vec_topic, "std_msgs/Float32MultiArray");
    state.ros_connected = ros.send_queue() != rosbridge_lib::send_return::seret_disconnected;
    if(!state.ros_connected){
	    ros.cleanup();
	    return 0;
    }

    // advertise command topic
    ros.advertise(com_topic, "std_msgs/Int32");
    state.ros_connected = ros.send_queue() != rosbridge_lib::send_return::seret_disconnected;
    if(!state.ros_connected){
	    ros.cleanup();
	    return 0;
    }

    // subscribe to depth topic
    ros.subscribe(depth_topic, depth_callback);
    state.ros_connected = ros.send_queue() != rosbridge_lib::send_return::seret_disconnected;
    if(!state.ros_connected){
	    ros.cleanup();
	    return 0;
    }

    // add spinners
    // TODO-low maybe add pipe functionality to rosbridge_lib so that we dont have to spin?
    g_timeout_add(3, spin_ROS, NULL);
    g_timeout_add(50, publish_move_vec, NULL);

    return 1;

}

//connects joystick listener and schedules poll every 20ms
int joystick_connect(){

	joystick.disconnect();
    state.joy_connected = joystick.connect() == joystick_status::joy_stat_connected;
    if(!state.joy_connected){
        joystick.disconnect();
        return 0;
    }

    return 1;

}

//
// GUI manipulation funcs
//

//updates ros connection status label
void set_ros_label_status(int status){
    
    gchar* markup;
    if(status){
        markup = g_strdup_printf("<span foreground='green'>%s</span>","connected");
    }
    else{
        markup = g_strdup_printf("<span foreground='red'>%s</span>","disconnected");
    }
    gtk_label_set_markup(GTK_LABEL(ros_status_label), markup);
    g_free(markup);

}

//
// GUI event callbacks
//

// button callback - respond to move button pressed
void press_movement_button(GtkWidget *widget, gpointer data){
    (void)data;

    auto ret = button_map[widget];
    button_global_mov_vector[ret.first] = ret.second;

}

// button callback - reset movement vector
void release_movement_button(GtkWidget *widget, gpointer data){
    (void)data;
    (void)widget;

    std::fill(button_global_mov_vector.begin(), button_global_mov_vector.end(), 0);

}

// button callback - start joystick listener
void click_joystick_connect_button(GtkWidget *widget, gpointer data){
    (void)data;
    (void)widget;

    std::cout << "attempting to connect joystick: ";
    if(joystick_connect()){
        std::cout << "success" << std::endl;
    }
    else{
        std::cout << "failure" << std::endl;
    }

}

// button callback - connect to rosbridge server
void click_ros_connect_button(GtkWidget *widget, gpointer data){
    (void)data;
    (void)widget;
    
    // parse port
    char* port_str = (char*)gtk_entry_get_text(GTK_ENTRY(ros_connect_port_entry));
    char* p;
    int port = strtol ( port_str, &p, 10);
    if(*p != 0){
        std::cout << "invalid port" << std::endl;
        return;
    }

    //parse address
    std::string address = std::string((char*)gtk_entry_get_text(GTK_ENTRY(ros_connect_add_entry)));
    if(address.empty()){
        std::cout << "invalid address" << std::endl;
        return;
    }
    
    //connect to server
    std::cout << "attempting to connect to ros server: ";
    if(ros_connect(address, port)){
        std::cout << "success" << std::endl;
    }
    else{
        std::cout << "failure" << std::endl;
    }
    set_ros_label_status(state.ros_connected);

}

//button callback - homes arm
void click_home_arm_button(GtkWidget *widget, gpointer data){
    (void)widget;
    (void)data;

    send_command(home_command_action);
}

//load joystick config
void click_load_joy_config_button(GtkWidget* widget, gpointer data){
    (void)widget;
    (void)data;
    
    GtkWidget* dialog;
    GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_OPEN;
    gint res;

    dialog = gtk_file_chooser_dialog_new("Open File", GTK_WINDOW(window), action,("_Cancel"), GTK_RESPONSE_CANCEL, ("_Open"), GTK_RESPONSE_ACCEPT, NULL);

    res = gtk_dialog_run (GTK_DIALOG(dialog));
    if(res == GTK_RESPONSE_ACCEPT){
        char *filename;
        GtkFileChooser *chooser = GTK_FILE_CHOOSER(dialog);
        filename = gtk_file_chooser_get_filename(chooser);
        joy_conf.load_config(std::string(filename));
        g_free(filename);
    }

    gtk_widget_destroy (dialog);

}

// close window callback - quit application
void quit_app(gpointer data){
    (void)data;

    if(state.ros_connected){
        ros.cleanup();
    }
    gtk_main_quit();

}

//
// setup
//

void setup_GUI(int argc, char **argv){

    //init GTK
    gtk_init(&argc, &argv);
    
    //create window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(window), 600, 400);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);

    //load CSS
    auto display = gdk_display_get_default();
    auto screen = gdk_display_get_default_screen(display);
    auto css_provider = gtk_css_provider_new();
    gtk_style_context_add_provider_for_screen(screen, GTK_STYLE_PROVIDER(css_provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);    
    GError* g_err = NULL;
    gtk_css_provider_load_from_path(css_provider, "./style.css", &g_err);
    if(g_err != NULL){
        std::cout << "err" << std::endl;
    }

    //add notebook
    noot = gtk_notebook_new();
    gtk_container_add(GTK_CONTAINER(window), noot);


    //
    // add CTRL page
    //
    ctrl_grid = gtk_grid_new();
    GtkWidget* ctrl_tab_label = gtk_label_new("ctrl");
    gtk_notebook_append_page(GTK_NOTEBOOK(noot), ctrl_grid, ctrl_tab_label);

        //add ros connect button
        ros_connect_button = gtk_button_new_with_label("Connect ROS");
        gtk_grid_attach(GTK_GRID(ctrl_grid), ros_connect_button, 0, 0, 1, 1);
        g_signal_connect(G_OBJECT(ros_connect_button), "clicked", G_CALLBACK(click_ros_connect_button), NULL);

        //add ros connect address entry
        ros_connect_add_entry = gtk_entry_new();
        gtk_grid_attach(GTK_GRID(ctrl_grid), ros_connect_add_entry, 1, 0, 1, 1);
        gtk_entry_set_width_chars(GTK_ENTRY(ros_connect_add_entry), 20);
        gtk_entry_set_text(GTK_ENTRY(ros_connect_add_entry), default_ros_address.c_str());

        //add ros connect port entry
        ros_connect_port_entry = gtk_entry_new();
        gtk_grid_attach(GTK_GRID(ctrl_grid), ros_connect_port_entry, 2, 0, 1, 1);
        gtk_entry_set_width_chars(GTK_ENTRY(ros_connect_port_entry), 8);
        gtk_entry_set_text(GTK_ENTRY(ros_connect_port_entry), default_ros_port.c_str());

        //add ros status label
        ros_status_label = gtk_label_new("");
        set_ros_label_status(state.ros_connected);
        gtk_grid_attach(GTK_GRID(ctrl_grid), ros_status_label, 3, 0, 1, 1);
    
        //joystick button
        joy_connect_button = gtk_button_new_with_label("Connect Joystick");
        g_signal_connect(G_OBJECT(joy_connect_button), "clicked", G_CALLBACK(click_joystick_connect_button), NULL);
        gtk_grid_attach(GTK_GRID(ctrl_grid), joy_connect_button, 0, 1, 1, 1);

        joy_config_button = gtk_button_new_with_label("Load Joystick Config File");
        g_signal_connect(G_OBJECT(joy_config_button), "clicked", G_CALLBACK(click_load_joy_config_button), NULL);
        gtk_grid_attach(GTK_GRID(ctrl_grid), joy_config_button, 1, 1, 1, 1);

        //add home arm button
        home_arm_button = gtk_button_new_with_label("Reset ARM Postition");
        g_signal_connect(G_OBJECT(home_arm_button), "clicked", G_CALLBACK(click_home_arm_button), NULL);
        gtk_grid_attach(GTK_GRID(ctrl_grid), home_arm_button, 0, 2, 2, 1);


    
    //register move buttons helper
    auto register_button = [](GtkWidget*& layout, GtkWidget*& target, std::string text, int x, int y) {
        //create button
        target = gtk_button_new_with_label(text.c_str());
        //add to layout
        gtk_layout_put(GTK_LAYOUT(layout), target, x, y);
        //set size
        gtk_widget_set_size_request(target, 100, 50);
        //add signals
        g_signal_connect(G_OBJECT(target), "pressed", G_CALLBACK(press_movement_button), NULL);
        g_signal_connect(G_OBJECT(target), "released", G_CALLBACK(release_movement_button), NULL);

        auto context = gtk_widget_get_style_context(target);
        gtk_style_context_add_class(context,"mov_button");
    };


    //
    // base button page
    //
    base_layout = gtk_layout_new(NULL, NULL);
    gtk_widget_set_size_request(base_layout, 300, 150);

    //add buttons
    register_button(base_layout, base_buttons[0], "←", 0, 50);
    register_button(base_layout, base_buttons[1], "→", 200, 50);
    register_button(base_layout, base_buttons[2], "↓", 100, 100);
    register_button(base_layout, base_buttons[3], "↑", 100, 0);
    register_button(base_layout, base_buttons[4], "↶", 0, 0);
    register_button(base_layout, base_buttons[5], "↷", 200, 0);

    //move_vector map
    button_map[base_buttons[0]] = std::make_pair(0, -75);
    button_map[base_buttons[1]] = std::make_pair(0, 75);
    button_map[base_buttons[2]] = std::make_pair(1, -75);
    button_map[base_buttons[3]] = std::make_pair(1, 75);
    button_map[base_buttons[4]] = std::make_pair(2, -75);
    button_map[base_buttons[5]] = std::make_pair(2, 75);

    GtkWidget* base_page_label = gtk_label_new("base");
    gtk_notebook_append_page(GTK_NOTEBOOK(noot), base_layout, base_page_label);


    //
    //arm button page
    //
    arm_layout = gtk_layout_new(NULL, NULL);
    gtk_widget_set_size_request(arm_layout, 600, 150);

    register_button(arm_layout, arm_buttons[0], "←", 0, 50);
    register_button(arm_layout, arm_buttons[1], "→", 200, 50);
    register_button(arm_layout, arm_buttons[2], "↙", 200, 100);
    register_button(arm_layout, arm_buttons[3], "↗", 200, 0);
    register_button(arm_layout, arm_buttons[4], "↓", 100, 100);
    register_button(arm_layout, arm_buttons[5], "↑", 100, 0);

    button_map[arm_buttons[0]] = std::make_pair(3, 50);
    button_map[arm_buttons[1]] = std::make_pair(3, -50);
    button_map[arm_buttons[2]] = std::make_pair(5, -50);
    button_map[arm_buttons[3]] = std::make_pair(5, 50);
    button_map[arm_buttons[4]] = std::make_pair(4, -50);
    button_map[arm_buttons[5]] = std::make_pair(4, 50);

    register_button(arm_layout, arm_buttons[6], "←", 300, 50);
    register_button(arm_layout, arm_buttons[7], "→", 500, 50);
    register_button(arm_layout, arm_buttons[8], "↓", 400, 100);
    register_button(arm_layout, arm_buttons[9], "↑", 400, 0);
    register_button(arm_layout, arm_buttons[10], "↶", 300, 0);
    register_button(arm_layout, arm_buttons[11], "↷", 500, 0);

    button_map[arm_buttons[6]] = std::make_pair(7, 50);
    button_map[arm_buttons[7]] = std::make_pair(7, -50);
    button_map[arm_buttons[8]] = std::make_pair(6, 50);
    button_map[arm_buttons[9]] = std::make_pair(6, -50);
    button_map[arm_buttons[10]] = std::make_pair(8, -50);
    button_map[arm_buttons[11]] = std::make_pair(8, 50);

    register_button(arm_layout, arm_buttons[12], "grip", 500, 100);
    button_map[arm_buttons[12]] = std::make_pair(9, 0.8);

    depth_label = gtk_label_new("depth: ");
    gtk_layout_put(GTK_LAYOUT(arm_layout), depth_label, 0, 150);
    gtk_widget_set_size_request(depth_label, 100, 50);

    GtkWidget* arm_page_label = gtk_label_new("arm");
    gtk_notebook_append_page(GTK_NOTEBOOK(noot), arm_layout, arm_page_label);
    

    //exit signal 
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(quit_app), NULL);
    
    //show window
    gtk_widget_show_all(window);

}

//
// main
//
int main(int argc, char **argv){

    if (argc>1) {
        default_ros_address = std::string(*(argv+argc-2));
        default_ros_port = std::string(*(argv+argc-1));
        argc-=2;
    }
    
    //setup GUI
    setup_GUI(argc, argv);

    //begin main event loop
    gtk_main();

    return 0;

}
