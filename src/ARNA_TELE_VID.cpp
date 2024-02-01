#include <cstdlib>
#include <functional>
#include <gtkmm-3.0/gtkmm.h>
#include <gdkmm-3.0/gdkmm.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include "ARNA_THEORA.hpp"
#include "rosbridge_client.hpp"

std::string default_stream_topic = "/image_raw/theora";
std::string default_ros_address = "localhost";
std::string default_ros_port = "9090";

/*
// GTK widgets, I would use gtkmm but it is a piece of ......
*/
GtkWidget *window;
GtkWidget *gird;

// stream buttons
GtkWidget* stream_image;
GtkWidget* stream_entry;
GtkWidget* stream_button;

// ros buttons
GtkWidget *ros_connect_button;
GtkWidget *ros_connect_add_entry;
GtkWidget *ros_connect_port_entry;

/*
// Networking
*/
rosbridge_lib::rosbridge_client ros;
int ros_connected = rosbridge_lib::listener_status::lstat_disconnected;

/*
// stream buffers
*/
theora_context stream_context;
cv::Mat stream_buffer;
cv::Mat resized_buffer;
GdkPixbuf *pbp;
int width = 500;
int height = 600;

//
// video stream callback
//

// copies data from cv::Mat to GtkImage
void update_img(cv::Mat& mat, GtkWidget* target){

    if(!mat.empty()){
        
        cv::resize(stream_buffer, stream_buffer, cv::Size(std::max(width-10, 0), std::max(height-50, 0)));

        pbp = gdk_pixbuf_new_from_data(mat.data, GDK_COLORSPACE_RGB, false,
                        8, mat.cols, mat.rows, mat.cols * 3, NULL, NULL);
        
        
        gtk_image_set_from_pixbuf(GTK_IMAGE(target), pbp); //this should trigger redraw

    }
}

// TODO-low maybe make this generic with templates, not just theora streams
// callback for recieving new theora frame
void theora_recv_callback(nlohmann::json j) {
    
    // create ogg packet
    ogg_packet oggpacket;
    theora_json_to_oggpacket(j, oggpacket);

    // decode data and copy it to cvmat associated with a stream
    int new_frame = theora_oggpacket_to_cvmat(stream_context, oggpacket, stream_buffer);
    if(new_frame == 1){
	    update_img(stream_buffer, stream_image);
    }
    
    // free oggpacket data
    free(oggpacket.packet);
}

//
// ROS functions
//

//spins ros once
gint spin_ROS(gpointer data) {
    (void)data;

    ros_connected = ros.spin_once() != rosbridge_lib::spret_disconnected;
    return 1;
}

//connects to rosbridge server, schedules spin every 3ms and movement transmit every 50ms
int ros_connect(std::string address, int port) {

    // startup rosbridge_client
    ros.cleanup(); //reset ros state
    int result = ros.connect(port, address);
    if(result != rosbridge_lib::conn_return::cnret_succes) {
	    ros.cleanup();
	    ros_connected = 0;
	    return 0;
    }
    ros_connected = 1;
    
    // add network read to event loop
    // TODO-low add pipe functionality to rosbridge_lib so that we dont have to spin ros every 3ms
    g_timeout_add(3, spin_ROS, NULL);

    return ros_connected;
}

//subscribes to stream
int stream_connect(std::string topic){

    //subscribe to new topic and bind callback
    ros.subscribe(topic, theora_recv_callback);
    ros_connected = ros.send_queue() != rosbridge_lib::send_return::seret_disconnected;

    return ros_connected;
}

//
// GUI button callbacks
//

// button callback - connect to rosbridge server
void click_ros_connect_button(gpointer data) {
    (void)data;

    std::cout << "attempting to connect to ros server: ";
    
    // parse entries
    char* port_str = (char*)gtk_entry_get_text(GTK_ENTRY(ros_connect_port_entry));
    char* p;
    int port = strtol ( port_str, &p, 10);
    
    std::string address = std::string((char*)gtk_entry_get_text(GTK_ENTRY(ros_connect_add_entry)));

    //check if state valid to connect to server
    if(*p != 0) {std::cout << "invalid port" << std::endl; return;}
    if(address.empty()) {std::cout << "invalid address" << std::endl; return;}
    
    //connect to server
    ros_connect(address, port);

    if(ros_connected){std::cout << "success" << std::endl;}
    else{std::cout << "failure" << std::endl;}
}

// button callback - connect stream
void click_stream_connect_button(GtkWidget* button, gpointer data) {
    (void)button;
    (void)data;

    std::string topic = std::string((char*)gtk_entry_get_text(GTK_ENTRY(stream_entry))); //get topic from entry
    
    //check if state valid to connect stream
    if(!ros_connected){return;}
    if(topic.empty()) {std::cout << "invalid topic" << std::endl; return;}

    std::cout << "subscribing to stream" << std::endl;
    stream_connect(topic);
}

//
// GTK setup
//

// quit application
void quit_app(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;

    if(ros_connected){ros.cleanup();}
    gtk_main_quit();
}

void configure_callback(GtkWindow* window, GdkEvent* event, gpointer data){
    (void)event;
    (void)data;
    
    gint w, h;
    gtk_window_get_size(window, &w, &h);
    width = w;
    height = h;
}

//setup gui
void setup_GUI(int argc, char **argv) {

    //init GTK
    gtk_init(&argc, &argv);
    
    //create window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(window), width, height);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);

    //add grid
    gird = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(window), gird);
    
    //add ros connect button
    ros_connect_button = gtk_button_new_with_label("Connect");
    gtk_grid_attach(GTK_GRID(gird), ros_connect_button, 0, 0, 1, 1);
    g_signal_connect(G_OBJECT(ros_connect_button), "clicked", G_CALLBACK(click_ros_connect_button), NULL);

    //add ros connect address entry
    ros_connect_add_entry = gtk_entry_new();
    gtk_grid_attach(GTK_GRID(gird), ros_connect_add_entry, 1, 0, 1, 1);
    gtk_entry_set_width_chars(GTK_ENTRY(ros_connect_add_entry), 20);
    gtk_entry_set_text(GTK_ENTRY(ros_connect_add_entry), default_ros_address.c_str());

    //add ros connect port entry
    ros_connect_port_entry = gtk_entry_new();
    gtk_grid_attach(GTK_GRID(gird), ros_connect_port_entry, 2, 0, 1, 1);
    gtk_entry_set_width_chars(GTK_ENTRY(ros_connect_port_entry), 8);
    gtk_entry_set_text(GTK_ENTRY(ros_connect_port_entry), default_ros_port.c_str());

    
    //add stream connect button
    stream_button = gtk_button_new_with_label("Begin Stream");
    gtk_grid_attach(GTK_GRID(gird), stream_button, 3, 0, 1, 1);
    g_signal_connect(G_OBJECT(stream_button), "clicked", G_CALLBACK(click_stream_connect_button), NULL);
    

    stream_entry = gtk_entry_new();
    gtk_grid_attach(GTK_GRID(gird), stream_entry, 4, 0, 1, 1);
    gtk_entry_set_width_chars(GTK_ENTRY(stream_entry), 30);
    gtk_entry_set_text(GTK_ENTRY(stream_entry), default_stream_topic.c_str());

    //add stream image
    stream_image = gtk_image_new();
    gtk_grid_attach(GTK_GRID(gird), stream_image, 0, 1, 5, 1);    
    
    //exit signal
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(quit_app), NULL);
    
    //configure signal
    g_signal_connect(G_OBJECT(window), "configure-event", G_CALLBACK(configure_callback), NULL);
    
    //show window
    gtk_widget_show_all(window);
}

//
// main
//
int main(int argc, char **argv) {

    if (argc>1) {
        default_ros_address = std::string(*(argv+argc-3));
        default_ros_port = std::string(*(argv+argc-2));
        default_stream_topic = std::string(*(argv+argc-1));
        argc-=3;
    }

    //setup GUI
    setup_GUI(argc, argv);

    //begin main event loop
    gtk_main();

    return 0;
}
