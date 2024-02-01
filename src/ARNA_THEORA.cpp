#include "ARNA_THEORA.hpp"
#include <cstring>

theora_context::theora_context(){
    pplevel_ = 0;
    received_header_ = false;
    received_keyframe_ = false;
    //header_info_ default initialized
    //header_comment_ default initialized
    setup_info_ = 0;
    decoding_context_ = 0;
}

theora_context::~theora_context(){
    th_setup_free(setup_info_);
    th_decode_free(decoding_context_);
}

void theora_context::reset(){
    pplevel_ = 0;
    received_header_ = false;
    received_keyframe_ = false;

    if (decoding_context_) {
	th_decode_free(decoding_context_);
	decoding_context_ = NULL;
    }

    if (setup_info_){
	th_setup_free(setup_info_);
	setup_info_ = NULL;
    }
	

    th_info_clear(&header_info_);
    th_info_init(&header_info_);

    th_comment_clear(&header_comment_);
    th_comment_init(&header_comment_);
}

int theora_context:: update_pp_level(){
    int pplevel_max;
    int err = th_decode_ctl(decoding_context_, TH_DECCTL_GET_PPLEVEL_MAX, &pplevel_max, sizeof(int));
    if (err) {
	//error - failed to get maximum post-processing level
    } else if (pplevel_ > pplevel_max) {
	//error - post-processing level %d is above the maximum
	pplevel_ = pplevel_max;
    }

    err = th_decode_ctl(decoding_context_, TH_DECCTL_SET_PPLEVEL, &pplevel_, sizeof(int));
    if (err) {
	//error - failed to set post-processing level
    }

    return 0;
}

// copies json data ogg_packet, remember to free ogg data!
void theora_json_to_oggpacket(nlohmann::json& j, ogg_packet& ogg){
    
    // TODO-low optimize this, use a fixed size char[]
    std::string s = j["msg"]["data"].dump();
    std::string_view sv = std::string_view(s).substr(1,s.size()-2);
    std::vector<char> data = rosbridge_lib::base64_decode(sv); //TODO-low optimize this, too many copies
    int bytes = sv.size();

    // fill ogg packet
    ogg.bytes      = bytes;
    ogg.b_o_s      = j["msg"]["b_o_s"];
    ogg.e_o_s      = j["msg"]["e_o_s"];
    ogg.granulepos = j["msg"]["granulepos"];
    ogg.packetno   = j["msg"]["packetno"];
    // WARNING caller responsible for freeing ogg.packet
    // TODO-low make this a smart pointer?
    ogg.packet = new unsigned char[ogg.bytes];
    memcpy(ogg.packet, data.data(), ogg.bytes);
}

// copies decoded contents of ogg_packet to cv::Mat
// returns 1 if new frame, 0 if no new frame, -1 if error
// TODO-low handle decode errors
int theora_oggpacket_to_cvmat(theora_context& decode, ogg_packet& oggpacket, cv::Mat& target){
    
    // beginning of stream, reset the theora_context
    if (oggpacket.b_o_s == 1) {
	decode.reset();
    }

    // decode header packets until we get the first video packet
    if (decode.received_header_ == false) {
	int result  = th_decode_headerin(&decode.header_info_, &decode.header_comment_, &decode.setup_info_, &oggpacket);
	if(result > 0){
	    //valid header packet, do nothing
	}
	else if(result == 0){
	    //recieved full header, create decoding_context_ and continue
	    decode.decoding_context_ = th_decode_alloc(&decode.header_info_, decode.setup_info_);
	    if (!decode.decoding_context_) {
		//decoding parameters were invalid
		return -1;
	    }
	    decode.received_header_ = true;
	    decode.update_pp_level();
	}
	else{
	    //invalid header packet
	    return -1;
	}
    }

    // wait until we recieve a keyframe
    decode.received_keyframe_ = decode.received_keyframe_ || (th_packet_iskeyframe(&oggpacket) == 1);
    if (!decode.received_keyframe_) { return 0; }

    // decode video packet
    int result = th_decode_packetin(decode.decoding_context_, &oggpacket, NULL);
    if(result > 0){
	//duplicate packet
	return 0;
    }
    else if(result == 0){
	//valid packet, do nothing
    }
    else{
	//invalid packet
	return -1;
    }

    // TODO-low optimize this?
    // decode new frame
    th_ycbcr_buffer ycbcr_buffer;
    th_decode_ycbcr_out(decode.decoding_context_, ycbcr_buffer);
  
    // wrap data into OpenCV format
    th_img_plane &y_plane = ycbcr_buffer[0], &cb_plane = ycbcr_buffer[1], &cr_plane = ycbcr_buffer[2];
    cv::Mat y(y_plane.height, y_plane.width, CV_8UC1, y_plane.data, y_plane.stride);
    cv::Mat cb_sub(cb_plane.height, cb_plane.width, CV_8UC1, cb_plane.data, cb_plane.stride);
    cv::Mat cr_sub(cr_plane.height, cr_plane.width, CV_8UC1, cr_plane.data, cr_plane.stride);

    // upsample chroma channels
    cv::Mat cb, cr;
    cv::pyrUp(cb_sub, cb);
    cv::pyrUp(cr_sub, cr);

    // merge into interleaved image
    cv::Mat ycrcb, channels[] = {y, cr, cb};
    cv::merge(channels, 3, ycrcb);

    // convert to bgr
    cv::Mat bgr_padded;
    cv::cvtColor(ycrcb, bgr_padded, cv::COLOR_YCrCb2RGB);

    // pull out original image region
    target = bgr_padded(cv::Rect(decode.header_info_.pic_x,
		        decode.header_info_.pic_y,
			decode.header_info_.pic_width,
			decode.header_info_.pic_height));

    return 1;
}
