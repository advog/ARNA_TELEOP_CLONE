#pragma once

#include <ogg/ogg.h>
#include <theora/codec.h>
#include <theora/theoradec.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "rosbridge_client.hpp"
#include "json.hpp"

class theora_context {
public:
    int pplevel_; 
    bool received_header_;
    bool received_keyframe_;

    th_info header_info_;
    th_comment header_comment_;

    th_setup_info* setup_info_;
    th_dec_ctx* decoding_context_;

    theora_context();
    ~theora_context();
    void reset();
    int update_pp_level();
};

int theora_oggpacket_to_cvmat(theora_context& decode, ogg_packet& oggpacket, cv::Mat& target);
void theora_json_to_oggpacket(nlohmann::json& j, ogg_packet& ogg);



