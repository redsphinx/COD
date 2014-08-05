#pragma once
#include <opencv2/core/core.hpp>
#include "Misc.h"
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alvision/alvisionextractor.h>
#include <alvalue/alvalue.h>


class Camerastuff
{

    private: 
///{{{        
        AL::ALVideoDeviceProxy cameraProxy;
        cv::Mat src = cv::Mat(cv::Size(SRC_WIDTH, SRC_HEIGHT), CV_8UC3);
        const std::string cameraClientName;
        AL::ALValue alSrc;
        bool isUnsubscribed;
///}}}

    public:
///{{{  
        cv::Mat getSrc();

        unsubscribeFromProxy();

};
///}}}
