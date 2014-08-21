#pragma once
#include <opencv2/core/core.hpp>
#include "Misc.h"
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alvision/alvisionextractor.h>
#include <alvalue/alvalue.h>
#include <alcommon/almodule.h>


namespace AL
{
    //this is a forward devlaration of AL:ALBroker which avoids including 
    //<alcommon/albroker.h> in this header
    class ALBroker;
}


/**
 * this class inherits AL::ALModule. this allows it to bind methods
 * and be run as a remote executable withing NAOqi
 */
class Camerastuff : public AL::ALModule
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
        
        Camerastuff(boost::shared_ptr<AL::ALBroker> broker, const std::string &name);

        virtual ~Camerastuff();
        /**
         * overloading ALModule::init()
         * this is called right after the module has been loaded
         */
        virtual void init();

        cv::Mat getSrc();

        unsubscribeFromProxy();

};
 ///}}}

