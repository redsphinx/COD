#include "Camerastuff.h"
#include "Misc.h"
#include <alcommon/albroker.h>

//constructor
Camerastuff::Camerastuff(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) : AL::ALModule(broker, name)
///{{{ 
{
    setModuleDescription("module that does camera things");
    
    functionName("getSrc", "Camerastuff", "takes a picture and returns it");
    BIND_METHOD(Camerastuff::getSrc);

    functionName("unsubscribeFromProxy", "Camerastuff", "unsubscribes from proxy");
    BIND_METHOD(Camerastuff::unsubscribeFromProxy);
    
    cameraProxy(ROBOT_IP, ROBOT_PORT);
    cameraClientName = cameraProxy.subscribe("srcImage", kVGA, kBGRColorSpace, 5);


}
///}}}

Camerastuff::~Camerastuff();

void Camerastuff::init()
{
}

//gets the taken picture
cv::Mat Camerastuff::getSrc()
///{{{
{
    alSrc = cameraProxy.getImageRemote(cameraClientName);
    src.data = (uchar*) alSrc[6].GetBinary();
    cameraProxy.releaseImage(cameraClientName);
    return src;
}
///}}}


//cleanup
void Camerastuff::unsubscribeFromProxy()
///{{{
{
    isUnsubscribed = cameraProxy.unsubscribe(cameraClientName);
}
///}}}
