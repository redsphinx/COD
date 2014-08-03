#include "Camerastuff.h"
#include "Misc.h"

//constructor
Camerastuff::Camerastuff()
{
    cameraProxy(ROBOT_IP, ROBOT_PORT);
    cameraClientName = cameraProxy.subscribe("srcImage", kVGA, kBGRColorSpace, 5);

}

//gets the taken picture
cv::Mat Camerastuff::getSrc()
{
    alSrc = cameraProxy.getImageRemote(cameraClientName);
    src.data = (uchar*) alSrc[6].GetBinary();
    cameraProxy.releaseImage(cameraClientName);
    return src;
}


//cleanup
void Camerastuff::unsubscribeFromProxy()
{
    isUnsubscribed = cameraProxy.unsubscribe(cameraClientName);
}
