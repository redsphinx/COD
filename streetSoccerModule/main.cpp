include "Movement.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

//#include <stdlib.h>

#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

int main()
{

    // We will try to connect our broker to a running NAOqi
    int pport = PORT;
    std::string pip = IP_ADDRESS;

    // Need this to for SOAP serialization of floats to work
    setlocale(LC_NUMERIC, "C");

    // A broker needs a name, an IP and a port:
    const std::string brokerName = "mybroker";

    //try these ports if 22 doesn't work
    //Proto Recv-Q Send-Q Local Address           Foreign Address         State      
    //tcp        0      0 127.0.0.1:631           0.0.0.0:*               LISTEN     
    //tcp        0      0 127.0.0.1:57151         0.0.0.0:*               LISTEN     
    //tcp        0      0 127.0.1.1:53            0.0.0.0:*               LISTEN 
   
    int brokerPort = 22;
    // listen port of the broker (here an anything)
    const std::string brokerIp = "0.0.0.0";


    // Create your own broker
    boost::shared_ptr<AL::ALBroker> broker;

    broker = AL::ALBroker::createBroker(
                brokerName,
                brokerIp,
                brokerPort,
                pip,
                pport,
                0    // you can pass various options for the broker creation,
                // but default is fine
                );

    // Deal with ALBrokerManager singleton (add your borker into NAOqi)
    AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(broker);

    // Now it's time to load your module with
    // AL::ALModule::createModule<your_module>(<broker_create>, <your_module>);
    AL::ALModule::createModule<streetSoccerModule>(broker, "streetSoccerModule");

    while (true)
        qi::os::sleep(1);

    std::cout << "FUCKIN TEEEST" << std::endl;
    return 0;
}



// basics. search for the ball, go to it and kick it.
//void main()
//{
    //Movement move;
    //move.finalize();
//}
