#include "Movement.h"
#include "Misc.h"


//constructor
Movement::Movement()
///{{{
{
    motionProxy(ROBOT_IP, ROBOT_PORT);
    AL::ALValue stiffness = 1.0f;
    setMotionConfig(AL::ALValue::array(AL::ALValue::array("MaxStepX",0.08),
                AL::ALValue::array("MaxStepY", 0.160),
                AL::ALValue::array("MaxStepTheta", 0.524),
                AL::ALValue::array("MaxStepFrequency", 1.),
                AL::ALValue::array("StepHeight", 0.040),
                AL::ALValue::array("TorsoWx", 0.122),
                AL::ALValue::array("TorsoWy", 0.122)
                ));
    FRACTION_MAX_SPEED.push_back(1.0f);
    LEFT_LEG.push_back("LLeg");
    RIGHT_LEG.push_back("RLeg");
    postureProxy(ROBOT_IP, ROBOT_PORT);
}
///}}}

/**
 * checks if the specified object has been located
 *
 * @param[in] searchForThis the object that has to be located SEARCH_FOR_BALL or SEARCH_FOR_GOAL
 * @return a boolean indicating whether the object has been located
 */
bool Movement::locateThisObject(int searchForThis)
///{{{
{
    if(searchForThis == SEARCH_FOR_BALL)
    {
        std::pair<cv::Vec4i, float> results = ball.finalize();
        distanceToBall = results.second; 
        if(distanceToBall > 0) 
        {
            return true;
        }
        else
            return false;
    }

    //else if(searchForThis == SEARCH_FOR_GOAL)
    //{
        //distanceToGoal = goal.finalize(); //TODO make finalize in findgoal return the distance from goal
        //if(distanceToGoal > 0)
        //{
            //return true;
        //}
        //else 
            //return false;
    //}
}
///}}}


/**
 * if you want the nao to turn theta degrees in deg, then it has to take so many steps
 * @param[in] theta the angle in degrees
 * @param[in] takes into account a turning radius
 * @return the amount of steps the robot has to take
 */
// TODO what to do in case of radius
int Movement::getThetaAmountOfSteps(float theta, float radius)
///{{{
{ 
    if(radius < 50)
    {
        int steps = std::ceil(std::abs(theta) / THETA_UNIT);
        return steps;
    }
    else
    {
        //TODO

    }
}
///}}}


/**
 * if you want the nao to walk a distance in mm then in has to take so many steps
 *
 * @param[in] distance the distance the robot has to walk
 * @param[in] footStepSize the size of each step
 * @return the amount fo steps the robot has to take 
 */
int Movement::getAmountOfSteps(float distance, float footStepSize)
///{{{
{
    int steps = std::floor(distance / footStepSize);
    return steps;
}
///}}} 


/**
 * tells the robot to rotate in a circle with a radius.useful for rotating around the ball
 *
 * @param[in] theta the angle
 * @param[in] radius the radius of the circle
 */
void Movement::rotateWithRadius(float theta, float radius)
///{{{
{
    std::vector<std::string> startLeg;
    std::vector<std::string> otherLeg;

    
    if(theta < 0) //turn right
    {
        startLeg = RIGHT_LEG;
        otherLeg = LEFT_LEG;
        theta = -theta * DEG2RAD;
    }

    else
    {
        startLeg = LEfT_LEG;
        otherLeg = RIGHT_LEG;
        theta = theta * DEG2RAD;
    }


    //for now radius is 0

    if(radius < 50) //in mm
    {
        int brokerTaskId = motionProxy.moveInit();
        AL::ALValue footSteps = AL::ALValue::array(0, 0, theta);
        clearExisting = false;
        int steps = getThetaAmountOfSteps(theta, 0);

        for(int i=0; i<steps; i++)
        {
            if(i % 2 == 0)
            {
                motionProxy.setFootStepsWithSpeed(startLeg, footSteps, FRACTION_MAX_SPEED, clearExisting);
            }

            else
            {
                motionProxy.setFootStepsWithSpeed(otherLeg, footSteps, FRACTION_MAX_SPEED, clearExisting);

            }
        }
    }
    //else if we have to turn with a valid radius
    else if(radius >= 50)
    {
        //TODO

    }
}
///}}}


/**
 * makes the robot turn its head to search for the ball or goal
 *
 * @param[in] searchForThis indicates what to search for. 1 for ball and 2 for goal
 */
void Movement::moveHeadAndSearch(int searchForThis)
///{{{
{
    bool objectFound = false;
    bool headChangedDirection = false; //keeps track of the head, if it changed from moving left to right or right to left
    int headDirectionChangeCntr = 0; //if the counter reaches 2 this means the entire area has been searched for without finding the object

    while(objectFound == false)
    {
        if(headDirectionChangeCntr < 2) //if the entire area hasn't been scanned yet
        {

            topCamIsSet = ALVisionExtractor::setActiveCamera(AL::kTopCamera);
            //set the head pitch to the correct angle for the top cam
            motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, HEAD_PITCH_TOP_CAM, 1.0);


            objectFound = locateThisObject(searchForThis); //try to locate object with top cam

            if(objectFound == false)
            {

                botCamIsSet = ALVisionExtractor::setActiveCamera(AL::kBottomCamera);
                motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, HEAD_PITCH_BOT_CAM, 1.0);

                objectFound = locateThisObject(searchForThis); //try to locate object with bot cam


                if(locateThisObject == false) //if object still isnt located then turn the head
                {
                    currentHeadYaw = motionProxy.getAngles(HEAD_YAW_JOINT_NAME, true); 

                    if(currentHeadYaw == HEAD_YAW_MIDDLE && headChangedDirection == false)
                    {
                        motionProxy.setAngles(HEAD_YAW_JOINT_NAME, HEAD_YAW_THRESH_LEFT, 1.0);
                    }

                    else if(currentHeadYaw > HEAD_YAW_MIDDLE && currentHeadYaw < HEAD_YAW_RIGHT_MAX && headChangedDirection == false)
                    {
                        motionProxy.setAngles(HEAD_YAW_JOINT_NAME, HEAD_YAW_THRESH_RIGHT, 1.0);
                    }

                    else if(currentHeadYaw >= HEAD_YAW_RIGHT_MAX && headChangedDirection == false)
                    {
                        headChangedDirection == true;
                        headDirectionChangeCntr++;
                        motionProxy.setAngles(HEAD_YAW_JOINT_NAME, 0, 1.0)
                    }

                    else if(currentHeadYaw == HEAD_YAW_MIDDLE && headChangedDirection == true)
                    {
                        motionProxy.setAngles(HEAD_YAW_JOINT_NAME, HEAD_YAW_THRESH_LEFT, 1.0);
                    }

                    else if(currentHeadYaw < HEAD_YAW_MIDDLE && currentHeadYaw > HEAD_YAW_LEFT_MAX && headChangedDirection == true)
                    {
                        motionProxy.setAngles(HEAD_YAW_JOINT_NAME, HEAD_YAW_THRESH_LEFT, 1.0);
                    }

                    else if(currentHeadYaw <= HEAD_YAW_LEFT_MAX && headChangedDirection == true)
                    {
                        headChangedDirection = false;
                        headDirectionChangeCntr++;
                        motionProxy.setAngles(HEAD_YAW_JOINT_NAME, 0, 1.0);
                    }
                }
            }
        }

        else if(headDirectionChangeCntr >= 2) //if entire area has been scanned
        {

            headChangedDirection = false;
            headDirectionChangeCntr = 0;
            motionProxy.setAngles(HEAD_YAW_JOINT_NAME, 0, 1.0);
            motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, HEAD_PITCH_TOP_CAM, 1.0);
            rotateWithRadius(-BODY_ANGLE_THRESH, 0); //minus to indicate clockwise rotation
        }
    }
}
///}}}


/**
 * aligns the body with the head
 * after the robot has seen the ball, the next step is to align it's body to the direction the head is facing
 */
void Movement::alignBody()
///{{{
{

    //get the current angle of the head relative to the body
    currentHeadYaw = motionProxy.getAngles(HEAD_YAW_JOINT_NAME, true); 
    //
    rotateWithRadius(currentHeadYaw, 0);

    motionProxy.setAngles(HEAD_YAW_JOINT_NAME, HEAD_YAW_MIDDLE, 1.0);

}
///}}}


/**
 * makes the robot walk a certain distance before checking the camera for the next picture. distance walked depends on the distance between the robot and the obejct
 *
 * @param[in] distanceX the front or back distance in mm
 * @param[in] distanceY the left or right distance in mm
 */
void Movement::walkDistance(float distanceX, float distanceY)
///{{{
    //if distance is negative, for x this means backwards, for y this means to the right
{
    float theta = 0.0f;
    motionProxy.walkInit();

    std::vector<std::string> startLeg;
    std::vector<std::string> otherLeg;
    std::vector<std::string> lastMovedLeg;

    //if we are stepping to the side
    if(distanceX == 0) //if we are stepping to the side
    {
        float sideStepSize = 50.0f; //in mm
        int amountOfSteps = getAmountOfSteps(distanceY, sideStepSize);

        AL::ALValue footSteps = AL::ALValue::array(0, sideStepSize, 0);
        bool clearExisting = true;

        if(distanceY < 0) //if we move right, start with right leg
        {
            startLeg = RIGHT_LEG;
            otherLeg = LEFT_LEG;
        }
        else if(distanceY > 0) //if we move left, start with left leg
        {
            startLeg = LEFT_LEG;
            otherLeg = RIGHT_LEG;
        }

        for(int i=0; i<amountOfSteps; i++)
        {
            if(amountOfSteps % 2 == 0)
            {
                motionProxy.setFootStepsWithSpeed(startLeg, footSteps, FRACTION_MAX_SPEED, clearExisting);
                lastMovedLeg = startLeg;

            }
            else
            {
                motionProxy.setFootStepsWithSpeed(otherLeg, footSteps, FRACTION_MAX_SPEED, clearExisting);
                lastMovedLeg = otherLeg;

            }
        }
    }

    //if we are stepping forward or backward
    else if(distanceY == 0) 
    {
        float largeStepSize = 80.0f; //in mm
        float smallStepSize = 30.0f; //in mm
        int amountOfSteps;
        AL::ALValue footSteps;
        if(distanceY < 300) //in mm
        {
            amountOfSteps = getAmountOfSteps(distanceY, smallStepSize);
            footSteps = AL::ALValue::array(smallStepSize, 0, 0);
        }
        else if(distanceY >= 300) //in mm
        {
            amountOfSteps = getAmountOfSteps(distanceY, largeStepSize);
            footSteps = AL::ALValue::array(largeStepSize, 0, 0);
        }

        bool clearExisting = true;

        if(distanceY < 0) //if we are moving backwards
        {
            startLeg = RIGHT_LEG;
            otherLeg = LEFT_LEG;
        }
        else if(distanceY > 0) //if we are moving forwards
        {
            startLeg = LEFT_LEG;
            otherLeg = RIGHT_LEG;
        }
        for(int i=0; i<amountOfSteps; i++)
        {
            if(amountOfSteps % 2 == 0)
            {
                motionProxy.setFootStepsWithSpeed(startLeg, footSteps, FRACTION_MAX_SPEED, clearExisting);
                lastMovedLeg = startLeg;

            }
            else
            {
                motionProxy.setFootStepsWithSpeed(otherLeg, footSteps, FRACTION_MAX_SPEED, clearExisting);
                lastMovedLeg = otherLeg;

            }
        }
    }
}
///}}}


/**
 * makes the robot walk to the ball
 */
void Movement::walkToBall()
///{{{
{
    walkDistance(distanceToBall, 0);
}
///}}}


//takes pictures until it sees the ball
/**
 * takes pictures until the object is seen
 *
 * @param[in] mode the object you want to find
 */
void Movement::takePicUntilSeen(int mode) 
///{{{
{

    bool seen = false;
    std::pair<cv::Vec4i, float> finalizeResults;
    cv::Vec4i bbStorage;

    if(mode == SEARCH_FOR_BALL)
    {
        while(seen == false)
        {
            //gets information from the FindBall class to see if the ball has been located
            finalizeResults = ball.finalize();
            //the Vec4i vector with the coordinates of the ball if found. if not found it will contain a (0, 0, 0, 0) vector
            bbStorage = finalizeResults.first;
            if(bbStorage =! ZERO_BB)
            {
                seen = true;
            }

            //if the ball has not been seen
            if(seen == false)
            {

                currentHeadPitch = motionProxy.getAngles(HEAD_PITCH_JOINT_NAME, true); 
                float tmpPitch = currentHeadPitch;
                float fractionValue = HEAD_PITCH_FRONT_MAX / tmpPitch;
                //adjust pitch to normal
                motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, 0, FRACTION_MAX_SPEED);
                //adjust pitch back to how it was
                motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, fractionValue, FRACTION_MAX_SPEED);
                motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, HEAD_PITCH_TOP_CAM, FRACTION_MAX_SPEED);
                //adjust pitch to tmpPitch
            }
        }
    }
    // TODO finish when the kicking is working
    ///{{{
    else if(mode == SEARCH_FOR_GOAL)
    { 
        while(seen == false)
        {
            finalizeResults = goal.finalize(); //TODO in FindGoal.cpp
            bbStorage = finalizeResults.first;
            if(bbStorage =! ZERO_BB)
            {
                seen = true;
            }

            if(seen == false)
            {
                currentHeadPitch = motionProxy.getAngles(HEAD_PITCH_JOINT_NAME, true); 
                float tmpPitch = currentHeadPitch;
                float fractionValue = 29.5 / tmpPitch*RAD2DEG;
                //adjust pitch to normal
                motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, 0, 1.0);
                //adjust pitch back to how it was
                motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, fractionValue, 1.0);
                motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, HEAD_PITCH_TOP_CAM, 1.0);
                //adjust pitch to tmpPitch
            }
        }
    }
    ///}}}
}
///}}}


/**
 * moves the robot correctly to the left or right, front or back to get in correct position behind the ball
 *
 * @param[in] leg indicated left or right
 * @param[in] mode indicates horizontal or vertical, 1 is hor, 2 is ver
 * @param[in] bbBall the bounding box of the last detected ball
 */
void Movement::moveRobotToCorrectBallPlacement(std::vector<std::string> leg, int modeHorVer, cv::Vec4i bbBall)
///{{{
{

    bool isInPos = false;
    int bbMiddle;
    int perfectBallPos;
    int distanceToShift;
    cv::Vec4i perfectBallPosBB;
    float distanceX = 0;
    float distanceY = 0;

    if(modeHorVer == ROBOT_MOVE_MODE_HOR) //hor so X
    {
        bbMiddle = (bbBall[0]+bbBall[2]) / 2;

        if(leg == LEFT_LEG)
        {
            perfectBallPos = PERFECT_LEFT_BALL_POS_X;
            perfectBallPosBB = PERFECT_LEFT_BALL_POS_BB;
        }
        else if(leg == RIGHT_LEG)
        {
            perfectBallPos = PERFECT_RIGHT_BALL_POS_X;
            perfectBallPosBB = PERFECT_RIGHT_BALL_POS_BB;
        }
    }

    else if(modeHorVer == ROBOT_MOVE_MODE_VER) //ver so Y
    {
        bbMiddle = (bbBall[1]+bbBall[3]) / 2;

        if(leg == LEFT_LEG)
        {
            perfectBallPos = PERFECT_LEFT_BALL_POS_Y;
            perfectBallPosBB = PERFECT_LEFT_BALL_POS_BB;
        }
        else if(leg == RIGHT_LEG)
        {
            perfectBallPos = PERFECT_RIGHT_BALL_POS_Y;
            perfectBallPosBB = PERFECT_RIGHT_BALL_POS_BB;
        }

    }

    while(isInPos == false)
    {
        takePicUntilSeen(bbBall, SEARCH_FOR_BALL); 

        if(bbMiddle < perfectBallPos)
        {
            distanceToShift = perfectBallPos - bbMiddle;
        }

        else if(bbMiddle >= perfectBallPos)
        {
            distanceToShift = bbMiddle - perfectBallPos;
        }

        //set the correct valus for distances
        if(modeHorVer == ROBOT_MOVE_MODE_HOR)
        {
            distanceX = 0;
            if(leg == RIGHT_LEG)
            {
                distanceY = -distanceToShift;
            }
            else if(leg == LEFT_LEG)
            {
                distanceY = distanceToShift;
            }

        }
        else if(modeHorVer == ROBOT_MOVE_MODE_VER)
        {
            distanceX = distanceToShift;
            distanceY = 0;

        }

        //walk some distance
        walkDistance(distanceX, distanceY);

        //check if the ball is in the good position
        takePicUntilSeen(bbBall, SEARCH_FOR_BALL);

        int var1;
        int var2;

        if(modeHorVer == ROBOT_MOVE_MODE_HOR)
        {
            var1 = 0;
            var2 = 2;
        }
        else if(modeHorVer == ROBOT_MOVE_MODE_VER)
        {
            var1 = 1;
            var1 = 3;
        }

        if(leg == LEFT_LEG)
        {
            perfectBallPosBB = PERFECT_LEFT_BALL_POS_BB;
        }
        else if(leg == RIGHT_LEG)
        {
            perfectBallPosBB = PERFECT_RIGHT_BALL_POS_BB;
        }

        bbMiddle = (bbBall[var1]+bbBall[var2]) / 2;

        if(bbMiddle <= perfectBallPosBB[var2] && bbMiddle > perfectBallPosBB[var1])
        {
            isInPos = true;

        }
    }
}
///}}}


//get the angle that the robot need to turn itself into position behind ball correctly 
float Movement::getTurnAngle(float angleToGoal, float angleToBall)
///{{{
{

    //implement if he can walk and kick
    return 1.0f;

}
///}}}


/**
 * makes the robot stand correctly behind the ball facing the goal
 */
void Movement::positionBehindBall()
///{{{
{
    std::vector<std::string> leg;
    botCamIsSet = ALVisionExtractor::setActiveCamera(AL::kBottomCamera);

    motionProxy.setAngles(HEAD_PITCH_JOINT_NAME, HEAD_PITCH_BOT_CAM_CLOSE, 1.0);

    //assume the robot is standing facing the goal

    bool isInPosition = false;
    cv::Vec4i bbBall;

    while(isInPosition == false)
    {

        takePicUntilSeen(bbBall, SEARCH_FOR_BALL);

        //now we have the bounding box
        //see what side the bounding box is at, so if right set kicking foot to right, if left set kicking foot to left (horizontal)
        int bbMiddleX = (bbBall[0]+bbBall[2]) / 2;

        if(bbMiddleX <= SRC_WIDTH/2) //midpoint of ball is to the left
        {
            leg = LEFT_LEG;
        }

        else
        {
            leg = RIGHT_LEG;
        }

        KICK_LEG = leg; 

        //shift to correct horizontal position
        moveRobotToCorrectBallPlacement(KICK_LEG, ROBOT_MOVE_MODE_HOR, bbBall); 
        //shift to correct vertical position
        moveRobotToCorrectBallPlacement(KICK_LEG, ROBOT_MOVE_MODE_VER, bbBall); 

    }
}
///}}}


/**
 * makes the robot kick the ball
 */
void Movement::kickTheBall()
///{{{
{
    if (KICK_LEG == RIGHT_LEG)
    {
        AL::ALValue names = AL::ALValue::array("RShoulderRoll", "RShoulderPitch", "LShoulderRoll", "LShoulderPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll");
        AL::ALValue angles = AL::ALValue::array(AL::ALValue::array(-0.3), AL::ALValue::array(0.4), AL::ALValue::array(0.5), AL::ALValue::array(1.0), AL::ALValue::array(0.0), AL::ALValue::array(-0.4, -0.2), AL::ALValue:: array(0.95, 1.5), AL::ALValue::array(-0.55, -1), AL::ALValue::array(0.2), AL::ALValue::array(0.0), AL::ALValue::array(-0.4), AL::ALValue::array(0.95), AL::ALValue::array(-0.55), AL::ALValue::array(0.2));
        AL::ALValue times =  AL::ALValue::array(AL::ALValue::array( 0.5), AL::ALValue::array(0.5), AL::ALValue::array(0.5), AL::ALValue::array(0.5), AL::ALValue::array(0.5), AL::ALValue::array( 0.4,  0.8), AL::ALValue:: array( 0.4, 0.8),  AL::ALValue::array(0.4, 0.8), AL::ALValue::array(0.4), AL::ALValue::array(0.5), AL::ALValue::array( 0.4), AL::ALValue::array( 0.4), AL::ALValue::array( 0.4),  AL::ALValue::array(0.4));

        motionProxy.angleInterpolation(names, angles, times, true);

        motionProxy.angleInterpolationWithSpeed(AL::ALValue::array("RShoulderPitch", "RHipPitch", "RKneePitch", "RAnklePitch"),
                AL::ALValue::array(             1.5,        -0.7,         1.05,          -0.5),
                1.0,
                true);
        motionProxy.angleInterpolation(AL::ALValue::array("RHipPitch", "RKneePitch", "RAnklePitch"),
                AL::ALValue::array(       -0.5,          1.1,         -0.65),
                0.25,
                true);
        motionProxy.moveInit();
    }

    //---

    else if (KICK_LEG == LEFT_LEG)
    {
        AL::ALValue angles = AL::ALValue::array(AL::ALValue::array(0.3), AL::ALValue::array(0.4), AL::ALValue::array(-0.5), AL::ALValue::array(1.0), AL::ALValue::array(0.0), AL::ALValue::array(-0.4, -0.2), AL::ALValue:: array(0.95, 1.5), AL::ALValue::array(-0.55, -1), AL::ALValue::array(-0.2), AL::ALValue::array(0.0), AL::ALValue::array(-0.4), AL::ALValue::array(0.95), AL::ALValue::array(-0.55), AL::ALValue::array(-0.2));
        AL::ALValue times =  AL::ALValue::array(AL::ALValue::array(0.5), AL::ALValue::array(0.5), AL::ALValue::array( 0.5), AL::ALValue::array(0.5), AL::ALValue::array(0.5), AL::ALValue::array( 0.4,  0.8), AL::ALValue:: array( 0.4, 0.8), AL::ALValue::array( 0.4, 0.8), AL::ALValue::array( 0.4), AL::ALValue::array(0.5), AL::ALValue::array( 0.4), AL::ALValue::array(0.4) , AL::ALValue::array(0.4),   AL::ALValue::array(0.4));

        motionProxy.angleInterpolation(names, angles, times, true);
        motionProxy.angleInterpolationWithSpeed(AL::ALValue::array("LShoulderPitch", "LHipPitch", "LKneePitch", "LAnklePitch"),
                AL::ALValue::array(             1.5,        -0.7,         1.05,          -0.5),
                1.0,
                true);
        motionProxy.angleInterpolation(AL::ALValue::array("LHipPitch", "LKneePitch", "LAnklePitch"),
                AL::ALValue::array(       -0.5,          1.1,         -0.65),
                0.25,
                true);
        motionProxy.moveInit();
    }
}
///}}}

/**
 * makes it all come together
 */
void Movement::finalize()
///{{{
{
    
    //make sure the robot is standing upright
    postureProxy.goToPosture("StandInit", FRACTION_MAX_SPEED);

    moveHeadAndSearch(SEARCH_FOR_BALL);
    alignBody();

    //make sure the robot is standing upright
    postureProxy.goToPosture("StandInit", FRACTION_MAX_SPEED);

    walkToBall();
    positionBehindBall();
    kickTheBall();
}
///}}}
