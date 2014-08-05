#pragma once
#include <opencv2/core/core.hpp>
#include "FindBall.h"
#include "FindGoal.h"
#include <alproxies/almotionproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alvalue/alvalue.h>
#include <alproxies/alrobotpostureproxy.h>

class Movement
{

    private: 
///{{{
        AL::ALMotionProxy motionProxy;
        AL::ALRobotPostureProxy postureProxy;

        FindBall ball; //object of class FindBall
        FindGoal goal; //object of class FindGoal
        float relativeGoalAngle; //the angle of the goal relative to the robot's head, seen from a top-down view
        float relativeBallAngle; //the andle of the ball relative to the robot's head, seen from a top-down view
        cv::Mat src; //get original image, encode as RGB
        float distanceToBall;
        float distanceToGoal;
        bool cameraTopSelected; //indicate if top camera selected
        bool cameraBotSelected; //indicate if bottom camera selected
        float currentHeadYaw; //current position of the head yaw (some kind of aldebaran code probably)
        float currentHeadPitch; //current position of the head pitch (also aldebaran)
        float turnedHeadAngleSoFar; //how much in total (clockwise neg, anticlock pos) the head has turned since the start
        float turnedBodyAngleSoFar; //how much in total (cockwise, neg, anticlock pos) the body has turned since the start
        const AL::ALValue HEAD_YAW_JOINT_NAME = AL::ALValue::array("HeadYaw");
        const AL::ALValue HEAD_PITCH_JOINT_NAME = AL::ALValue::array("HeadPitch");
        bool topCamIsSet;
        bool botCamIsSet;
        float THETA_UNIT = 10; // how much the robot turns at once in degrees
        const AL::ALValue HEAD_PITCH_TOP_CAM = AL::ALValue::array(0.6);
        const AL::ALValue HEAD_PITCH_BOT_CAM = AL::ALValue::array(0.4);
        const AL::ALValue HEAD_PITCH_BOT_CAM_CLOSE = AL::ALValue::array(1.0); //use for ball positionning
        std::vector<float> FRACTION_MAX_SPEED;
        std::vector<std::string> LEFT_LEG;
        std::vector<std::string> RIGHT_LEG;
        std::vector<std::string> KICK_LEG;
///}}}

    public:
///{{{
        //constructor
        Movement();


        /**
         * makes the robot turn its head to search for the ball or goal
         *
         * @param[in] searchForThis indicates what to search for. 1 for ball and 2 for goal
         */
        bool moveHeadAndSearch(int searchForThis);


        /**
         * checks if the specified object has been located
         *
         * @param[in] searchForThis the object that has to be located SEARCH_FOR_BALL or SEARCH_FOR_GOAL
         * @return a boolean indicating whether the object has been located
         */
        bool locateThisObject(int searchForThis);


        /**
         * if you want the nao to turn theta degrees in deg, then it has to take so many steps
         * @param[in] theta the angle in degrees
         * @param[in] takes into account a turning radius
         * @return the amount of steps the robot has to take
         */
        int getThetaAmountOfSteps(float theta, float radius);


        /**
         * if you want the nao to walk a distance in mm then in has to take so many steps
         *
         * @param[in] distance the distance the robot has to walk
         * @param[in] footStepSize the size of each step
         * @return the amount fo steps the robot has to take 
         */
        int getAmountOfSteps(float distance, float footStepSize);


        /**
         * tells the robot to rotate in a circle with a radius.useful for rotating around the ball
         *
         * @param[in] theta the angle
         * @param[in] radius the radius of the circle
         */
        void rotateWithRadius(float theta, float radius);


        /**
         * aligns the body with the head
         */
        void alignBody();


        /**
         * makes the robot walk a certain distance before checking the camera for the next picture. distance walked depends on the distance between the robot and the obejct
         *
         * @param[in] distanceX the front or back distance in mm
         * @param[in] distanceY the left or right distance in mm
         */
        void walkDistance(float distanceX, float distanceY);


        /**
         * makes the robot walk to the ball
         */
        void walkToBall();


        //takes pictures until it sees the ball
        /**
         * takes pictures until the object is seen
         *
         * @param[in] mode the object you want to find
         */
        void takePicUntilSeen(int mode); 


        /**
         * moves the robot correctly to the left or right, front or back to get in correct position behind the ball
         *
         * @param[in] leg indicated left or right
         * @param[in] mode indicates horizontal or vertical, 1 is hor, 2 is ver
         * @param[in] bbBall the bounding box of the last detected ball
         */
        void moveRobotToCorrectBallPlacement(std::vector<std::string> leg, int modeHorVer, cv::Vec4i bbBall);


        /**
         * get the angle that the robot needs to turn itself into position facing the goal behind ball  
         */
        float getTurnAngle(float angleToGoal, float angleToBall);


        /**
         * makes the robot stand correctly behind the ball facing the goal
         */
        void positionBehindBall();


        /**
         * makes the robot kick the ball
         */
        void kickTheBall();

        /**
         * makes it all come together
         */
        void finalize();

};
///}}}
