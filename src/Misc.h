#pragma once
#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <alvalue/alvalue.h>

//robot joint angle stuff
///{{{

const float HEAD_YAW_RIGHT_MAX = -119.5 * DEG2RAD; 
const float HEAD_YAW_LEFT_MAX = 119.5 * RAD2DEG; 
const float HEAD_YAW_MIDDLE = 0;
//const int HOR_ANGLE_TOP_CAM = 0; //TODO not sure why i made this
//const int VER_ANGLE_TOP_CAM = 0; //TODO not sure why i made this

AL::ALValue HEAD_YAW_THRESH_RIGHT = AL::ALValue::array(-0.5f); //maximum turn to the right
AL::ALValue HEAD_YAW_THRESH_LEFT = AL::ALValue::array(0.5f); //maximum turn to the left

const int BODY_ANGLE_THRESH = 60 * DEG2RAD; //60 degrees

const float HEAD_PITCH_FRONT_MAX = 29.5 * DEG2RAD; 
const float HEAD_PITCH_BACK_MAX = -38.5 * DEG2RAD; 

///}}}


//foot and ball stuff
///{{{

const cv::Vec4i PERFECT_LEFT_BALL_POS_BB = cv::Vec4i(173, 257, 272, 354);
const cv::Vec4i PERFECT_RIGHT_BALL_POS_BB = cv::Vec4i(381, 256, 479, 353);
const int PERFECT_LEFT_BALL_POS_X = (int)(PERFECT_LEFT_BALL_POS_BB[0]+PERFECT_LEFT_BALL_POS_BB[2]) / 2;
const int PERFECT_LEFT_BALL_POS_Y = (int)(PERFECT_LEFT_BALL_POS_BB[1]+PERFECT_LEFT_BALL_POS_BB[3]) / 2;
const int PERFECT_RIGHT_BALL_POS_X = (int)(PERFECT_RIGHT_BALL_POS_BB[0]+PERFECT_RIGHT_BALL_POS_BB[2]) / 2;
const int PERFECT_RIGHT_BALL_POS_Y = (int)(PERFECT_RIGHT_BALL_POS_BB[1]+PERFECT_RIGHT_BALL_POS_BB[3]) / 2;

//const std::vector<std::string> LEFT_FOOT = ["LLeg"]; //indicates the left foot of the robot
//const std::vector<std::string> RIGHT_FOOT = ["RLeg"]; //indicates the right foot of the robot
const int NO_FOOT = 0; //indicates no foot

const float BALL_RADIUS_MIN = 80/2; //in mm
const float BALL_RADIUS_MAX = 60/2; //in mm
const float BALL_DISTANCE_ERROR = BALL_RADIUS_MAX - BALL_RADIUS_MIN + 5; //fuck it guess 5

const float FOOTSTEP_SIZE_MAX = 1.0; //TODO

///}}}


//field stuff
///{{{

const int FIELD_WIDTH = 4500; //in mm
const int FIELD_LENGTH = 6000; //in mm
const int FIELD_DIAGONAL = (int) std::sqrt(std::pow(FIELD_WIDTH, 2) + std::pow(FIELD_LENGTH, 2)); //length of the field diagonal
const int DISTANCE_DIVIDER = 10; //divide the distance that is to be walked in shorter distances for more accurate results



///}}}


//camera and picture stuff
///{{{

const int SRC_WIDTH = 640;
const int SRC_HEIGHT = 480;

const float CAM_FIELD_OF_VIEW_HOR = 60.97; 
const float CAM_FIELD_OF_VIEW_VER = 47.64; //for both top and bottom cam 


///}}}


//miscellaneous
///{{{

const std::string ROBOT_IP = "local"; //the ip address of the robot
int ROBOT_PORT = 9559; //the robot port
const float PI = 3.14159265358979323846; //the value of the constant pi
const float DEG2RAD = PI/180; //convert degrees to radians
const float RAD2DEG = 180/PI; //convert radians to degrees
const int SEARCH_FOR_BALL = 1; //indicator that the ball is the object being searched for, sometimes indicated as 'mode' or 'searchForThis'
const int SEARCH_FOR_GOAL = 2; //indicator that the goal is the object being searched for, sometimes indicated as 'mode' or 'searchForThis'
const int ROBOT_MOVE_MODE_HOR = 1; //indicates that the robot needs to correct its position horizontally
const int ROBOT_MOVE_MODE_VER = 2; //indicates that the robot needs to correct its position vertically
const cv::Vec4i ZERO_BB = cv::Vec4i(0,0,0,0); 
const std::vector<float> FRACTION_MAX_SPEED = [1.0];

///}}}
