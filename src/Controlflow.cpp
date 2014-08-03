#include "Movement.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

// basics. search for the ball, go to it and kick it.
void Controlflow::execute1()
{
    Movement move;

    bool ballFound = move.moveHeadAndSearch(SEARCH_FOR_BALL);
    bool bodyIsAlligned = move.alignBody();
    //get the distance
    int distance;
    move.walkDistance(distance);
    bool isBehindBall = positionBehindBall();
    // do another check in case the robot veered off path
    move.kickTheBall();
}
