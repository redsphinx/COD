//#include "Movement.h"
#include "FindBall.h"
#include "Misc.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

//constructor
FindBall::FindBall()
{
} 



/**
 * returns the bounding box of a blob
 *
 * @param[in] blob a contour with a roundish shape
 * @return the bounding box with top-left and bottom-right coordinate
 */
cv::Vec4i FindBall::getBoundingBox(std::vector<cv::Point> blob)
//{{{
{
    int maxX = 0;
    int minX = 1000000;
    int maxY = 0;
    int minY = 1000000;

    for(int i=0; i<blob.size(); i++)
    {
        int blobX = blob[i].x;
        int blobY = blob[i].y;

        if(blobX > maxX)
        {
            maxX = blobX;
        }

        if(blobY > maxY)
        {
            maxY = blobY;
        }

        if(blobX < minX)
        {
            minX = blobX;
        }

        if(blobY < minY)
        {
            minY = blobY;
        }
        
    }
    return cv::Vec4i(minX, minY, maxX, maxY);
}
//}}}


/**
 * returns the error of a blob
 *
 * @param[in] bbox he bounding box of a blob
 * @param[in] blob a blob is a contour, which is a collection of points. a contour defines the outline of a found round-ish object
 * @return the error of a blob
 */
float FindBall::getBlobError(cv::Vec4i bbox, std::vector<cv::Point> blob)
//{{{
{
    float error = 0;
    int x, y;
    int bbMaxX = std::max(bbox[0], bbox[2]);
    int bbMinX = std::min(bbox[0], bbox[2]);
    int bbMaxY = std::max(bbox[1], bbox[3]);
    int bbMinY = std::min(bbox[1], bbox[3]);
    float averageRadius = ((bbMaxX-bbMinX) + (bbMaxY-bbMinY))/4; //the average radius of the blob 
    float pointRadius; //the radius from the center of the blob to a point of the blob
    int originX = (bbMaxX+bbMinX)/2; //horizontal midpoint of the bb of the blob
    int originY = (bbMinY+bbMaxY)/2; //vertical midpoint of the bb of the blob

    for(int i=0; i < blob.size(); i++)
    {
        x = blob[i].x; //x coordinate of a point of the blob
        y = blob[i].y; //y coordinate of a point of the blob

        if(x <= bbMaxX && x > originX  && y < originY && y >= bbMinY)
        {
            //point is in quadrant 1
            pointRadius = std::sqrt(std::pow(x-originX, 2)+std::pow(originY-y, 2)); 
            error += abs(pointRadius - averageRadius);
        }

        else if(x <= bbMaxX && x > originX && y <= bbMaxY && y >= originY)
        {
            //point is in quadrant 2
            pointRadius = std::sqrt(std::pow(x-originX, 2)+std::pow(y-originY, 2));
            error += abs(pointRadius - averageRadius); 
        }

        else if(x >= bbMinX && x <= originX && y <=bbMaxY  &&  y >= originY)
        {
            //point is in quadrant 3
            pointRadius = std::sqrt(std::pow(originX-x, 2)+std::pow(y-originY, 2));
            error += abs(pointRadius - averageRadius);
        }

        else if(x >= bbMinX && x <= originX && y < originY && y >= bbMinY)
        {
            //point is in quadrant 4
            pointRadius = std::sqrt(std::pow(originX-x, 2)+std::pow(originY-y, 2));
            error += abs(pointRadius - averageRadius);
        }
    }

    error = error/blob.size();
    return error;
}
//}}}

/**
 * gets the distance from the NAO to a blob
 * x is the x coordinate for the point you want to calculate the distance to
 * y is the y coordinate for the point you want to calculate the distance to
 */
//TODO fix some things in movement
float FindBall::getDistanceToPoint(int x, int y, int camera, float headPitch, float cameraHeight, float someAngle)
//{{{
{
    float dist = 0;
    //from the bbox, get the point we want to calculate the distance to. this will be the bottom middle coordinate of the bbox
    distanceToBall = 0;
    float camFieldOfViewHorRad = CAM_FIELD_OF_VIEW_HOR * DEG2RAD;
    float camFieldOfViewVerRad = CAM_FIELD_OF_VIEW_VER * DEG2RAD;
    //get current head pitch, sumsum aldebaran
    //std::string names = "HeadPitch"; //do this in Movement
    //bool useSensors = true; //do this is Movement
    //float someAngle = motion.getAngles(names, useSensors); //do this in Movement
    int otherAngle = someAngle - 0.5 * CAM_FIELD_OF_VIEW_VER; //angle between ground and bottom of image
    //float cameraHeight = motion.getTransform(camera, 2, true)[11]; //dunno what I'm doing here. was in roel's code //TODO do this in Movement
    float x2 = SRC_WIDTH - x; //roel says: rotation counter clockwise
    x2 = x2 - SRC_WIDTH/2; //roel: relative to center of image
    float xAngle = (x2 / (SRC_WIDTH/2)) * (CAM_FIELD_OF_VIEW_VER/2) * DEG2RAD;
    float y2 = SRC_HEIGHT - y;
    float yAngle = someAngle + (y2/SRC_HEIGHT) * CAM_FIELD_OF_VIEW_VER * DEG2RAD;
    dist = cameraHeight * std::tan(yAngle) / std::cos(xAngle); 
    dist *= 1000; //from meters to mm

    return dist;
}
//}}}


/**
 * calculates the distance to a blob and updates distanceToBall
 *  
 *  @param[in] bbox the bounding box of a blob
 */
void FindBall::getDistanceToBlob(cv::Vec4i bbox, float headPitch, float cameraHeight, float someAngle)
//{{{
{
    int x = (bbox[0]+bbox[2])/2;
    int y = bbox[3];

    distanceToBall = getDistanceToPoint(x, y, headPitch, cameraHeight, someAngle);
}
//}}}


/**
 * returns true when the distance and size of the ball make sense
 *
 * @param[in] bbox the bounding box of a blob
 * @param[in] blobDistance the distance to a blob
 * @return true if the blob size to distance ratio is valid
 */
bool FindBall::isBlobLogicalBall(cv::Vec4i bbox, int blobDistance)
//{{{
{ 
    bool blobIsLogical = false;
    if(distanceToBall >= 0 && distanceToBall < FIELD_DIAGONAL)
    {
        float avgDiameterBbox = ((bbox[2]-bbox[0])+(bbox[3]-bbox[1]))/2;
        float expectedDiameterBbox = (- std::pow(blobDistance, 3) + std::pow(blobDistance, 2) - 1.55 * blobDistance + 118) * 10; //*10 for mm
        float error = std::abs(avgDiameterBbox - expectedDiameterBbox);
        if(error <= BALL_DISTANCE_ERROR)
        {
            blobIsLogical = true;
            return blobIsLogical;
        }
    }

    return blobIsLogical;
}
//}}}


/**
 * finds all blobs that are good candidates for being a ball
 *
 * @param foundContours all the found contours
 * @return returns a pair. a vector of Vec4i with coordinates of the bounding box of all possiballs. and the index indicating the detected ball
 */
std::pair<int, std::vector<cv::Vec4i> > FindBall::getAllCandidates(std::vector <std::vector<cv::Point> > foundContours)
//{{{
{
    std::vector<cv::Point> contour; //a contour in foundContours
    cv::Vec4i bbox; 
    float bboxSquareness; //a number representing how square the bb is 
    float squareLowerThreshold = 0.875; //ratio of 7/8, in this context this is considered square enough
    float squareUpperThreshold = 1.143; //ratio of 8/7, in this context this is considered square enough
    float blobError; 
    float blobErrorThreshold = 1.5;
    int minContourSize = 20; //or 30. Contour of blob has to contain at least this amount of pixels. Depends on the quality of the image and on the quality of the contour and the distance of the ball to the NAO. Smaller objects have smaller contours thus fewer points.
    std::vector< std::vector<cv::Point> > possiballs; //contains contours of the possible balls
    std::vector<cv::Vec4i> possiballsBbox; //contains the bounding box of the possible balls

    for(int i=0; i<foundContours.size(); i++)
    {
        contour = foundContours[i];

        if(contour.size() > minContourSize) 
        {
            bbox = getBoundingBox(contour);

            if(abs(bbox[0]-bbox[2])>0 && abs(bbox[1]-bbox[3])>0) //if contour is a not a straight line
            {
                float bboxSquareness = ((float)abs(bbox[0]-bbox[2]))/(float)(abs(bbox[1]-bbox[3]));

                if((bboxSquareness >= squareLowerThreshold) && (bboxSquareness <= squareUpperThreshold))
                {
                    blobError = getBlobError(bbox, foundContours[i]);

                    if(blobError <= blobErrorThreshold) //if blobError is below the threshold
                    {
                        //(cv::Vec4i bbox, float headPitch, float cameraHeight, float someAngle)
                        getDistanceToBlob();
                        if(isBlobLogicalBall(bbox, blobDistance)) //if the distance to ballsize ratio makes sense
                        {
                            possiballs.push_back(contour); //add contour to the list
                            possiballsBbox.push_back(bbox); //add boundingbox to the list

                        }
                    }
                }
            }
        }
    }

    //select the ball with the lowest error
    float bigNum = 10000;
    float err;
    int ind = 0; //the index of the ball with the lowest error
    cv::Vec4i theBall; //the blob with the lowest error

    if(possiballs.size() == 0) //no possiballs found
    {
        possiballsBbox.push_back(cv::Vec4i(0, 0, 0, 0));
    }
    else
    {
        for(int i=0; i<possiballs.size(); i++)
        {
            err = getBlobError(getBoundingBox(possiballs[i]), possiballs[i]);
            if(err < bigNum)
            { 
                bigNum = err;
                theBall = getBoundingBox(possiballs[i]);
                ind = i;
            }
        }
    }
    ballBoundingBox = possiballsBbox[ind]; //store the location of the found ball
    std::pair<int, std::vector<cv::Vec4i> > results = std::make_pair(ind, possiballsBbox);
    return results;
}
//}}}


/**
 * finds the contours of an image and draws it on the image 
 *
 * @param[in] src the source image 
 * @return returns a pair. a vector of vector with points of all detected contours. and a Mat object made from src with the drawn contours
 */
std::pair<std::vector <std::vector<cv::Point> >, cv::Mat> FindBall::getContours(cv::Mat src)
//{{{
{
    //_____Convolution with a Gaussian kernel to minimize noise
    cv::Mat blurred; //blurred image
    double sigmaX = 0; //Gaussian kernel standard deviation in X direction
    double sigmaY = 0; //Gaussian kernel standard deviation in Y direction
    int borderType = cv::BORDER_DEFAULT; //pixel extrapolation method
    int kernelSize = 1; //Gaussian kernel size. Select a small kernel size for the ball because it is a small object. too much blur will cause it to not be detected when it is far away.

    cv::GaussianBlur(src, blurred, cv::Size(kernelSize, kernelSize), sigmaX, sigmaY, borderType);

    //_____Apply Canny using sobel or scharr operator. TODO make scharr operator work
    int threshold1 = 140; //first hysteresis, searches for values above the threshold
    int threshold2 = 160; //second hysteresis, searches for values above the threshold that are close to the first threshold
    int sobel = 3; //sobel operator
    int scharr = -1; //scharr operator
    cv::Mat cannied; //image after canny edge detector 
    bool l2gradient = false;

    //using scharr operator, might be more accurate than sobel
    cv::Canny(src, cannied, threshold1, threshold2, sobel, l2gradient); //Finds edges in an image using the [Canny86] algorithm.

    //_____Use found canny edges to generate contours
    std::vector <std::vector<cv::Point> > foundContours; //the found contours
    std::vector <std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy; // containing information about the image topology
    int mode = CV_RETR_TREE; //Contour retrieval mode
    int method = CV_CHAIN_APPROX_SIMPLE; //Contour approximation method
    cv::Point offset = cv::Point(0,0);

    cv::findContours(cannied, contours, hierarchy, mode, method, offset); //finds contours in a binary image.

    foundContours = contours;

    cv::Mat drawing = cv::Mat::zeros(src.size(), CV_8UC3);

    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
        //note that drawing is the destination image
    }

    std::pair<std::vector <std::vector<cv::Point> >, cv::Mat> results = make_pair(foundContours, drawing);
    return results;
}
//}}}


/**
 * Draws a bounding box of a blob.
 */
void FindBall::drawThisBlob(cv::Vec4i ballPoints, cv::Mat img, cv::Scalar color)
//{{{
{
    line(img, cv::Point(ballPoints[0], ballPoints[1]), cv::Point(ballPoints[2], ballPoints[1]), color, 2, CV_AA); //scalar is in BGR order
    line(img, cv::Point(ballPoints[0], ballPoints[1]), cv::Point(ballPoints[0], ballPoints[3]), color, 2, CV_AA);
    line(img, cv::Point(ballPoints[2], ballPoints[3]), cv::Point(ballPoints[2], ballPoints[1]), color, 2, CV_AA);
    line(img, cv::Point(ballPoints[2], ballPoints[3]), cv::Point(ballPoints[0], ballPoints[3]), color, 2, CV_AA);
}
//}}}


/**
 * Draws all bounding boxes of blobs.
 */
void FindBall::drawTheseBlobs(std::vector<cv::Vec4i> theBall, int ind, cv::Mat contours)
//{{{
{
    cv::Scalar color;
    for(int i=0; i<theBall.size(); i++)
    {
        color = cv::Scalar(0, 0, 255);
        if(i == ind)
        {
            color = cv::Scalar(0, 255, 0);
        }
        std::cout << theBall[0] << " " << theBall[1] << " " << theBall[2] << " " << theBall[3] << std::endl;
        drawThisBlob(theBall[i], contours, color);
    }
}
//}}}


/**performs conversion from BGR color space to the specified color space and returns the converted image as an array of its 3 channels
 *
 * @param[in] src the image to be converged
 * @param[out] dst the converged image as an array of 3 channels
 * @param[in] colorSpace the colorspace you wish to convert to 
 */
void FindBall::cvtToAndSplit(cv::Mat src, cv::Mat dst[3], int colorSpace)
//{{{
{
    cv::Mat tmp;
    cv::cvtColor(src, tmp, colorSpace);
    cv::split(tmp, dst);
}
//}}}


/**
 * loads images from the file
 */
void FindBall::loadSrc(std::string filename, cv::Mat& src)
//{{{
{
    src = cv::imread(filename, 1);
    //it reads it
}
//}}}


/**
 * temporary method, makes it all come together.
 */
std::pair<cv::Vec4i, float> FindBall::finalize()
//{{{
{
    src = camera.getSrc();
    //camera.unsubscribeFromProxy(); //not sure if necessary
    //cv::Mat src ;//= cv::imread(filename, 1);

    //use HSV color space because during experiments it has shown to be best
    int colorSpaceHSV = CV_BGR2HSV;
    //int colorSpaceHLS = CV_BGR2HLS;
    //loadSrc(filename, src);

    std::pair<std::vector <std::vector<cv::Point> >, cv::Mat> contourResults;
    std::vector <std::vector<cv::Point> > foundContours;
    cv::Mat contours;
    cv::Mat dst[3];
    cv::Mat chosenChannel;
    cvtToAndSplit(src, dst, colorSpaceHSV);
    chosenChannel = dst[1]; //choose S channel from the HSV
    contourResults = getContours(chosenChannel);
    foundContours = contourResults.first;
    contours = contourResults.second;

    std::pair<int, std::vector<cv::Vec4i> > balls = getAllCandidates(foundContours);
    std::vector<cv::Vec4i> theBall = balls.second;
    int ind = balls.first;

    drawTheseBlobs(theBall, ind, contours);

    if(foundContours.size() > 0)
    {
        cv::imshow("detected contours" + filename, contours);
    }

    std::pair<cv::Vec4i, float> results = std::make_pair(ballBoundingBox, distanceToBall);
    return results;
}
//}}}


int main()
{
    FindBall ball;
    ball.finalize();

    cv::waitKey(0);
    return 0;
}
