#include "ColorIndependentBallPerceptor3.h"

// For debugging:
//#include "Tools/Debugging/Debugging.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

//MAKE_MODULE(ColorIndependentBallPerceptor, Perception)

/**
 * Returns the bounding box of a blob.
 *
 * @param[in] blob  A possible ball. Contour with a roundish shape.
 * @return a Vec4i with top-left and bottom-right coordinate.
 */
cv::Vec4i ColorIndependentBallPerceptor::makeBB(std::vector<cv::Point> blob)
{
    //std::cout << "___Making bounding box." << std::endl;
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


/**
 * Returns the error of a blob.
 *
 * @param[in] bbox  The bounding box of a blob.
 * @param[in] blob  A blob.
 * @return the error of a blob.
 */
float ColorIndependentBallPerceptor::blobError(cv::Vec4i bbox, std::vector<cv::Point> blob)
{
    //std::cout << "___Calculating blob error\n";
    float error = 0;
    int x, y;
    int bbMaxX = std::max(bbox[0], bbox[2]);
    int bbMinX = std::min(bbox[0], bbox[2]);
    int bbMaxY = std::max(bbox[1], bbox[3]);
    int bbMinY = std::min(bbox[1], bbox[3]);
    float averageRadius = ((bbMaxX-bbMinX) + (bbMaxY-bbMinY))/4; //the average raduis of the blob 
    float pointRadius;
    int originX = (bbMaxX+bbMinX)/2;
    int originY = (bbMinY+bbMaxY)/2;

    for(int i=0; i < blob.size(); i++)
    {
        x = blob[i].x;
        y = blob[i].y;
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
        else
        {
            std::cout << "____Something went terribly wrong.\n";
        }
    }
    error = error/blob.size();
    //std::cout << "____Error: " << error << std::endl;
    return error;
}

/**
 * Finds all blobs that are good candidates for being a ball.
 *
 * @param foundContours  All the found contours
 * @return  Returns a pair. A vector of Vec4i with coordinates of the bounding box of all possiballs. And the index indicating the detected ball.
 */
std::pair<int, std::vector<cv::Vec4i> > ColorIndependentBallPerceptor::findBall(std::vector <std::vector<cv::Point> > foundContours)
{
    std::cout << "___Searching for the ball.\n";
    cv::Vec4i ballPoints;
    std::vector<cv::Point> contour;
    int blobSurface;
    cv::Vec4i bbox;
    float bboxSquareness;
    float error;
    float th = 2.0;
    int minSize = 20; //or 30. Contour of blob has to contain at least this amount of pixels. 
    std::vector< std::vector<cv::Point> > possiballs; //contains contours of the possible balls
    std::vector<cv::Vec4i> possiballsBbox; //contains the bounding box of the possible balls

    for(int i=0; i<foundContours.size(); i++)
    {
        contour = foundContours[i];
        if(contour.size() > minSize)
        {
            bbox = makeBB(contour);

            if(abs(bbox[0]-bbox[2])>0 && abs(bbox[1]-bbox[3])>0) //if contour is a not a straight line
            {
                float bboxSquareness = ((float)abs(bbox[0]-bbox[2]))/(float)(abs(bbox[1]-bbox[3]));

                if((bboxSquareness >= 0.7) && (bboxSquareness <= 1.3)) //originally 0.8 and 1.2
                {
                    error = blobError(bbox, foundContours[i]);

                    if(error <= th)
                    {
                        std::cout << "____Possiball at i = " << i << std::endl;
                        possiballs.push_back(contour);
                        possiballsBbox.push_back(bbox);
                    }
                }
            }
        }
    }
    //select the ball with the lowest error
    float mm = 10000;
    float tmp;
    int ind = 0; //the index of the ball with the lowest error
    cv::Vec4i theBall;

    if(possiballs.size() == 0)
    {
        std::cout << "_____No possiballs.\n";
        possiballsBbox.push_back(cv::Vec4i(0, 0, 0, 0));
    }
    else
    {
        for(int i=0; i<possiballs.size(); i++)
        {
            tmp = blobError(makeBB(possiballs[i]), possiballs[i]);
            if(tmp < mm)
            { 
                mm = tmp;
                theBall = makeBB(possiballs[i]);
                ind = i;
            }
            std::cout << "____Error: " << tmp << std::endl;
        }
    }
    std::pair<int, std::vector<cv::Vec4i> > results = make_pair(ind, possiballsBbox);
    return results;
}


/**
 * Finds the contours of an image. And draws it on the image. 
 *
 * @param[in] src  The source image. 
 * @return  Returns a pair. A vector of vector with points of all detected contours. And a Mat object made from src.
 */
std::pair<std::vector <std::vector<cv::Point> >, cv::Mat> ColorIndependentBallPerceptor::contourThisShit(cv::Mat src)
{
    std::cout << "___Finding the contours\n";
    std::vector <std::vector<cv::Point> > foundContours;
    cv::Mat srcBlur;
    cv::Mat dest, convDest;
    double sigmaX = 0;
    double sigmaY = 0;
    int borderType;
    int kernelSize = 17;
    int sobel = 3;
    int scharr = -1;

    cv::GaussianBlur(src, srcBlur, cv::Size(kernelSize, kernelSize), sigmaX, sigmaY, borderType = cv::BORDER_DEFAULT); //blurs an image using a Gaussian filter.
    
    int threshold1 = 140; //first hysteresis, searches for values above the threshold
    int threshold2 = 160; //second hysteresis, searches for values above the threshold that are close to the first threshold

    //using scharr operator, might be more accurate than sobel
    cv::Canny(src, dest, threshold1, threshold2, sobel, false); //Finds edges in an image using the [Canny86] algorithm.

    //cv::imshow("canny edges", dest);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( dest, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) ); //finds contours in a binary image.

    foundContours = contours;
    //printContour(contours);

    cv::Mat drawing = cv::Mat::zeros( dest.size(), CV_8UC3 );

    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
    }

    std::pair<std::vector <std::vector<cv::Point> >, cv::Mat> results = make_pair(foundContours, drawing);
    return results;
}


/**
 * Draws a bounding box of a blob.
 * 
 * @param[in] ballPoints  The coordinates.
 * @param[in] img         The image to draw on. 
 * @param[in] color       The color.
 */
void ColorIndependentBallPerceptor::drawThisBlob(cv::Vec4i ballPoints, cv::Mat img, cv::Scalar color)
{
    //std::cout << "___Drawing a blob\n";
    line(img, cv::Point(ballPoints[0], ballPoints[1]), cv::Point(ballPoints[2], ballPoints[1]), color, 2, CV_AA); //scalar is in BGR order
    line(img, cv::Point(ballPoints[0], ballPoints[1]), cv::Point(ballPoints[0], ballPoints[3]), color, 2, CV_AA);
    line(img, cv::Point(ballPoints[2], ballPoints[3]), cv::Point(ballPoints[2], ballPoints[1]), color, 2, CV_AA);
    line(img, cv::Point(ballPoints[2], ballPoints[3]), cv::Point(ballPoints[0], ballPoints[3]), color, 2, CV_AA);

    //draw left ballPos
    line(img, cv::Point(173, 257), cv::Point(272, 257), cv::Scalar(100,100,100), 2, CV_AA);
    line(img, cv::Point(173, 257), cv::Point(173, 354), cv::Scalar(100,100,100), 2, CV_AA);
    line(img, cv::Point(272, 354), cv::Point(173, 354), cv::Scalar(100,100,100), 2, CV_AA);
    line(img, cv::Point(272, 354), cv::Point(272, 257), cv::Scalar(100,100,100), 2, CV_AA);
    
    //draw right ballPos
    line(img, cv::Point(381, 256), cv::Point(479, 256), cv::Scalar(100,100,100), 2, CV_AA);
    line(img, cv::Point(381, 256), cv::Point(381, 353), cv::Scalar(100,100,100), 2, CV_AA);
    line(img, cv::Point(479, 353), cv::Point(381, 353), cv::Scalar(100,100,100), 2, CV_AA);
    line(img, cv::Point(479, 353), cv::Point(479, 256), cv::Scalar(100,100,100), 2, CV_AA);
}

/**
 * Draws all bounding boxes of blobs.
 *
 * @param[in] theBall  The vector of blob bounding box coordinates. 
 * @param[in] ind   The index of the detected ball.
 * @param[in] contours  The image to draw on. 
 *
 */
void ColorIndependentBallPerceptor::drawTheseBlobs(std::vector<cv::Vec4i> theBall, int ind, cv::Mat contours)
{
    cv::Scalar color;
    for(int i=0; i<theBall.size(); i++)
    {
        color = cv::Scalar(0, 0, 255);
        if(i == ind)
        {
            color = cv::Scalar(0, 255, 0);
        }
        drawThisBlob(theBall[i], contours, color);
    }
}

//performs conversion from BGR color space to the specified color space and returns the converted image as an array of it's 3 channels
void ColorIndependentBallPerceptor::cvtToAndSplit(cv::Mat src, cv::Mat dst[3], int colorSpace)
{
    cv::Mat tmp;
    cv::cvtColor(src, tmp, colorSpace);
    cv::split(tmp, dst);
}

//loads images from the file
void ColorIndependentBallPerceptor::loadSrcs(int amount, std::string filenames[6], cv::Mat srcs[6])
{
    for(int i=0; i<amount; i++)
    {
        srcs[i] = cv::imread(filenames[i], 1);
    }
}


cv::Mat ColorIndependentBallPerceptor::scaleValues(cv::Mat src, int scale, int threshold)
{
    float track = 0.0;
    cv::Mat srcTmp = src;
    for(int i=0; i<src.rows; i++)
    {
        for(int j=0; j<src.cols; j++)
        {
            if(srcTmp.at<unsigned char>(i, j) > threshold)
            {
                //srcTmp.at<unsigned char>(i, j) *= scale;
                
                srcTmp.at<unsigned char>(i, j)  = 255;

            }
            else
            {
                //srcTmp.at<unsigned char>(i, j) /= scale;
                srcTmp.at<unsigned char>(i, j)  = 0;

            }

            //if(srcTmp.at<unsigned char>(i, j) > track)
            //{
                //track = srcTmp.at<unsigned char>(i, j);
            
            //}
        }
    }

    //for(int i=0; i<srcTmp.cols; i++)
    //{
        //for(int j=0; j<srcTmp.rows; j++)
        //{
            //srcTmp.at<unsigned char>(i, j)  = (255/track) * srcTmp.at<unsigned char>(i, j) ;
        //}
    //}

    return srcTmp;
}


/**
 * Makes it all come together.
 *
 * @return void 
 */
void ColorIndependentBallPerceptor::finalize()
{

    int amount = 6;
    cv::Mat srcs[amount];
    int colorSpaceHSV = CV_BGR2HSV;
    int colorSpaceHLS = CV_BGR2HLS;
    loadSrcs(amount, filenames, srcs);

    std::pair<std::vector <std::vector<cv::Point> >, cv::Mat> contourResults;
    std::vector <std::vector<cv::Point> > foundContours;
    cv::Mat contours;
    cv::Mat dst[3];
    cv::Mat chosenChannel;
    cv::Mat scaled;
    for(int i=0; i<amount; i++)
    {
        cvtToAndSplit(srcs[i], dst, colorSpaceHSV);
        
        chosenChannel = dst[1]; //choose S channel from the HSV

        scaled = scaleValues(chosenChannel, 2,  110);

        cv::imshow(filenames[i], chosenChannel);
        contourResults = contourThisShit(chosenChannel);
        foundContours = contourResults.first;
        contours = contourResults.second;

        std::pair<int, std::vector<cv::Vec4i> > balls = findBall(foundContours);
        std::vector<cv::Vec4i> theBall = balls.second;
        int ind = balls.first;

        drawTheseBlobs(theBall, ind, contours);

        if(foundContours.size() > 0)
        {
            cv::imshow("detected contours" + filenames[i], contours);
        }
    }
}


int main()
{
    ColorIndependentBallPerceptor ball;
    ball.finalize();

    cv::waitKey(0);
    return 0;
}
