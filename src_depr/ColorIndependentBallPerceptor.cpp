#include "ColorIndependentBallPerceptor.h"

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
    float th = 1.5;
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
 * Outputs the contour.
 *
 * @param[in] vec  The vector with the vector of Points of all the created contours.  
 * @return  void
 */
void ColorIndependentBallPerceptor::printContour(std::vector <std::vector<cv::Point> > vec)
{
    std::cout << "___Contour size: " << vec.size() << std::endl;
    for(int i=0; i<vec.size(); i++)
    {
        std::vector<cv::Point> vec2 = vec[i];
        std::cout <<i <<" "<<  vec2 << std::endl;
    }
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

    //cv::blur(src, srcBlur, cv::Size(3,3)); //blurs an image using the normalized box filter.
    //cv::GaussianBlur(src, srcBlur, cv::Size(3,3), sigmaX, sigmaY, borderType = cv::BORDER_DEFAULT); //blurs an image using a Gaussian filter.
    //cv::imshow("GaussianBlur", srcBlur);

    //default = srcBlur
    
    cv::Canny(src, dest, 400, 500, 3, false); //50, 100. Finds edges in an image using the [Canny86] algorithm.

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

cv::Mat ColorIndependentBallPerceptor::normalize(cv::Mat channel, int track)
{
    for(int i=0; i<channel.cols; i++)
    {
        for(int j=0; j<channel.rows; j++)
        {
            int pixel = channel.at<unsigned char>(i, j);
            channel.at<unsigned char>(i, j) = (255/track) * pixel;

        }
    }
    return channel;
}

/**
 * Scales channel value.
 * 5 different operation modes: 0. threshold. 1. quadratic. 2. power. 3. exponential. 4. sinesoidal.
 *
 * @param[in] channel  The targeted channel.
 * @param[in] image    The input image. 
 * @param[in] operation  The level of operation. 
 * @return   Returns the processed channel.
 */
//cv::Mat ColorIndependentBallPerceptor::scaleChannelValuesBy(int channel, cv::Mat image[3], int operation)
void ColorIndependentBallPerceptor::scaleChannelValuesBy(cv::Mat chosenChannel, int operation)
{
    std::cout << "___Scaling values. Operation used: " << operation <<  std::endl;
    //std::cout << "___Scaling values of channel: " << channel << ". Operation used: " << operation <<  std::endl;
    float track = 0; //keeps track of the highest value. 
    //cv::Mat chosenChannel = image[channel];


    //threshold
    if(operation == 0)
    {
        float a = 2; //scale, default=2
        float b = 10; //threshold, default=90

        for(int i=0; i<chosenChannel.rows; i++)
        {
            for(int j=0; j<chosenChannel.cols; j++)
            {

                int pixel = chosenChannel.at<unsigned char>(i, j);
                if(chosenChannel.at<unsigned char>(i, j) < b)
                {
                    chosenChannel.at<unsigned char>(i, j) /= a;
                }
                else
                {
                    chosenChannel.at<unsigned char>(i, j) *= a;
                }

                if(chosenChannel.at<unsigned char>(i, j) > track)
                {
                    track = chosenChannel.at<unsigned char>(i,j);
                }
            }
        }
    }


    //quadratic
    if(operation == 1)
    {
        int a = 100; //default=1
        int b = 50; //default=1
        int c = 0; //default=0
        for(int i=0; i<chosenChannel.rows; i++)
        {
            for(int j=0; j<chosenChannel.cols; j++)
            {

                int pixel = chosenChannel.at<unsigned char>(i, j);
                chosenChannel.at<unsigned char>(i, j) = a * std::pow(pixel, 2) + b * pixel + c; 

                if(chosenChannel.at<unsigned char>(i, j) > track)
                {
                    track = chosenChannel.at<unsigned char>(i,j);
                }
            }
        }
    }

    //std::power
    else if(operation == 2)
    {

        int a = 1; //default=1
        int b = 3; //default=3
        for(int i=0; i<chosenChannel.rows; i++)
        {
            for(int j=0; j<chosenChannel.cols; j++)
            {

                int pixel = chosenChannel.at<unsigned char>(i, j);
                chosenChannel.at<unsigned char>(i, j) = a * std::pow(pixel, b);

                if(chosenChannel.at<unsigned char>(i, j) > track)
                {
                    track = chosenChannel.at<unsigned char>(i,j);
                }
            }
        }
    }

    //exponential
    else if(operation == 3)
    {
        float a = 200; //default=1
        float b = 1.001; //default=2, >1
        for(int i=0; i<chosenChannel.rows; i++)
        {
            for(int j=0; j<chosenChannel.cols; j++)
            {

                float pixel = chosenChannel.at<unsigned char>(i, j);
                //std::cout << "old: " << pixel << std::endl;
                chosenChannel.at<unsigned char>(i, j) = a * std::pow(b, pixel);
                //std::cout << "new: " << a*std::pow(b, pixel) << std::endl;

                if(chosenChannel.at<unsigned char>(i, j) > track)
                {
                    track = chosenChannel.at<unsigned char>(i,j);
                }
            }
        }
    }

    //sinesoidal
    else if(operation == 4)
    {

        int a = 250; //default=1
        int b = 1; //default=1
        int c = PI; //default=1
        for(int i=0; i<chosenChannel.rows; i++)
        {
            for(int j=0; j<chosenChannel.cols; j++)
            {

                int pixel = chosenChannel.at<unsigned char>(i, j);
                chosenChannel.at<unsigned char>(i, j) = a * std::sin(b * pixel + c);

                if(chosenChannel.at<unsigned char>(i, j) > track)
                {
                    track = chosenChannel.at<unsigned char>(i,j);
                }
            }
        }
    }

    normalize(chosenChannel, track);
}


void ColorIndependentBallPerceptor::scaleBGRChannelValuesBy(cv::Mat src[3], int operation)
{
    for(int i=0; i<3; i++)
    {
        scaleChannelValuesBy(src[i], operation);
    }
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

// Perform in-place unsharp masking operation
void ColorIndependentBallPerceptor::unsharpMask(cv::Mat src) 
{
    //cv::Mat src = cv::imread(filename, 1);

    double alpha = 7;
    double beta = 1 - alpha;
    double gamma = 0;
    cv::Mat tmp;
    cv::GaussianBlur(src, tmp, cv::Size(7,7), 8);
    cv::addWeighted(src, alpha, tmp, beta, gamma, src);

    //cv::imshow("sharpened", src);
}

//perform in-place unsharp masking operation with an image containing edges instead of a gaussian blur
void ColorIndependentBallPerceptor::unsharpMask2(cv::Mat src)
{
    double alpha = -15;
    double beta = 1 - alpha;
    double gamma = 50;
    cv::Mat tmp = src;
    //perform first unsharp mask with the gaussian 
    unsharpMask(tmp);
    cv::imshow("after first unsharpMask", tmp);
    //split image
    cv::Mat splittedTmp[3];
    cv::split(tmp, splittedTmp);
    //scale channel values
    int operation = 0;
    scaleBGRChannelValuesBy(splittedTmp, operation);
    //nullify the blue channel. this channel contains a lot of noise
    splittedTmp[0] = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    //merge back the splitted image
    cv::Mat mergedTmp;
    cv::merge(splittedTmp, 3, mergedTmp);
    //perfom unsharpmask with the "edges"
    cv::addWeighted(src, alpha, mergedTmp, beta, gamma, src);
}



void ColorIndependentBallPerceptor::hsv(std::string filename, cv::Mat splitImg[3])
{
    cv::Mat img, trf;
    img = cv::imread(filename, 1);
    cv::cvtColor(img, trf, CV_BGR2HSV);
    cv::split(trf, splitImg);
}

void ColorIndependentBallPerceptor::hls(std::string filename, cv::Mat splitImg[3])
{
    cv::Mat img, trf;
    img = cv::imread(filename, 1);
    cv::cvtColor(img, trf, CV_BGR2HLS);
    cv::split(trf, splitImg);
}

/**
 * Makes it all come together.
 *
 * @return void 
 */
void ColorIndependentBallPerceptor::finalize()
{
    std::cout << "Here" << std::endl;
    std::pair<std::vector <std::vector<cv::Point> >, cv::Mat> contourResults;
    std::vector <std::vector<cv::Point> > foundContours;
    cv::Mat contours;
    //cv::Mat src = cv::imread(filename, 1); 
    //cv::imshow("original", src);
    //cv::Mat gausBlur;
    // 0 = threshold
    // 1 = quadratic
    // 2 = power
    // 3 = exponential
    // 4 = sin - sucks dont use it
    int operation = 0;
    int amount = 28;
    std::cout << "HERE" << std::endl;

    ////load all images in the Mat array
    //for(int i=0; i<amount; i++)
    //{
        //srcs[i] = cv::imread(filenames[i], 1);
    //}

    ////for each image, perform the algrithm and show detected contours
    //for(int i=0; i<amount; i++)
    //{
        //std::cout << i << std::endl;
        //cv::Mat splitted[3];
        //cv::Mat src = srcs[i];
        //hsv(filenames[i], splitted);
        //cv::Mat chosenChannel = splitted[1];

        //contourResults = contourThisShit(chosenChannel);

        //foundContours = contourResults.first;
        //contours = contourResults.second;

        ////cv::Vec4i theBall = findBall(foundContours);
        //std::pair<int, std::vector<cv::Vec4i> > balls = findBall(foundContours);
        //std::vector<cv::Vec4i> theBall = balls.second;
        //int ind = balls.first;

        //drawTheseBlobs(theBall, ind, contours);

        ////cv::imshow("channel", splittedImage[channel]);
        //if(foundContours.size() > 0)
        //{
            //cv::imshow("detected contours" + filenames[i], contours);
        //}
    
    //}
    
    //--- 
    //cv::Mat splittedImage[3]; //array of Mat objects, each is a channel


    //for(int i=0; i<1; i++)
    //{
        //unsharpMask(src);
    //}
    //cv::split(src, splittedImage);

    //scaleBGRChannelValuesBy(splittedImage, operation);
    ////cv::Mat scaledChannel = scaleChannelValuesBy(src,  operation); //modifies src
    ////cv::Mat scaledChannel = scaleChannelValuesBy(channel, splittedImage, operation); //modifies splittedImage
    
    ////splittedImage[0] = cv::Mat(src.rows, src.cols, CV_8UC1, 0); 
    //splittedImage[0] = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    
    //cv::Mat afterMerge;
    
    //cv::merge(splittedImage, 3, afterMerge);

    //cv::GaussianBlur(afterMerge, gausBlur, cv::Size(3,3), 20, 20, cv::BORDER_DEFAULT); //blurs an image using a Gaussian filter.
    //cv::imshow("after blur", gausBlur);
    //for(int i=0; i<1; i++)
    //{
        //unsharpMask(gausBlur);
    //}

    //cv::Mat afterMergeGrey;
    //cv::cvtColor(gausBlur, afterMergeGrey, CV_BGR2GRAY );
    //cv::imshow("afterMergeGrey", afterMergeGrey);
    ////whiten the colored pixels on the image for maximum contrast
    //for(int i=0; i<afterMergeGrey.rows; i++)
    //{
        //for(int j=0; j<afterMergeGrey.cols; j++)
        //{
            //if(afterMergeGrey.at<unsigned char>(i, j) > 9)
            //{
                //std::cout << "HERE" << std::endl;
                //afterMergeGrey.at<unsigned char>(i, j) = 255;
            //}
            //else
            //{
                //afterMergeGrey.at<unsigned char>(i, j) = 0;
            //}
        //}
    //}

    //cv::imshow("afterMergeGrey whitened", afterMergeGrey);

    //unsharpMask2(src);
    //cv::imshow("after unsharpMask2", src);
    //unsharpMask(src);
    //cv::imshow("after unsharpMask", src);
    //---


}



//void ColorIndependentBallPerceptor::update(BallPercept& ballPercept)
//{
//}

int main()
{
    ColorIndependentBallPerceptor ball;
    std::cout << "HRE" << std::endl;
    ball.finalize();
    //ball.unsharpMask();





    cv::waitKey(0);
    return 0;
}
