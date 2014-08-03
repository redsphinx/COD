#pragma once

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include <tuple>

/*#include "Tools/Module/Module.h"*/
//#include "Representations/Infrastructure/Image.h"
//#include "Representations/Infrastructure/FrameInfo.h"
//#include "Representations/Perception/BallPercept.h"


/*MODULE(ColorIndependentBallPerceptor)*/
//REQUIRES(Image)
//REQUIRES(FrameInfo) // You can use this to skip on certain iterations

//PROVIDES(BallPercept)
//END_MODULE


class ColorIndependentBallPerceptor 
//class ColorIndependentBallPerceptor : public ColorIndependentBallPerceptorBase
{
    public:
        /**
         * Returns the bounding box of a blob.
         *
         * @param blob  A possible ball. Contour with a roundish shape.
         * @return a Vec4i with top-left and bottom-right coordinate.
         */
        cv::Vec4i makeBB(std::vector<cv::Point> blob);

        /**
         * Returns the error of a blob.
         *
         * @param[in] bbox  The bounding box of a blob.
         * @param[in] blob  A blob.
         * @return the error of a blob.
         */
        float blobError(cv::Vec4i bbox, std::vector<cv::Point> blob);


        /**
         * Finds all blobs that are good candidates for being a ball.
         *
         * @param foundContours  All the found contours
         * @return  Returns a pair. A vector of Vec4i with coordinates of the bounding box of all possiballs. And the index indicating the detected ball.
         */
        std::pair<int, std::vector<cv::Vec4i>> findBall(std::vector<std::vector<cv::Point>> foundContours);


        /**
         * Outputs the contour.
         *
         * @param[in] vec  The vector with the vector of Points of all the created contours.  
         * @return  void
         */
        void printContour(std::vector<std::vector<cv::Point>> vec);


        /**
         * Finds the contours of an image. And draws it on the image. 
         *
         * @param[in] src  The source image. 
         * @return  Returns a pair. A vector of vector with points of all detected contours. And a Mat object made from src.
         */
        std::pair<std::vector<std::vector<cv::Point>>, cv::Mat> contourThisShit(cv::Mat src);

        cv::Mat normalize(cv::Mat channel, int track);

        /**
         * Scales channel value.
         * 5 different operation modes: 0. threshold. 1. quadratic. 2. power. 3. exponential. 4. sinesoidal.
         *
         * @param[in] channel  The targeted channel.
         * @param[in] image    The input image. 
         * @param[in] operation  The level of operation. 
         * @return   Returns the processed channel.
         */
        //cv::Mat scaleChannelValuesBy(int channel, cv::Mat image[3], int operation);
        void scaleChannelValuesBy(cv::Mat src, int operation);

        void scaleBGRChannelValuesBy(cv::Mat src[3], int operation);


        /**
         * Draws a bounding box of a blob.
         * 
         * @param[in] ballPoints  The coordinates.
         * @param[in] img         The image to draw on. 
         * @param[in] color       The color.
         * @return  void
         */
        void drawThisBlob(cv::Vec4i ballPoints, cv::Mat img, cv::Scalar color);


        /**
         * Draws all bounding boxes of blobs.
         *
         * @param[in] theBall  The vector of blob bounding box coordinates. 
         * @param[in] ind   The index of the detected ball.
         * @param[in] contours  The image to draw on. 
         * @return  void
         */
        void drawTheseBlobs(std::vector<cv::Vec4i> theBall, int ind, cv::Mat contours);


        /**
         * Makes it all come together.
         *
         * @return void 
         */
        void finalize();

        void unsharpMask(cv::Mat src);

        void unsharpMask2(cv::Mat src);

        //void update(BallPercept& ballPercept);
        
        void hsv(std::string filename, cv::Mat splitImg[3]);

        void hls(std::string filename, cv::Mat splitImg[3]);

    private:
        const int histSize = 256;
        //std::string filename = "/home/redsphinx/Projects/BachelorThesis/TestImages7/kitchen1.jpg";
        std::string filename = "/home/redsphinx/Projects/BachelorThesis/TestImages8/1.jpg";
        std::string filename2 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/2.jpg";
        std::string filename3 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/3.jpg";
        std::string filename4 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/4.jpg";
        std::string filename5 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/5.jpg";
        std::string filename6 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/6.jpg";
        std::string filename7 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/7.jpg";
        std::string filename8 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/8.jpg";
        std::string filename9 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/9.jpg";
        std::string filename10 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/10.png";
        std::string filename11 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/11.png";
        std::string filename12 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/12.png";
        std::string filename13 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/13.png";
        std::string filename14 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/14.png";
        std::string filename15 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/15.png";
        std::string filename16 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/16.png";
        std::string filename17 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/17.png";
        std::string filename18 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/18.png";
        std::string filename19 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/19.png";
        std::string filename20 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/20.png";
        std::string filename21 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/21.png";
        std::string filename22 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/22.png";
        std::string filename23 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/23.png";
        std::string filename24 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/24.png";
        std::string filename25 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/25.png";
        std::string filename26 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/26.png";
        std::string filename27 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/27.png";
        std::string filename28 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/28.png";
        std::string filename29 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/29.png";
        std::string filename30 = "/home/redsphinx/Projects/BachelorThesis/TestImages8/30.png";

        std::string filenames[27] = {filename, filename2, filename3, filename4, filename5, filename6, filename7, filename8, filename9, filename10, filename11, filename12, filename13, filename14, filename15, filename16, filename17, filename18, filename19, filename20, filename21, filename22, filename23, filename24, filename25, filename26, filename28};
        //cv::Mat srcs[28] = {cv::Mat, cv::Mat,  cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, };
        //cv::Mat srcs[28];
        cv::RNG rng;
        //RNG rng(12345);
        const float PI = 3.14159265358979323846;
        //const float PERFECT_RATIO = PI/4;

};


