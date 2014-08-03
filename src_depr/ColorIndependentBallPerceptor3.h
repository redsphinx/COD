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
         * Finds the contours of an image. And draws it on the image. 
         *
         * @param[in] src  The source image. 
         * @return  Returns a pair. A vector of vector with points of all detected contours. And a Mat object made from src.
         */
        std::pair<std::vector<std::vector<cv::Point>>, cv::Mat> contourThisShit(cv::Mat src);


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

        //void update(BallPercept& ballPercept);
        

        void cvtToAndSplit(cv::Mat src, cv::Mat dst[3], int colorSpace);

        void loadSrcs(int amount, std::string filenames[6], cv::Mat srcs[6]);

        cv::Mat scaleValues(cv::Mat src, int scale, int threshold);

    private:
        const int histSize = 256;
        //std::string filename = "/home/redsphinx/Projects/BachelorThesis/TestImages7/kitchen1.jpg";
        std::string filename = "/home/redsphinx/Desktop/redLeft.jpg";
        std::string filename2 = "/home/redsphinx/Desktop/redRight.jpg";
        std::string filename3 = "/home/redsphinx/Desktop/orangeLeft.jpg";
        std::string filename4 = "/home/redsphinx/Desktop/orangeRight.jpg";
        std::string filename5 = "/home/redsphinx/Desktop/yellowLeft.jpg";
        std::string filename6 = "/home/redsphinx/Desktop/yellowRight.jpg";


        std::string filenames[6] = {filename, filename2, filename3, filename4, filename5, filename6};
        //cv::Mat srcs[28] = {cv::Mat, cv::Mat,  cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, };
        //cv::Mat srcs[28];
        cv::RNG rng;
        //RNG rng(12345);
        const float PI = 3.14159265358979323846;
        //const float PERFECT_RATIO = PI/4;

};


