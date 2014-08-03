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

        void loadSrcs(int amount, std::string filenames[29], cv::Mat srcs[29]);

    private:
        const int histSize = 256;
        //std::string filename = "/home/redsphinx/Projects/BachelorThesis/TestImages7/kitchen1.jpg";
        std::string filename1 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/2-1-headpitch-02";
        std::string filename2 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/2-2-headpitch-02";
        std::string filename3 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/2-4-headpitch-02";
        std::string filename4 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/2-5-headpitch-02";
        std::string filename5 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/2-6-headpitch-02";
        std::string filename6 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/2-8-headpitch-02";
        std::string filename7 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/2-goal-headpitch-02";
        std::string filename8 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-2-headpitch";
        std::string filename9 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-1-headpitch-01";
        std::string filename10 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-2-headpitch-05";
        std::string filename11 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-5-headpitch-01";
        std::string filename12 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-6-headpitch-02";
        std::string filename13 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-8-headpitch-05";
        std::string filename14 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-goal-headpitch-01";
        std::string filename15 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/4-1-headpitch-01";
        std::string filename16 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/4-2-headpitch-01";
        std::string filename17 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/4-5-headpitch-01";
        std::string filename18 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/4-6-headpitch-05";
        std::string filename19 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/4-8-headpitch-04";
        std::string filename20 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/4-goal-headpitch-01";
        std::string filename21 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-1-headpitch-05";
        std::string filename22 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-2-headpitch-02";
        std::string filename23 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-3-headpitch-02";
        std::string filename24 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-4-headpitch-02";
        std::string filename25 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-5-headpitch-02";
        std::string filename26 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-6-headpitch-02";
        std::string filename27 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-7-headpitch-02";
        std::string filename28 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-8-headpitch-02";
        std::string filename29 = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-goal-headpitch-02";

        std::string filenames[29] = {filename1, filename2, filename3, filename4, filename5, filename6, filename7, filename8, filename9, filename10, filename11, filename12, filename13, filename14, filename15, filename16, filename17, filename18, filename19, filename20, filename21, filename22, filename23, filename24, filename25, filename26, filename27, filename28, filename29};
        //cv::Mat srcs[28] = {cv::Mat, cv::Mat,  cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, };
        //cv::Mat srcs[28];
        cv::RNG rng;
        //RNG rng(12345);
        const float PI = 3.14159265358979323846;
        //const float PERFECT_RATIO = PI/4;

};


