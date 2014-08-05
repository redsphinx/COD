#pragma once
#include <opencv2/core/core.hpp>
#include "Camerastuff.h"
#include <string>
#include <vector>
#include <tuple>


class FindBall 
{
    private:
///{{{
        const int histSize = 256;
        std::string filename = "/home/redsphinx/Projects/BachelorThesis/TestImages9/3-1-headpitch-01";
        //cv::Mat src;
        cv::RNG rng;
        //RNG rng(12345);
        cv::Vec4i ballBoundingBox;
        float distanceToBall; //in mm
        Camerastuff camera;
        cv::Mat src;
///}}}

    public:
///{{{
        /**
         * returns the bounding box of a blob
         *
         * @param[in] blob a contour with a roundish shape
         * @return the bounding box with top-left and bottom-right coordinate
         */
        cv::Vec4i getBoundingBox(std::vector<cv::Point> blob);


        /**
         * returns the error of a blob
         *
         * @param[in] bbox he bounding box of a blob
         * @param[in] blob a blob is a contour, which is a collection of points. a contour defines the outline of a found round-ish object
         * @return the error of a blob
         */
        float getBlobError(cv::Vec4i bbox, std::vector<cv::Point> blob);

        /**
         * gets the distance from the NAO to a blob
         *
         */
        float getDistanceToPoint(int x, int y, int camera, float headPitch);


        float getDistanceToBlob(cv::Vec4i bbox);

        /**
         * returns true when the distance and size of the ball make sense
         *
         */
        bool isBlobLogicalBall(cv::Vec4i bbox, int blobDistance);

        /**
         * finds all blobs that are good candidates for being a ball
         *
         * @param foundContours all the found contours
         * @return returns a pair. a vector of Vec4i with coordinates of the bounding box of all possiballs. and the index indicating the detected ball
         */
        //TODO add distance heuristic
        std::pair<int, std::vector<cv::Vec4i>> getAllCandidates(std::vector<std::vector<cv::Point>> foundContours);


        /**
         * finds the contours of an image and draws it on the image 
         *
         * @param[in] src the source image 
         * @return returns a pair. a vector of vector with points of all detected contours. and a Mat object made from src with the drawn contours
         */
        std::pair<std::vector<std::vector<cv::Point>>, cv::Mat> getContours(cv::Mat src);


        /**
         * Draws a bounding box of a blob.
         */
        void drawThisBlob(cv::Vec4i ballPoints, cv::Mat img, cv::Scalar color);


        /**
         * Draws all bounding boxes of blobs.
         */
        void drawTheseBlobs(std::vector<cv::Vec4i> theBall, int ind, cv::Mat contours);

        /**performs conversion from BGR color space to the specified color space and returns the converted image as an array of its 3 channels
         *
         * @param[in] src the image to be converged
         * @param[out] dst the converged image as an array of 3 channels
         * @param[in] colorSpace the colorspace you wish to convert to 
         */
        void cvtToAndSplit(cv::Mat src, cv::Mat dst[3], int colorSpace);


        /**
         * loads images from the file
         */
        void loadSrc(std::string filename, cv::Mat& src);

        /**
         * finds the ball in the image if there is a ball in the image
         *
         * @param[in] src the source image
         * @return a pair consisting of the bounding box of the foundball and the distance to the ball
         */
        std::pair<cv::Vec4i, float> findTheBall(cv::Mat src);

        /**
         * Makes it all come together.
         */
        void finalize();

};
///}}}

