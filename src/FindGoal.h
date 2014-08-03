#pragma once
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>


class FindGoal
{
    private: 

        const std::string filename = "/home/redsphinx/Projects/BachelorThesis/TestImages9/6-goal-headpitch-02";
        cv::Mat src;
        cv::Mat srcCopy;
        const int GOALLENGTH = 1500; // in mm
        const int GOALHEIGHT = 800; // in mm
        const float GOALRATIO = GOALLENGTH/GOALHEIGHT;
        const int GOALRATIOERROR = 2;
        const float PI = 3.14159265358979323846;
        cv::Vec4i goalBoundingBox;
        int distanceToGoal; //in mm

    public:


        //loads the files
        void loadSrc();

        /**
         * from the list of lines, make all permutations of 3 with them
         *
         * @param[in] lines std::vector of cv::cv::Vec4i's containing coordinates of straight lines found in the image
         * @return all permutations of 3 that can be made with the 3 lines. returns them as a std::vector of std::vector containing cv::cv::Vec4i's
         */
        std::vector< std::vector<cv::Vec4i> > getAllPermutations(std::vector<cv::Vec4i> lines);


        /**
         * if 3 lines are in the shape of a goal, return true
         * 
         * @param[in] i first line
         * @param[in] j second line
         * @param[in] k third line
         */
        bool isInShapeOfGoal(cv::Vec4i i, cv::Vec4i j, cv::Vec4i k);


        /**
         * finds the valid goal out of the permutations of goal shaped lines
         *
         * @param[in] valComb the vector of the valid combos
         * return the coordinates of the bounding box of the found goal
         */
        cv::Vec4i findTheGoal(std::vector< std::vector<cv::Vec4i> > valComb);


        /**
         * does the preprocessing
         *
         * @param[in] src the original image
         * return the processed image
         */
        cv::Mat preProcess(cv::Mat src);


        /**
         * does the line detection
         * @param[in] blurred source image that has been convoluted with Gaussian kernel
         * @return a vector containing the coordinates of the found lines
         */
        std::vector<cv::Vec4i> edgeDetection(cv::Mat blurred, cv::Mat srcCopy);


        /**
         * takes the found lines and checks if 3 of them form a valid goal shape
         * @param[in] houghLines the detected lines
         * @return the coordinates of the found goal
         */
        cv::Vec4i shapeValidation(std::vector<cv::Vec4i> houghLines);


        //draw the found goal
        void graphics(cv::Vec4i goalCors, cv::Mat srcCopy);

        //wraps it all together
        void finalize();
};

