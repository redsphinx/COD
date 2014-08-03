#include "FindGoal.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iostream>


//loads the files
void FindGoal::loadSrc()
{
    src = cv::imread(filename, 1);
    srcCopy = src;
}

/**
 * from the list of lines, make all permutations of 3 with them
 *
 * @param[in] lines std::vector of cv::cv::Vec4i's containing coordinates of straight lines found in the image
 * @return all permutations of 3 that can be made with the 3 lines. returns them as a std::vector of std::vector containing cv::cv::Vec4i's
 */
std::vector< std::vector<cv::Vec4i> > FindGoal::getAllPermutations(std::vector<cv::Vec4i> lines)
{
    int cntAllCombos = 0;
    int linesSize = lines.size();
    std::vector< std::vector<cv::Vec4i> > allCombos;
    for(int i=0; i<linesSize; i++)
    { 

        for(int j=0; j<linesSize; j++)
        { 

            for(int k=0; k<linesSize; k++)
            { 

                if(lines[i] != lines[j] && lines[i] != lines[k] && lines[j] != lines[k])
                {
                    std::vector<cv::Vec4i> rrow;
                    allCombos.push_back(rrow);
                    allCombos[cntAllCombos].push_back(lines[i]);     
                    allCombos[cntAllCombos].push_back(lines[j]); 
                    allCombos[cntAllCombos].push_back(lines[k]);
                    cntAllCombos++;
                }
            }
        }
    }

    return allCombos;
}

/**
 * if 3 lines are in the shape of a goal, return true
 * 
 * @param[in] i first line
 * @param[in] j second line
 * @param[in] k third line
 */
bool FindGoal::isInShapeOfGoal(cv::Vec4i i, cv::Vec4i j, cv::Vec4i k)
{
    //check horizontal and vertical positions of each line
    bool isGoal = false;
    int e = 30; //error parameter for how skewed the lines can be
    //get big and small X and Y's
    int bigXi, smallXi, bigYi, smallYi, bigXj, smallXj, bigYj, smallYj, bigXk, smallXk, bigYk, smallYk;
    
    // the left pole
    bigXi = std::max(i[0], i[2]);
    smallXi = std::min(i[0], i[2]);
    bigYi = std::max(i[1], i[3]);
    smallYi = std::min(i[1], i[3]);
    
    // the top bar
    bigXj = std::max(j[0], j[2]);
    smallXj = std::min(j[0], j[2]);
    bigYj = std::max(j[1], j[3]);
    smallYj = std::min(j[1], j[3]);
    
    // the right pole
    bigXk = std::max(k[0], k[2]);
    smallXk = std::min(k[0], k[2]);
    bigYk = std::max(k[1], k[3]);
    smallYk = std::min(k[1], k[3]);
    
    //if all conditions hold then the 3 given lines form a valid goal shape
    if(abs(i[0]-i[2])<=e && abs(k[0]-k[2])<=e && abs(j[1]-j[3])<=e && smallYj<=smallYi && smallXj>=(bigXi-e) && smallYj<=smallYk && bigXj<=(smallXk+e) )
    { 
        isGoal = true;
    }

    return isGoal;
}

/**
 * finds the valid goal out of the permutations of goal shaped lines
 *
 * @param[in] valComb the vector of the valid combos
 * return the coordinates of the bounding box of the found goal
 */
cv::Vec4i FindGoal::findTheGoal(std::vector< std::vector<cv::Vec4i> > valComb)
{
    int valCombSize = valComb.size();
    float margin = 0.5;

    if(valCombSize == 0)
    {
        cv::Vec4i tmp = cv::Vec4i(0,0,0,0);
        return tmp;
    }

    else if(valCombSize > 1)
    {
        std::vector< std::vector<cv::Vec4i> > filtValComb; //4 cols, 3 for lines and 1 for the minX minY maxX maxY 
        int k = 0; //keeps track of inserted std::vectors in the filtValComb

        //for each valid combo, check if the ratio GOALLENGTH/GOALHEIGHT holds with some margin of error
        //this makes sure that the shape of the found "goal" has approximately the correct size
        for(int i=0; i<valCombSize; i++)
        {
            cv::Vec4i line1 = valComb[i][0];
            cv::Vec4i line2 = valComb[i][1];
            cv::Vec4i line3 = valComb[i][2];

            int minX = std::min(std::min(line3[0], line3[2]), std::min(std::min(line2[0], line2[2]), std::min(line1[0], line1[2])));
            int maxX = std::max(std::max(line3[0], line3[2]), std::max(std::max(line2[0], line2[2]), std::max(line1[0], line1[2])));
            int minY = std::min(std::min(line3[1], line3[3]), std::min(std::min(line2[1], line2[3]), std::min(line1[1], line1[3])));
            int maxY = std::max(std::max(line3[1], line3[3]), std::max(std::max(line2[1], line2[3]), std::max(line1[1], line1[3])));
            cv::Vec4i goalCors = cv::Vec4i(minX, minY, maxX, maxY);
            float hei = maxY-minY;
            float len = maxX-minX;
            if((len/hei) < GOALRATIOERROR && (len/hei) >= (GOALRATIO - margin))
            {
                std::vector<cv::Vec4i> roow; //create row and push before inserting elements
                filtValComb.push_back(roow);
                filtValComb[k].push_back(line1);
                filtValComb[k].push_back(line2);
                filtValComb[k].push_back(line3);
                filtValComb[k].push_back(goalCors);
                k++;
            }
        }

        if(filtValComb.size() == 0)
        {
            cv::Vec4i tmp = cv::Vec4i(0, 0, 0, 0);
            goalBoundingBox = tmp;
            return tmp;
        }

        else if(filtValComb.size() == 1)
        {
            goalBoundingBox = filtValComb[0][3];
            return filtValComb[0][3];
        }

        else
        {
            goalBoundingBox = filtValComb[0][3];
            return filtValComb[0][3]; //for now return first one until think of what to do when multiple valid ratio-good goals detected
        }
    }
}

/**
 * does the preprocessing
 *
 * @param[in] src the original image
 * return the processed image
 */
cv::Mat FindGoal::preProcess(cv::Mat src)
{
    cv::Mat blurred; //blurred image
    double sigmaX = 0; //Gaussian kernel standard deviation in X direction
    double sigmaY = 0; //Gaussian kernel standard deviation in Y direction
    int borderType = cv::BORDER_DEFAULT; //pixel extrapolation method
    int kernelSize = 1; //Gaussian kernel size. Select a small kernel size for the ball because it is a small object. too much blur will cause it to not be detected when it is far away.

    cv::GaussianBlur(src, blurred, cv::Size(kernelSize, kernelSize), sigmaX, sigmaY, borderType);
    //blur(img, imgblur, Size(3,3));
    return blurred;
}

/**
 * does the line detection
 * @param[in] blurred source image that has been convoluted with Gaussian kernel
 * @return a vector containing the coordinates of the found lines
 */
std::vector<cv::Vec4i> FindGoal::edgeDetection(cv::Mat blurred, cv::Mat srcCopy)
{
    cv::Mat cannied; 
    double threshold1 = 100; //first hysteresis
    double threshold2 = 80; //second hysteresis
    int sobel = 3; //use the sobel operator
    bool l2gradient = true; //flag for the L2 norm, if set to true coul dbe more accurate

    cv::Canny(blurred, cannied, threshold1, threshold2, sobel, l2gradient);
    //cvtColor(dst, cdst, CV_GRAY2BGR);

    double rho = 1; //distance resolution of the accumulator in pixels
    double theta = PI/180; //angle resolution of the accumulator in radians
    int threshold3 = 50; //accumulator threshold parameter. only those lines are returned that are >threshold
    double minLineLength = 70; //minimum line length. line segments shorter than that are rejected
    double maxLineGap = 10; //maximum allowed gap between points on the same line to link them
    std::vector<cv::Vec4i> houghLines;

    HoughLinesP(cannied, houghLines, rho, theta, threshold3, minLineLength, maxLineGap);


    //draw the lines
    for( size_t i = 0; i < houghLines.size(); i++ )
    {
        cv::Vec4i l = houghLines[i];
        cv::line( srcCopy, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
    }

    return houghLines;
}

/**
 * takes the found lines and checks if 3 of them form a valid goal shape
 * @param[in] houghLines the detected lines
 * @return the coordinates of the found goal
 */
cv::Vec4i FindGoal::shapeValidation(std::vector<cv::Vec4i> houghLines)
{
    int n = houghLines.size();
    int amount = n*(n-1)*(n-2); //all possible permutations without repetition
    std::vector< std::vector<cv::Vec4i> > validCombos; //array to store the valid line combos that might be goals
    int cntrValCmb = 0; //keeps track of where the valid combos have to be stored
    std::vector< std::vector<cv::Vec4i> > allCombos; //vector of all the permutations

    allCombos = getAllPermutations(houghLines);

    for(int i=0; i<allCombos.size(); i++)
    {

        if(isInShapeOfGoal(allCombos[i][0], allCombos[i][1], allCombos[i][2]))
        {
            std::vector<cv::Vec4i> tmpLine;
            validCombos.push_back(tmpLine);
            validCombos[cntrValCmb].push_back(allCombos[i][0]);
            validCombos[cntrValCmb].push_back(allCombos[i][1]);
            validCombos[cntrValCmb].push_back(allCombos[i][2]);
            cntrValCmb++;
        }
    }

    cv::Vec4i goalCors = findTheGoal(validCombos);
    return goalCors;
}

//draw the found goal
void FindGoal::graphics(cv::Vec4i goalCors, cv::Mat srcCopy)
{
    cv::line(srcCopy, cv::Point(goalCors[0], goalCors[1]), cv::Point(goalCors[2], goalCors[1]), cv::Scalar(0, 255, 0), 2, CV_AA); //scalar is in BGR order
    cv::line(srcCopy, cv::Point(goalCors[2], goalCors[1]), cv::Point(goalCors[2], goalCors[3]), cv::Scalar(0, 255, 0), 2, CV_AA);
    cv::line(srcCopy, cv::Point(goalCors[2], goalCors[3]), cv::Point(goalCors[0], goalCors[3]), cv::Scalar(0, 255, 0), 2, CV_AA);
    cv::line(srcCopy, cv::Point(goalCors[0], goalCors[3]), cv::Point(goalCors[0], goalCors[1]), cv::Scalar(0, 255, 0), 2, CV_AA);

    cv::imshow("detected lines", srcCopy);

}

//wraps it all together
void FindGoal::finalize()
{
    loadSrc();
    cv::Mat blurred = preProcess(src);
    std::vector<cv::Vec4i> houghLines = edgeDetection(blurred, srcCopy);
    cv::Vec4i goalCors = shapeValidation(houghLines);
    graphics(goalCors, srcCopy);
}

int main()
{
    FindGoal goal;
    goal.finalize();
    cv::waitKey(0);
    return 0;
}
