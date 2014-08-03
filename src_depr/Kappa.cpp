#include <cv.h>
#include <highgui.h>
#include <math.h>

using namespace cv;

void doProc(Mat gray, vector<Vec3f> circles, Mat img, string channel)
{
    GaussianBlur( gray, gray, Size(1, 1), 0, 0 );
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
            2, gray.rows/8, 200,  5 );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // draw the circle center
        circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
    namedWindow( channel, 1 );
    imshow( channel, img );

}

void xyz(string filename)
{
    Mat img, trf;
    Mat splitImg[3];
    img = imread(filename, 1);
    cvtColor(img, trf, CV_BGR2XYZ);
    split(trf, splitImg);
    imshow("XYZ - X " + filename, splitImg[0]);
    imshow("XYZ - Y " + filename, splitImg[1]);
    imshow("XYZ - Z " + filename, splitImg[2]);
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/XYZ - X - .jpg", splitImg[0] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/XYZ - Y - .jpg", splitImg[1] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/XYZ - Z - .jpg",  splitImg[2] );
}

void hsv(string filename)
{
    Mat img2, trf2;
    Mat splitImg2[3];
    img2 = imread(filename, 1);
    cvtColor(img2, trf2, CV_BGR2HSV);
    split(trf2, splitImg2);
    //imshow("HSV - H " + filename, splitImg2[0]);
    imshow("HSV - S " + filename, splitImg2[1]);
    //imshow("HSV - V " + filename, splitImg2[2]);
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/HSV - H - .jpg", splitImg2[0] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/HSV - S - .jpg", splitImg2[1] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/HSV - V - .jpg", splitImg2[2] );
}

void bgr(string filename)
{
    Mat img3, trf3;
    Mat splitImg3[3];
    img3 = imread(filename, 1);
    split(img3, splitImg3);
    imshow("BGR - B " + filename, splitImg3[0]);
    imshow("BGR - G " + filename, splitImg3[1]);
    imshow("BGR - R " + filename, splitImg3[2]);
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/BGR - B - .jpg", splitImg3[0] );
    imwrite( "/home/redsphinx/Projects/BachelorThesis/TestImages8/BGR - G - .jpg", splitImg3[1]);
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/BGR - R - .jpg", splitImg3[2] );
}

void hls(string filename)
{
    Mat img4, trf4;
    Mat splitImg4[3];
    img4 = imread(filename, 1);
    cvtColor(img4, trf4, CV_BGR2HLS);
    split(trf4, splitImg4);
    //imshow("HLS - H " + filename, splitImg4[0]);
    //imshow("HLS - L " + filename, splitImg4[1]);
    imshow("HLS - S " + filename, splitImg4[2]);
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/HLS - H - .jpg", splitImg4[0] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/HLS - L - .jpg", splitImg4[1] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/HLS - S - .jpg", splitImg4[2] );
}

void lab(string filename)
{
    Mat img5, trf5;
    Mat splitImg5[3];
    img5 = imread(filename, 1);
    cvtColor(img5, trf5, CV_BGR2Lab);
    split(trf5, splitImg5);
    imshow("Lab - L " + filename, splitImg5[0]);
    imshow("Lab - a " + filename, splitImg5[1]);
    imshow("Lab - b " + filename, splitImg5[2]);
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/Lab - L - .jpg", splitImg5[0] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages9/Lab - a - .jpg", splitImg5[1] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/Lab - b - .jpg", splitImg5[2] );
}

void luv(string filename)
{
    Mat img6, trf6;
    Mat splitImg6[3];
    img6 = imread(filename, 1);
    cvtColor(img6, trf6, CV_BGR2Luv);
    split(trf6, splitImg6);
    imshow("Luv - L " + filename, splitImg6[0]);
    imshow("Luv - u " + filename, splitImg6[1]);
    imshow("Luv - v " + filename, splitImg6[2]);
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/Luv - L - .jpg", splitImg6[0] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/Luv - u - .jpg", splitImg6[1] );
    imwrite("/home/redsphinx/Projects/BachelorThesis/TestImages8/Luv - v - .jpg", splitImg6[2] );
}

int main()
{
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

    Mat src = imread(filename11, 1);
    imshow("original", src);
    hls(filename);
    hls(filename2);
    hls(filename3);
    hls(filename4);
    hls(filename5);
    hls(filename6);
    hls(filename7);
    hls(filename8);
    hls(filename9);
    hls(filename10);
    hls(filename11);
    hls(filename12);
    hls(filename13);
    hls(filename14);
    hls(filename15);
    hls(filename16);
    hls(filename17);
    hls(filename18);
    hls(filename19);
    hls(filename20);
    hls(filename21);
    hls(filename22);
    hls(filename23);
    hls(filename24);
    hls(filename25);
    hls(filename26);
    hls(filename27);
    hls(filename28);

    


    //---
    //if( argc != 2 && !(img=imread(argv[1], 1)).data)
        //return -1;
    //cvtColor(img, gray, CV_BGR2GRAY);
    //cv::Mat splittedTmp[3];
    //split(img, splittedTmp);
    //string channel = "2";
    //int ch = 2;
    //gray = splittedTmp[ch];
    //imshow("red", gray);
    //doProc(gray, circles, img, channel);
    
    //channel = "1";
    //ch = 1;
    //gray = splittedTmp[ch];
    //imshow("greem", gray);
    //doProc(gray, circles, img, channel);

    //// smooth it, otherwise a lot of false circles may be detected
    std::cout << "HERE" << std::endl;
    waitKey(0);
    return 0;
}
