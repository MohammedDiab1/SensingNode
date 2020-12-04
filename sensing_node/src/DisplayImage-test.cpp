#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;


Mat RedFilter(const Mat& src)
{
  bool isColor = (src.type() == CV_8UC3);

    Mat CocaCan;
    inRange(src, Scalar(0, 0, 0), Scalar(0, 0, 255), CocaCan);

    return CocaCan;
}

int main(int argc, char** argv)
{
    const string sourceReference = argv[1],sourceCompareWith = argv[2];
    VideoCapture captRefrnc(sourceReference);
    // or
//    VideoCapture captUndTst;
//    captUndTst.open(sourceCompareWith);

    VideoCapture cap(0); //capture the video from web cam, its zero bcz this is only one camera(if multi, you have to assign the ID), Moreover, we can assign the string if its a vedio stream


         if ( !cap.isOpened() )  // if not success, exit program
         {
             cout << "Cannot open the web cam" << endl;
             return -1;
        }
//         Mat frameReference, frameUnderTest;
//         captRefrnc >> frameReference;
//         captUndTst.open(frameUnderTest);
//         if( frameReference.empty()  || frameUnderTest.empty())
//         {
//          // exit the program
//         }
//         if (!captUndTst.isOpened())
//         {
//             cout  << "Could not open case test " << sourceCompareWith << endl;
//             return -1;
//         }
         {   Mat src;
             // get one frame from camera to know frame size and type
             cap >> src;
             // check if we succeeded
             if (src.empty()) {
                 cerr << "ERROR! blank frame grabbed\n";
                 return -1;
             }
             bool isColor = (src.type() == CV_8UC3);

             //--- INITIALIZE VIDEOWRITER
             VideoWriter writer;
             int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
             double fps = 25.0;                          // framerate of the created video stream
             string filename = "./live.avi";             // name of the output video file
             writer.open(filename, codec, fps, src.size(), isColor);
             // check if we succeeded
             if (!writer.isOpened()) {
                 cerr << "Could not open the output video file for write\n";
                 return -1;
             }
             //--- GRAB AND WRITE LOOP
             cout << "Writing videofile: " << filename << endl
                  << "Press any key to terminate" << endl;
             for (;;)
             {
                 // check if we succeeded
                 if (!cap.read(src)) {
                     cerr << "ERROR! blank frame grabbed\n";
                     break;
                 }
                 // encode the frame into the videofile stream
                 writer.write(src);
                 // show live and wait for a key with timeout long enough to show images
                 imshow("Live", src);
                 if (waitKey(5) >= 0)
                     break;
             }

             while (true)
                 {
                     Mat imgOriginal;

                     bool bSuccess = cap.read(imgOriginal); // read a new frame from video

                      if (!bSuccess) //if not success, break loop
                     {
                          cout << "Cannot read a frame from video stream" << endl;
                          break;
                     }

            Mat imgHSV;
            cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
            Mat imgThresholded;
              //morphological opening (remove small objects from the foreground)
              erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
              dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

              //morphological closing (fill small holes in the foreground)
              dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
              erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
              Mat CocaCan = RedFilter(imgOriginal);
              imshow("Thresholded Image", imgThresholded); //show the thresholded image
              imshow("Original", imgOriginal); //show the original image
              if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
                     {
                          cout << "esc key is pressed by user" << endl;
                          break;
                     }

    // detect squares after filtering...
}
         }
    return 0;
}

