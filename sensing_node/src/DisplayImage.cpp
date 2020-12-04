#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;


//Mat redFilter(const Mat& cap)
//{ //Mat imgOriginal;
////   // assert(cap.read(imgOriginal) == CV_8UC3);

//   Mat redOnly;
////    inRange(cap, Scalar(0, 0, 0), Scalar(0, 0, 255), redOnly);
//inRange(cap, Scalar(160, 160, 60), Scalar(179, 255, 255), redOnly); //Threshold the image

//    return redOnly;
//}
int main(int argc, char** argv)
{

    VideoCapture cap(0); //capture the video from web cam


         if ( !cap.isOpened() )  // if not success, exit program
         {
             cout << "Cannot open the web cam" << endl;
             return -1;
        }

         {
//             if ( argc != 2 )
//             {
//                 printf("usage: DisplayImage.out <Image_Path>\n");
//                 return -1;
//             }

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
            inRange(imgHSV, Scalar(160, 160, 60), Scalar(179, 255, 255), imgThresholded); //Threshold the image


              //morphological opening (remove small objects from the foreground)
              erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
              dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

              //morphological closing (fill small holes in the foreground)
              dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
              erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

              imshow("Thresholded Image", imgThresholded); //show the thresholded image
              imshow("Original", imgOriginal); //show the original image
// To read the red only

            // Mat input = imread("colored_squares.png");

            //imshow("input", input);

//              Mat redOnly = redFilter(imgOriginal);

//              imshow("redOnly", redOnly);
              if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
                     {
                          cout << "esc key is pressed by user" << endl;
                          break;
                     }

   // waitKey();

    // detect squares after filtering...
}
         }
    return 0;
}

