////
//// Visual and Autonomous Exploration Systems Research Laboratory
//// Created by Qasim Mahmood on 3/25/18
//
//// This code takes in an image as resized grayscale, performs contrast enhance using its histogram,
//// blurs the image, uses the new histogram to find a nice thresholding point, thresholds,
//// then performs nearest neighbors to remove noise, then utilizes canny to find edges.
//// Following, it will find contours, keeping only the ones that look like squares.
//// Then we create a mask for the first square (meaning only that contour in the image, everything else is black
//// Using that mask, and the coords of the square contour, we use OpenCV prespective transform on the sent in tempaltes
//// Each of those templates get mapped to that contour, and rotated three times
//// Following that, we do this for any other templates sent it (transform and rotate)
//// We keep the best matching one as the found element, then move onto the next contour we have in the image
////
//// Right now, the input parameters are the image we want to condition, and following the
//// number of patters, final area of the image, then lastly any templates you want to compare
////
//// Example->    ./main six_targets.jpg 2 100 pat1.jpg pat2.jpg pat3.jpg pat4.jpg pat5.jpg pat6.jpg
////
//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include <stdio.h>
//#include <stdlib.h>
//#include <vector>
//#include <chrono>
//
//using namespace std;
//using namespace cv;
//
//static double angle( Point pt1, Point pt2, Point pt0 )
//{
//    double dx1 = pt1.x - pt0.x;
//    double dy1 = pt1.y - pt0.y;
//    double dx2 = pt2.x - pt0.x;
//    double dy2 = pt2.y - pt0.y;
//    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
//}
//
//
//void nearestNeighbor(Mat image, int max){
//    int row=1, col=1, NN=0;
//    for(row=1; row<image.rows-1; row++){
//        for(col=1; col<image.cols-1; col++){
//            NN = 0;
//            NN += image.at<uchar>(row-1,col-1);
//            NN += image.at<uchar>(row-1,col);
//            NN += image.at<uchar>(row-1,col+1);
//            NN += image.at<uchar>(row,col-1);
//            NN += image.at<uchar>(row,col+1);
//            NN += image.at<uchar>(row+1,col-1);
//            NN += image.at<uchar>(row+1,col);
//            NN += image.at<uchar>(row+1,col+1);
//            if( NN<(4*max) ){image.at<uchar>(row,col) = 0;}
//        }
//    }
//    return;
//}
//
//// Here we compare the two ROIs (template image and target cluster) We increment for a match, decrement for a mismatch
//// We are using XOR, so if 01 and 10 are mismatch, 00 and 11 are matching
//// We also filter out the image outside the template warping since the bounding box is a straight rectangle but the target might be off axis
//// For this, we filter out any grey values, the total pixels are only ones which are within the tempalte/target cluster, not the surrounding areas
//float compare_images(Mat image_roi_smoothed, Mat image_roi_template){
//    float matched = 0.0;
//    float total = 0.0;
//    int row=0, col=0;
//
//    // For each pixel, if it is not grey, then compare (this filters out the pixel comparisons outside of the template / target cluster
//    for(row=1; row<image_roi_smoothed.rows-1; row++){
//        for(col=1; col<image_roi_smoothed.cols-1; col++){
//            if(image_roi_template.at<uchar>(row,col) != 128){
//                total++;
//                if(image_roi_template.at<uchar>(row,col) == image_roi_smoothed.at<uchar>(row,col)){matched++;}
//                else{matched--;}
//            }
//        }
//    }
//    // Return the total matched over the total pixels
//    return (matched/total);
//}
//
//
//int main(int argc, const char * argv[]) {
//
//    // Defining the number of patterns, the final area we want to maximize to (currently not used), and the resolution we wish to operate on
//    int nPatterns, final_area;
//    // Using first argument as picture name
//    string target_name = argv[1];
//    // How many patterns to find in the target image
//    sscanf(argv[2], "%d", &nPatterns);
//    // What's the final resolution you want to expand the piture to
//    sscanf(argv[3], "%d", &final_area);
//
//    // Creating an image matrix for the image to be opened
//    Mat  input_target_image, resized_input_target_image, dst;
//
//    // Iteration through frames of video
//    string folder = "moving/";
//    string suffix = ".jpg";
//    int counter = 0;
//
//    ////////////////////////////////////////////////////////////////////////////////////
//    // Set Up Kalman stuff here
//
//    // >>>> Kalman Filter
//    int stateSize = 6;
//    int measSize = 4;
//    int contrSize = 0;
//
//    unsigned int type = CV_32F;
//    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
//
//    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//
//    // Transition State Matrix A
//    // Note: set dT at EACH processing step!
//    // [ 1 0 dT 0  0 0 ]
//    // [ 0 1 0  dT 0 0 ]
//    // [ 0 0 1  0  0 0 ]
//    // [ 0 0 0  1  0 0 ]
//    // [ 0 0 0  0  1 0 ]
//    // [ 0 0 0  0  0 1 ]
//    cv::setIdentity(kf.transitionMatrix);
//
//    // Measure Matrix H
//    // [ 1 0 0 0 0 0 ]
//    // [ 0 1 0 0 0 0 ]
//    // [ 0 0 0 0 1 0 ]
//    // [ 0 0 0 0 0 1 ]
//    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//    kf.measurementMatrix.at<float>(0) = 1.0f;
//    kf.measurementMatrix.at<float>(7) = 1.0f;
//    kf.measurementMatrix.at<float>(16) = 1.0f;
//    kf.measurementMatrix.at<float>(23) = 1.0f;
//
//    // Process Noise Covariance Matrix Q
//    // [ Ex   0   0     0     0    0  ]
//    // [ 0    Ey  0     0     0    0  ]
//    // [ 0    0   Ev_x  0     0    0  ]
//    // [ 0    0   0     Ev_y  0    0  ]
//    // [ 0    0   0     0     Ew   0  ]
//    // [ 0    0   0     0     0    Eh ]
//    kf.processNoiseCov.at<float>(0) = 1e-2;
//    kf.processNoiseCov.at<float>(7) = 1e-2;
//    kf.processNoiseCov.at<float>(14) = 5.0f;
//    kf.processNoiseCov.at<float>(21) = 5.0f;
//    kf.processNoiseCov.at<float>(28) = 1e-2;
//    kf.processNoiseCov.at<float>(35) = 1e-2;
//
//    // Measures Noise Covariance Matrix R
//    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
//    // <<<< Kalman Filter
//
//    char ch = 0;
//
//    double ticks = 0;
//    bool found = false;
//
//    int notFoundCount = 0;
//
//    //////////////////////////////////////////////////////////////////////
//    // Kalman ends here
//
//
//    // Loop over all frames in folder
//    while(1) {
//
//        // Predict the new location from previous location
//        double precTick = ticks;
//        ticks = (double) cv::getTickCount();
//        // Calculating the difference in time
//        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
//
//        if (found)
//        {
//            // >>>> Matrix A
//            kf.transitionMatrix.at<float>(2) = dT;
//            kf.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A
//
//            cout << "dT:" << endl << dT << endl;
//
//            state = kf.predict();
//            cout << "State post:" << endl << state << endl;
//
//            cv::Rect predRect;
//            predRect.width = state.at<float>(4);
//            predRect.height = state.at<float>(5);
//            predRect.x = state.at<float>(0) - predRect.width / 2;
//            predRect.y = state.at<float>(1) - predRect.height / 2;
//
//            // Draw prediction
//            cv::rectangle(input_target_image, predRect, CV_RGB(255,0,0), 2);
//            imshow("prediction" , input_target_image);
//            waitKey(0);
//        }
//
//
//        // Setting up name of frames
//        stringstream ss;
//        ss <<  setw(4) <<  setfill('0') << counter; // 0000, 0001, 0002, etc...
//        string number = ss.str();
//
//        // Name of frame to open
//        string name = folder + number + suffix;
//
//        // Reading the image in grayscale
//        input_target_image = imread(name.c_str(), IMREAD_GRAYSCALE);
//
//        // If the image is empty
//        if(input_target_image.empty()){
//            cout <<  "Could not open or find the image" << endl ;
//            return -1;}
//
//        // Resizing the image down 3 times, placing into resized Mat
//        Size size(input_target_image.size().width, input_target_image.size().height);
//        resize(input_target_image, resized_input_target_image, size, INTER_LINEAR);
//
//        Mat contrast_enhanced_target = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//        Mat smoothed_target = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//        Mat thresh_smoothed_enhanced_target = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//
//        // Contrast image by equalizing histogram
//        equalizeHist(resized_input_target_image, contrast_enhanced_target);
//
//        // Apply blur to image
//        bilateralFilter(contrast_enhanced_target, smoothed_target, 12, 50.0, 50.0);
//
//        // Histogram initialization
//        int histSize = 256;
//        float range[] = {0, 256};
//        const float* histRange = { range };
//        bool uniform = true; bool accumulate = false;
//        Mat gray_hist;
//
//        // Calculate the histogram after contrasting
//        calcHist( &smoothed_target, 1, 0, Mat(), gray_hist, 1, &histSize, &histRange, uniform, accumulate );
//
//        // If you want to see the histogram, uncomment this
//        int hist_w = 512; int hist_h = 400;
//        int bin_w = cvRound( (double) hist_w/histSize );
//        Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 255,255,255) );
//
//
//        // Finding peak center of histogram
//        int peakPixel = 0.0, value = 0.0;
//        int maxPixelIndex = 0;
//        for(int i=0; i<256; i++){
//            value = gray_hist.at<float>(i);
//            if(peakPixel<value){
//                peakPixel = value;
//                maxPixelIndex = i;
//            }
//        }
//
//        // Setting threshold value
//        double min, max;
//        minMaxLoc(smoothed_target, &min, &max);
//        maxPixelIndex = (float)(255 - maxPixelIndex)/2.7;
//
//        // Apply a threshold to the image
//        threshold(smoothed_target, smoothed_target, max-maxPixelIndex, 256, CV_THRESH_BINARY);
//
//        // Perform nearest neighbor
//        nearestNeighbor(smoothed_target, max);
//
//        RNG rng(123);
//        vector<vector<Point> > contours;
//        vector<Vec4i> hierarchy;
//
//        // Find contours in the image
//        findContours( smoothed_target, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );
//
//        vector<vector<Point> > squares;
//        vector<Point> approx;
//
//        // Test the contours with our criteria to find squares of ceratin size
//        for( size_t i = 0; i < contours.size(); i++ )
//        {
//            // approximate contour with accuracy proportional to the contour perimeter
//            approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.035, true);
//
//            // The square contours should have 4 vertices after approximation relatively large area (to filter out noisy contours) and be convex. Note: absolute value of an area is used because area may be positive or negative - in accordance with the contour orientation
//            if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)) )
//            {
//                double maxCosine = 0;
//                for( int j = 2; j < 5; j++ ){
//                    // find the maximum cosine of the angle between joint edges
//                    double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
//                    maxCosine = MAX(maxCosine, cosine);
//                }
//
//                // if cosines of all angles are small (all angles are ~90 degree) then write quandrange vertices to resultant sequence
//                if( maxCosine < 0.3 ) {
//                    squares.push_back(approx);
//                }
//            }
//        }
//
//        // Drawing only the ONE contour, the we are working with
//        for(size_t i = 0; i < 1; i++ )
//        {
//            // Check if the lines isn't on the edge of the picture
//            const Point* p = &squares[i][0];
//            int n = (int)squares[i].size();
//            //dont detect the border
//            if (p-> x > 3 && p->y > 3)
//                // Connect the four verticies with a line
//                polylines(input_target_image, &p, &n, 1, true, Scalar(0,0,255), 2, LINE_AA);
//        }
//
//
//        vector<vector<cv::Point> > balls;
//        vector<cv::Rect> ballsBox;
//
//        // Create the bounding box for that one contour
//        cv::Rect bBox;
//        bBox = cv::boundingRect(squares[0]);
//
//        balls.push_back(squares[0]);
//        ballsBox.push_back(bBox);
//
//        //  Drawing the bounding box on top of the contour we found, this is our measurement bounding box
//        cv::rectangle(input_target_image, ballsBox[0], CV_RGB(0,0,255), 2);
//
//        // Pass that first contour to the updating of the Kalman Filter
//        /////////////////////////////////////////////
//        // >>>>> Kalman Update
//        if (squares.size() == 0)
//        {
//            notFoundCount++;
//            cout << "notFoundCount:" << notFoundCount << endl;
//            if( notFoundCount >= 100 )
//            {
//                found = false;
//            }
//            /*else
//             kf.statePost = state;*/
//        }
//        else
//        {
//            notFoundCount = 0;
//
//            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
//            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
//            meas.at<float>(2) = (float)ballsBox[0].width;
//            meas.at<float>(3) = (float)ballsBox[0].height;
//
//            if (!found) // First detection!
//            {
//                // >>>> Initialization
//                kf.errorCovPre.at<float>(0) = 1; // px
//                kf.errorCovPre.at<float>(7) = 1; // px
//                kf.errorCovPre.at<float>(14) = 1;
//                kf.errorCovPre.at<float>(21) = 1;
//                kf.errorCovPre.at<float>(28) = 1; // px
//                kf.errorCovPre.at<float>(35) = 1; // px
//
//                state.at<float>(0) = meas.at<float>(0);
//                state.at<float>(1) = meas.at<float>(1);
//                state.at<float>(2) = 0;
//                state.at<float>(3) = 0;
//                state.at<float>(4) = meas.at<float>(2);
//                state.at<float>(5) = meas.at<float>(3);
//                // <<<< Initialization
//
//                kf.statePost = state;
//
//                found = true;
//            }
//            else
//                kf.correct(meas); // Kalman Correction
//
//            cout << "Measure matrix:" << endl << meas << endl;
//        }
//        // <<<<< Kalman Update
//        /////////////////////////////////////////////
//
//        counter++;
//    }
//
//    return 0;
//}
//
