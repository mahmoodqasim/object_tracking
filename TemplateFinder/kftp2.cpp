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
//#include "finding_contour.hpp"
//#include "kalman.hpp"
//#include "template_matching.hpp"
//
//using namespace std;
//using namespace cv;
//
//int main(int argc, const char * argv[]) {
//
//    // Defining classes for finding contour, kalman filter, and pairs for contour/template-match
//    fi_con f;
//    kal_fil kalFil;
//    // Map to keep track of the contours and identification
//    unordered_map<string, vector<Point> > tracking_map;
//    unordered_map<string, Point2f > itr;
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
//    Mat  input_target_image;
//    
//    // Iteration through frames of video / folder, start with frame 0
//    string folder = "moving/";
//    string suffix = ".jpg";
//    int counter = 0;
//
//
//    ////////////////////////////////////////////////////////////////////////////////////
//    // >>>> Kalman Filter set-up
//    int stateSize = 6;
//    int measSize = 4;
//    int contrSize = 0;
//    unsigned int type = CV_32F;
//
//    // Create Kalman filter
//    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
//
//    // Set up variables for the state and measured
//    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//
//    // Set up of matrices A, H, with noise Q and R
//    kalFil.setUpKalman(kf, state, meas, stateSize, measSize, type);
//
//    // Flags for Kalman filter
//    double ticks = 0;
//    bool found = false;
//    int notFoundCount = 0;
//
//    //////////////////////////////////////////////////////////////////////
//    // Kalman ends here
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
//        // Grabbing the next frame
//        stringstream ss;
//        ss <<  setw(4) <<  setfill('0') << counter; // 0000, 0001, 0002, etc...
//        string number = ss.str();
//        string name = folder + number + suffix;
//
//        // Reading the next frame as grayscale
//        input_target_image = imread(name.c_str(), IMREAD_GRAYSCALE);
//
//        // If the image is empty
//        if(input_target_image.empty()){
//            cout <<  "Could not open or find the image" << endl ;
//            return -1;}
//
//        // Image processing the image
//        Mat smoothed_target = f.image_processing(input_target_image);
//
//        // Get all contours that are of square shape
//        vector<vector<Point>> squares = f.find_squares(smoothed_target);
//
//        // After finding squares, we associate each square with the template found
//        temp_mat m;
//        m.find_contours_templates(squares, smoothed_target, argc, argv, tracking_map);
//
//        // Able to grab rab the contour by name now
//        unordered_map<string,vector<Point>>::const_iterator got = tracking_map.find ("pat1.jpg");
//        // Printing the first or second thing in it, YEAH BOI
//        cout << tracking_map.find ("pat2.jpg")->second << endl;
//        cout << got->second << endl;
//
//        int i=0;
//        // Drawing only the ONE contour, the we are working with
//        for(i = 0; i < squares.size(); i++ )
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
//        // Draw contours for all squares (measurements)
//        // Draw all predictions
//
//        // Create the bounding box for that one contour
//        vector<cv::Rect> ballsbounding_Boxes;
//        cv::Rect bBox;
//        bBox = cv::boundingRect(squares[0]);
//        ballsbounding_Boxes.push_back(bBox);
//
//        //  Drawing the bounding box on top of the contour we found, this is our measurement bounding box
//        cv::rectangle(input_target_image, ballsbounding_Boxes[0], CV_RGB(0,0,255), 2);
//
//        // Pass that first contour to the updating of the Kalman Filter update
//        /////////////////////////////////////////////
//        // >>>>> Kalman Update
//        if (squares.size() == 0){
//            notFoundCount++;
//            cout << "notFoundCount:" << notFoundCount << endl;
//            if( notFoundCount >= 100 ){
//                found = false;
//            }
//        }
//        else{
//            notFoundCount = 0;
//
//            meas.at<float>(0) = ballsbounding_Boxes[0].x + ballsbounding_Boxes[0].width / 2;
//            meas.at<float>(1) = ballsbounding_Boxes[0].y + ballsbounding_Boxes[0].height / 2;
//            meas.at<float>(2) = (float)ballsbounding_Boxes[0].width;
//            meas.at<float>(3) = (float)ballsbounding_Boxes[0].height;
//
//            // First detection! Set up Kalman and Object identification
//            if (!found){
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
//                kf.statePost = state;
//
//                /* // Now we can initialize the first contours
//                // Here we insert the contour, with the identification
//                tracking_map.insert(make_pair("tati", squares[0]));
//                tracking_map.insert(make_pair("pashaap", squares[0]));
//                tracking_map.insert(make_pair("tuni", squares[0]));
//                tracking_map.insert(make_pair("pity-tuni", squares[0]));
//
//                // Finding the object, YEAH BOI
//                unordered_map<string,vector<Point>>::const_iterator got = tracking_map.find ("pity-tuni");
//                // Printing the first or second thing in it, YEAH BOI
//                cout << got->second << endl;
//            */
//
//                found = true;
//            }
//            else {
//                kf.correct(meas); // Kalman Correction
//            }
//            cout << "Measure matrix:" << endl << meas << endl;
//        }
//        // <<<<< Kalman Update
//        /////////////////////////////////////////////
//
//        // Next frame
//        counter++;
//    }
//
//    return 0;
//}
//
