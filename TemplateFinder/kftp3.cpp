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
//    Mat  input_target_image, drawing_image;
//    
//    // Iteration through frames of video / folder, start with frame 0
//    string folder = "frames4/";
//    string suffix = ".jpg";
//    int counter = 0;
//    
//    ////////////////////////////////////////////////////////////////////////////////////
//    // >>>> Kalman Filter set-up
//    int stateSize = 6;
//    int measSize = 4;
//    int contrSize = 0;
//    unsigned int type = CV_32F;
//    
//    // Later automate this to create the number of KF trackers itself, right now it needs to be define manually.
//    
//    // Create Kalman filter 1 - 5 (these will map to the pat1 - pat5 sent in! So pat 1 is tracked via meas1)
//    cv::KalmanFilter kf1(stateSize, measSize, contrSize, type);
//    cv::KalmanFilter kf2(stateSize, measSize, contrSize, type);
//    cv::KalmanFilter kf3(stateSize, measSize, contrSize, type);
//    cv::KalmanFilter kf4(stateSize, measSize, contrSize, type);
//    cv::KalmanFilter kf5(stateSize, measSize, contrSize, type);
//    cv::KalmanFilter kf6(stateSize, measSize, contrSize, type);
//    
//    // Set up variables for the state and measured for 1
//    cv::Mat state1(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas1(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    
//    // Set up variables for the state and measured for 2
//    cv::Mat state2(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas2(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    
//    // Set up variables for the state and measured for 3
//    cv::Mat state3(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas3(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    
//    // Set up variables for the state and measured for 4
//    cv::Mat state4(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas4(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    
//    // Set up variables for the state and measured for 5
//    cv::Mat state5(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas5(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    
//    // Set up variables for the state and measured for 5
//    cv::Mat state6(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas6(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    
//    // Set up of matrices A, H, with noise Q and R for 1
//    kalFil.setUpKalman(kf1, state1, meas1, stateSize, measSize, type);
//    
//    // Set up of matrices A, H, with noise Q and R for 2
//    kalFil.setUpKalman(kf2, state2, meas2, stateSize, measSize, type);
//
//    // Set up of matrices A, H, with noise Q and R for 3
//    kalFil.setUpKalman(kf3, state3, meas3, stateSize, measSize, type);
//
//    // Set up of matrices A, H, with noise Q and R for 4
//    kalFil.setUpKalman(kf4, state3, meas4, stateSize, measSize, type);
//    
//    // Set up of matrices A, H, with noise Q and R for 5
//    kalFil.setUpKalman(kf5, state5, meas5, stateSize, measSize, type);
//    
//    // Set up of matrices A, H, with noise Q and R for 5
//    kalFil.setUpKalman(kf6, state6, meas6, stateSize, measSize, type);
//    
//    // Flags for Kalman filter, stay the same?
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
//        // Timing the code
//        auto start = std::chrono::high_resolution_clock::now();
//        
//        // Predict the new location from previous location
//        // Replace this with actual frequency of camera recorindg, i.e. FPS
//        double precTick = ticks;
//        ticks = (double) cv::getTickCount();
//        // Calculating the difference in time
//        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
//        
//        if (found)
//        {
//            // >>>> Matrix A for 1
//            kf1.transitionMatrix.at<float>(2) = dT;
//            kf1.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A for 1
//            
//            // >>>> Matrix A for 2
//            kf2.transitionMatrix.at<float>(2) = dT;
//            kf2.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A for 2
//    
//            // >>>> Matrix A for 3
//            kf3.transitionMatrix.at<float>(2) = dT;
//            kf3.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A for 3
//        
//            // >>>> Matrix A for 4
//            kf4.transitionMatrix.at<float>(2) = dT;
//            kf4.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A for 4
//            
//            // >>>> Matrix A for 5
//            kf5.transitionMatrix.at<float>(2) = dT;
//            kf5.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A for 5
//            
//            // >>>> Matrix A for 6
//            kf6.transitionMatrix.at<float>(2) = dT;
//            kf6.transitionMatrix.at<float>(9) = dT;
//            // <<<< Matrix A for 6
//            
////            cout << "dT:" << endl << dT << endl;
//            
//            state1 = kf1.predict();
////            cout << "State post:" << endl << state1 << endl;
//
//            state2 = kf2.predict();
//            state3 = kf3.predict();
//            state4 = kf4.predict();
//            state5 = kf5.predict();
//            state6 = kf6.predict();
//            
//            cv::Rect predRect1;
//            predRect1.width = state1.at<float>(4);
//            predRect1.height = state1.at<float>(5);
//            predRect1.x = state1.at<float>(0) - predRect1.width / 2;
//            predRect1.y = state1.at<float>(1) - predRect1.height / 2;
//
//            cv::Rect predRect2;
//            predRect2.width = state2.at<float>(4);
//            predRect2.height = state2.at<float>(5);
//            predRect2.x = state2.at<float>(0) - predRect2.width / 2;
//            predRect2.y = state2.at<float>(1) - predRect2.height / 2;
//            
//            cv::Rect predRect3;
//            predRect3.width = state3.at<float>(4);
//            predRect3.height = state3.at<float>(5);
//            predRect3.x = state3.at<float>(0) - predRect3.width / 2;
//            predRect3.y = state3.at<float>(1) - predRect3.height / 2;
//            
//            cv::Rect predRect4;
//            predRect4.width = state4.at<float>(4);
//            predRect4.height = state4.at<float>(5);
//            predRect4.x = state4.at<float>(0) - predRect4.width / 2;
//            predRect4.y = state4.at<float>(1) - predRect4.height / 2;
//            
//            cv::Rect predRect5;
//            predRect5.width = state5.at<float>(4);
//            predRect5.height = state5.at<float>(5);
//            predRect5.x = state5.at<float>(0) - predRect5.width / 2;
//            predRect5.y = state5.at<float>(1) - predRect5.height / 2;
//            
//            cv::Rect predRect6;
//            predRect6.width = state6.at<float>(4);
//            predRect6.height = state6.at<float>(5);
//            predRect6.x = state6.at<float>(0) - predRect6.width / 2;
//            predRect6.y = state6.at<float>(1) - predRect6.height / 2;
//            
//            // Draw prediction
//            cv::rectangle(input_target_image, predRect1, CV_RGB(0,0,200), 2);
//            cv::rectangle(input_target_image, predRect2, CV_RGB(0,0,200), 2);
//            cv::rectangle(input_target_image, predRect3, CV_RGB(0,0,200), 2);
//            cv::rectangle(input_target_image, predRect4, CV_RGB(0,0,200), 2);
//            cv::rectangle(input_target_image, predRect5, CV_RGB(0,0,200), 2);
//            cv::rectangle(input_target_image, predRect6, CV_RGB(0,0,200), 2);
//            imshow("prediction" , input_target_image);
//            imshow("drawing" , drawing_image);
////            imwrite("original.jpg" , drawing_image);
////            imwrite("grey-color-con-bounding.jpg" , drawing_image);
//            waitKey(0);
//        }
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
//        // Used to create images for the paper
//        drawing_image = imread(name.c_str());
//        
////        cvtColor(input_target_image, drawing_image, CV_GRAY2RGB);
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
//        unordered_map<string, Point2f > itr;
//
//        // Iterate over the contours and draw in the bounding rectangle on the measured contour
//        for (std::pair<string,vector<Point>> element : tracking_map){
//            // Gray scale image
//            cv::rectangle(input_target_image, cv::boundingRect(element.second), cv::Scalar(200,200,200), 2);
//            cv::putText(input_target_image, element.first, element.second[1], cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255,255,255));
//            
//            // Color image
//            cv::rectangle(drawing_image, cv::boundingRect(element.second), cv::Scalar(0,0,255), 2);
//            cv::putText(drawing_image, element.first, element.second[1], cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(0,255,255));
//        }
//        
//        // Able to grab the contour by name now, more specifically, we can find it by the name
////        unordered_map<string,vector<Point>>::const_iterator got = tracking_map.find ("pat1.jpg");
//        // Printing the first or second thing in it, YEAH BOI
////        cout << tracking_map.find ("pat2.jpg")->second << endl;
////        cout << got->second << endl;
//        
//// Drawing the contours!s
//        int i=0;
//        // Drawing all the contours found in measurement
//        for(i = 0; i < squares.size(); i++ ){
//            // Check if the lines isn't on the edge of the picture
//            const Point* p = &squares[i][0];
//            int n = (int)squares[i].size();
//            //dont detect the border
//            if (p-> x > 3 && p->y > 3)
//                // Connect the four verticies with a line
//                polylines(input_target_image, &p, &n, 1, true, cv::Scalar(160,160,160), 2, LINE_AA);
//                // Drawing on color image
//                polylines(drawing_image, &p, &n, 1, true, cv::Scalar(255,0, 0), 2, LINE_AA);
//        }
//
//        // Pass the contours to the updating of the Kalman Filter
//        /////////////////////////////////////////////
//        // >>>>> Kalman Update
//        if (squares.size() == 0){
//            notFoundCount++;
//            cout << "notFoundCount:" << notFoundCount << endl;
//            if( notFoundCount >= 100 ){ found = false; }
//        }
//        else{
//            notFoundCount = 0;
//            
//            // Find pattern 1 in our tracking map
//            unordered_map<string,vector<Point>>::const_iterator pattern = tracking_map.find("temp1");
//            // If it exists, then assign the bounding box to the measurement
//            if(pattern != tracking_map.end()){
//                cv::Rect box = cv::boundingRect(pattern->second);
//                meas1.at<float>(0) = box.x + box.width / 2;
//                meas1.at<float>(1) = box.y + box.height / 2;
//                meas1.at<float>(2) = (float)box.width;
//                meas1.at<float>(3) = (float)box.height;
//            }
//            
//            // Find pattern 2 in our tracking map
//            pattern = tracking_map.find("temp2");
//            // If it exists, then assign the bounding box to the measurement
//            if(pattern != tracking_map.end()){
//                cv::Rect box = cv::boundingRect(pattern->second);
//                meas2.at<float>(0) = box.x + box.width / 2;
//                meas2.at<float>(1) = box.y + box.height / 2;
//                meas2.at<float>(2) = (float)box.width;
//                meas2.at<float>(3) = (float)box.height; }
//            
//            // Find pattern 3 in our tracking map
//            pattern = tracking_map.find("temp3");
//            // If it exists, then assign the bounding box to the measurement
//            if(pattern != tracking_map.end()){
//                cv::Rect box = cv::boundingRect(pattern->second);
//                meas3.at<float>(0) = box.x + box.width / 2;
//                meas3.at<float>(1) = box.y + box.height / 2;
//                meas3.at<float>(2) = (float)box.width;
//                meas3.at<float>(3) = (float)box.height; }
//            
//            // Find pattern 4 in our tracking map
//            pattern = tracking_map.find("temp4");
//            // If it exists, then assign the bounding box to the measurement
//            if(pattern != tracking_map.end()){
//                cv::Rect box = cv::boundingRect(pattern->second);
//                meas4.at<float>(0) = box.x + box.width / 2;
//                meas4.at<float>(1) = box.y + box.height / 2;
//                meas4.at<float>(2) = (float)box.width;
//                meas4.at<float>(3) = (float)box.height; }
//            
//            // Find pattern 5 in our tracking map
//            pattern = tracking_map.find("temp5");
//            // If it exists, then assign the bounding box to the measurement
//            if(pattern != tracking_map.end()){
//                cv::Rect box = cv::boundingRect(pattern->second);
//                meas5.at<float>(0) = box.x + box.width / 2;
//                meas5.at<float>(1) = box.y + box.height / 2;
//                meas5.at<float>(2) = (float)box.width;
//                meas5.at<float>(3) = (float)box.height; }
//            
//            // Find pattern 6 in our tracking map
//            pattern = tracking_map.find("temp6");
//            // If it exists, then assign the bounding box to the measurement
//            if(pattern != tracking_map.end()){
//                cv::Rect box = cv::boundingRect(pattern->second);
//                meas6.at<float>(0) = box.x + box.width / 2;
//                meas6.at<float>(1) = box.y + box.height / 2;
//                meas6.at<float>(2) = (float)box.width;
//                meas6.at<float>(3) = (float)box.height; }
//            
//            // First detection! Set up Kalman and Object identification
//            if (!found){
//                // >>>> Initialization for 1
//                // Find pattern 1 in our tracking map
//                unordered_map<string,vector<Point>>::const_iterator pattern = tracking_map.find("temp1");
//                // If it exists, then assign the bounding box to the measurement
//                if(pattern != tracking_map.end()){
//                    kf1.errorCovPre.at<float>(0) = 1; // px
//                    kf1.errorCovPre.at<float>(7) = 1; // px
//                    kf1.errorCovPre.at<float>(14) = 1;
//                    kf1.errorCovPre.at<float>(21) = 1;
//                    kf1.errorCovPre.at<float>(28) = 1; // px
//                    kf1.errorCovPre.at<float>(35) = 1; // px
//                    
//                    state1.at<float>(0) = meas1.at<float>(0);
//                    state1.at<float>(1) = meas1.at<float>(1);
//                    state1.at<float>(2) = 0;
//                    state1.at<float>(3) = 0;
//                    state1.at<float>(4) = meas1.at<float>(2);
//                    state1.at<float>(5) = meas1.at<float>(3);
//                    kf1.statePost = state1; }
//                // <<<< Initialization for 1
//
//                // >>>> Initialization for 2
//                // Find pattern 2 in our tracking map
//                pattern = tracking_map.find("temp2");
//                // If it exists, then assign the bounding box to the measurement
//                if(pattern != tracking_map.end()){
//                    kf2.errorCovPre.at<float>(0) = 1; // px
//                    kf2.errorCovPre.at<float>(7) = 1; // px
//                    kf2.errorCovPre.at<float>(14) = 1; // px
//                    kf2.errorCovPre.at<float>(21) = 1; // px
//                    kf2.errorCovPre.at<float>(28) = 1; // px
//                    kf2.errorCovPre.at<float>(35) = 1; // px
//                    
//                    state2.at<float>(0) = meas2.at<float>(0);
//                    state2.at<float>(1) = meas2.at<float>(1);
//                    state2.at<float>(2) = 0;
//                    state2.at<float>(3) = 0;
//                    state2.at<float>(4) = meas2.at<float>(2);
//                    state2.at<float>(5) = meas2.at<float>(3);
//                    kf2.statePost = state2; }
//                // <<<< Initialization for 2
//
//                // >>>> Initialization for 3
//                // Find pattern 3 in our tracking map
//                pattern = tracking_map.find("temp3");
//                // If it exists, then assign the bounding box to the measurement
//                if(pattern != tracking_map.end()){
//                    kf3.errorCovPre.at<float>(0) = 1; // px
//                    kf3.errorCovPre.at<float>(7) = 1; // px
//                    kf3.errorCovPre.at<float>(14) = 1;
//                    kf3.errorCovPre.at<float>(21) = 1;
//                    kf3.errorCovPre.at<float>(28) = 1; // px
//                    kf3.errorCovPre.at<float>(35) = 1; // px
//                    
//                    state3.at<float>(0) = meas3.at<float>(0);
//                    state3.at<float>(1) = meas3.at<float>(1);
//                    state3.at<float>(2) = 0;
//                    state3.at<float>(3) = 0;
//                    state3.at<float>(4) = meas3.at<float>(2);
//                    state3.at<float>(5) = meas3.at<float>(3);
//                    kf3.statePost = state3; }
//                // <<<< Initialization for 3
//                
//                // >>>> Initialization for 4
//                // Find pattern 4 in our tracking map
//                pattern = tracking_map.find("temp4");
//                // If it exists, then assign the bounding box to the measurement
//                if(pattern != tracking_map.end()){
//                    kf4.errorCovPre.at<float>(0) = 1; // px
//                    kf4.errorCovPre.at<float>(7) = 1; // px
//                    kf4.errorCovPre.at<float>(14) = 1;
//                    kf4.errorCovPre.at<float>(21) = 1;
//                    kf4.errorCovPre.at<float>(28) = 1; // px
//                    kf4.errorCovPre.at<float>(35) = 1; // px
//                    
//                    state4.at<float>(0) = meas4.at<float>(0);
//                    state4.at<float>(1) = meas4.at<float>(1);
//                    state4.at<float>(2) = 0;
//                    state4.at<float>(3) = 0;
//                    state4.at<float>(4) = meas4.at<float>(2);
//                    state4.at<float>(5) = meas4.at<float>(3);
//                    kf4.statePost = state4; }
//                // <<<< Initialization for 4
//                
//                // >>>> Initialization for 5
//                // Find pattern 2 in our tracking map
//                pattern = tracking_map.find("temp5");
//                // If it exists, then assign the bounding box to the measurement
//                if(pattern != tracking_map.end()){
//                    kf5.errorCovPre.at<float>(0) = 1; // px
//                    kf5.errorCovPre.at<float>(7) = 1; // px
//                    kf5.errorCovPre.at<float>(14) = 1;
//                    kf5.errorCovPre.at<float>(21) = 1;
//                    kf5.errorCovPre.at<float>(28) = 1; // px
//                    kf5.errorCovPre.at<float>(35) = 1; // px
//                    
//                    state5.at<float>(0) = meas5.at<float>(0);
//                    state5.at<float>(1) = meas5.at<float>(1);
//                    state5.at<float>(2) = 0;
//                    state5.at<float>(3) = 0;
//                    state5.at<float>(4) = meas5.at<float>(2);
//                    state5.at<float>(5) = meas5.at<float>(3);
//                    kf5.statePost = state5; }
//                // <<<< Initialization for 5
//                
//                // >>>> Initialization for 6
//                // Find pattern 6 in our tracking map
//                pattern = tracking_map.find("temp6");
//                // If it exists, then assign the bounding box to the measurement
//                if(pattern != tracking_map.end()){
//                    kf6.errorCovPre.at<float>(0) = 1; // px
//                    kf6.errorCovPre.at<float>(7) = 1; // px
//                    kf6.errorCovPre.at<float>(14) = 1;
//                    kf6.errorCovPre.at<float>(21) = 1;
//                    kf6.errorCovPre.at<float>(28) = 1; // px
//                    kf6.errorCovPre.at<float>(35) = 1; // px
//                    
//                    state6.at<float>(0) = meas6.at<float>(0);
//                    state6.at<float>(1) = meas6.at<float>(1);
//                    state6.at<float>(2) = 0;
//                    state6.at<float>(3) = 0;
//                    state6.at<float>(4) = meas6.at<float>(2);
//                    state6.at<float>(5) = meas6.at<float>(3);
//                    kf6.statePost = state6; }
//                // <<<< Initialization for 6
//                
//                
//                /* // Now we can initialize the first contours
//                 // Here we insert the contour, with the identification
//                 tracking_map.insert(make_pair("tati", squares[0]));
//                 tracking_map.insert(make_pair("pashaap", squares[0]));
//                 tracking_map.insert(make_pair("tuni", squares[0]));
//                 tracking_map.insert(make_pair("pity-tuni", squares[0]));
//                 
//                 // Finding the object, YEAH BOI
//                 unordered_map<string,vector<Point>>::const_iterator got = tracking_map.find ("pity-tuni");
//                 // Printing the first or second thing in it, YEAH BOI
//                 cout << got->second << endl;
//                 */
//                
//                found = true;
//            }
//            else {
//                pattern = tracking_map.find("temp1");
//                    if(pattern != tracking_map.end()){ kf1.correct(meas1); } // Kalman Correction for 1
//                pattern = tracking_map.find("temp2");
//                    if(pattern != tracking_map.end()){ kf2.correct(meas2); } // Kalman Correction for 2
//                pattern = tracking_map.find("temp3");
//                    if(pattern != tracking_map.end()){ kf3.correct(meas3); } // Kalman Correction for 3
//                pattern = tracking_map.find("temp4");
//                    if(pattern != tracking_map.end()){ kf4.correct(meas4); } // Kalman Correction for 4
//                pattern = tracking_map.find("temp5");
//                    if(pattern != tracking_map.end()){ kf5.correct(meas5); } // Kalman Correction for 5
//                pattern = tracking_map.find("temp6");
//                    if(pattern != tracking_map.end()){ kf6.correct(meas6); } // Kalman Correction for 6
//                
//
//                // Drawing the correct state that uses both prediction and measurement
////                cv::Rect true_state1;
////                true_state1.width = kf1.statePost.at<float>(4);
////                true_state1.height = kf1.statePost.at<float>(5);
////                true_state1.x = kf1.statePost.at<float>(0) - true_state1.width / 2;
////                true_state1.y = kf1.statePost.at<float>(1) - true_state1.height / 2;
////                cv::rectangle(drawing_image, true_state1, CV_RGB(200,200,200), 2);
////
////                cv::Rect true_state2;
////                true_state2.width = kf2.statePost.at<float>(4);
////                true_state2.height = kf2.statePost.at<float>(5);
////                true_state2.x = kf2.statePost.at<float>(0) - true_state2.width / 2;
////                true_state2.y = kf2.statePost.at<float>(1) - true_state2.height / 2;
////                cv::rectangle(drawing_image, true_state2, CV_RGB(200,200,200), 2);
////
////                cv::Rect true_state3;
////                true_state3.width = kf3.statePost.at<float>(4);
////                true_state3.height = kf3.statePost.at<float>(5);
////                true_state3.x = kf3.statePost.at<float>(0) - true_state3.width / 2;
////                true_state3.y = kf3.statePost.at<float>(1) - true_state3.height / 2;
////                cv::rectangle(drawing_image, true_state3, CV_RGB(200,200,200), 2);
////
////                cv::Rect true_state4;
////                true_state4.width = kf4.statePost.at<float>(4);
////                true_state4.height = kf4.statePost.at<float>(5);
////                true_state4.x = kf4.statePost.at<float>(0) - true_state4.width / 2;
////                true_state4.y = kf4.statePost.at<float>(1) - true_state4.height / 2;
////                cv::rectangle(drawing_image, true_state4, CV_RGB(200,200,200), 2);
////
////                cv::Rect true_state5;
////                true_state5.width = kf5.statePost.at<float>(4);
////                true_state5.height = kf5.statePost.at<float>(5);
////                true_state5.x = kf5.statePost.at<float>(0) - true_state5.width / 2;
////                true_state5.y = kf5.statePost.at<float>(1) - true_state5.height / 2;
////                cv::rectangle(drawing_image, true_state5, CV_RGB(200,200,200), 2);
////
////                cv::Rect true_state6;
////                true_state6.width = kf6.statePost.at<float>(4);
////                true_state6.height = kf6.statePost.at<float>(5);
////                true_state6.x = kf6.statePost.at<float>(0) - true_state6.width / 2;
////                true_state6.y = kf6.statePost.at<float>(1) - true_state6.height / 2;
////                cv::rectangle(drawing_image, true_state6, CV_RGB(200,200,200), 2);
//                
//                
//                
//            }
////            cout << "Measure matrix 1:" << endl << meas1 << endl;
////            cout << "Measure matrix 2:" << endl << meas2 << endl;
////            cout << "Measure matrix 2:" << endl << meas3 << endl;
////            cout << "Measure matrix 2:" << endl << meas4 << endl;
////            cout << "Measure matrix 2:" << endl << meas5 << endl;
//        }
//        // <<<<< Kalman Update
//        /////////////////////////////////////////////
//        
//        // Next frame
//        counter++;
//        
////                // Outputting the time elapsed
////                auto finish = std::chrono::high_resolution_clock::now();
////                std::chrono::duration<double> elapsed = finish - start;
////                std::cout << "Elapsed time: " << elapsed.count() << " s\n";
//    }
//    
//    return 0;
//}
//
