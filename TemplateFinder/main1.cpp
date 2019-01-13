// Visual and Autonomous Exploration Systems Research Laboratory
// Version 1.0 by Qasim Mahmood on 12/30/18
// email-adress: mahmood_qasim@hotmail.com

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc.hpp>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <chrono>
#include "finding_contour.hpp"
#include "kalman.hpp"
#include "template_matching.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"

using namespace std;
using namespace cv;

/*
The README.md file explains the contents of the entire object_tracking program.
 This file contains the main function.
 This iterative code has three main sections for each frame: image processing, finding high interest contours, and tempalte matching.
 Three files are needed for this codebase: main1.cpp, finding_contour.cpp, and template_matching.cpp
 
 The loop for each frame lies between 94 and 185. This has three parts, image processing, finding high interest contours, and tempalte matching.
 
 */

int main(int argc, const char * argv[]) {
    
    // Defining classes for finding contour, kalman filter, and pairs for contour/template-match
    fi_con f;

    // Map to keep track of the contours and identification
    unordered_map< string, vector<Point> > tracking_map;
    
    // Defining the number of patterns, the final area we want to maximize to (currently not used), and the resolution we wish to operate on
    int blockSize, constant_subtactor;
    // How many patterns to find in the target image
    sscanf(argv[6], "%d", &blockSize);
    // What's the final resolution you want to expand the piture to
    sscanf(argv[7], "%d", &constant_subtactor);
    
    // Creating an image matrix for the image to be opened
    Mat  input_target_image, resized_input_target, drawing_image;
    
    // Iteration through frames of video / folder, start with frame 0
    string folder = "videos/";
    string suffix = ".jpg";
    
    // Create video capture
    VideoCapture cap(folder+argv[1]);
    
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    
    Mat frame;
    cap >> frame;
    // Find resize factor by taking the larger of the width/height and dividing by 640
    int largest_length = int(max(frame.size().width, frame.size().height));
    int resize_factor = largest_length/640;
    // Incase the image is smaller than 640
    if(resize_factor < 1){ resize_factor = 1; }
    Size S(dWidth/resize_factor, dHeight/resize_factor);
    string out_T = string(argv[1]) + "_out_thrsh.m4v";
    string out_D = string(argv[1]) + "_out_track.m4v";
    string out_G = string(argv[1]) + "_out_gray.m4v";
    
    if( remove(out_T.c_str()) != 0 )
        perror( "Threshold Video" );
    else
        puts( "Previous thresholding video successfully deleted" );
    
    if( remove(out_D.c_str()) != 0 )
        perror( "Drawing Video" );
    else
        puts( "Previous drawing video successfully deleted" );
    
    // Writing the output videos (threshold and drawing videos)
    VideoWriter video_T(out_T, 0x31637661, 15, S, false);
    VideoWriter video_D(out_D, 0x31637661, 15, S, true);
    VideoWriter video_G(out_G, 0x31637661, 15, S, false);
    
    // Loop over all frames in the video
    while(1) {
        
        // Timing the code
        auto start = std::chrono::high_resolution_clock::now();
        
        // Reading the next frame as grayscale
        cap >> input_target_image;
        
        // If the image is empty
        if(input_target_image.empty()){
            cout <<  "End of video" << endl ;
            cap.release();
            video_T.release();
            video_D.release();
            return -1;}
        
        // Convert frame to grayscale
        cvtColor(input_target_image, input_target_image, CV_RGB2GRAY);

        // Find resize factor by taking the larger of the width/height and dividing by 640
        int largest_length = int(max(input_target_image.size().width, input_target_image.size().height));
        int resize_factor = largest_length/640;
        // Incase the image is smaller than 640
        if(resize_factor < 1){ resize_factor = 1; }

        // Resize the image
        Size size(input_target_image.size().width/resize_factor, input_target_image.size().height/resize_factor);
        resize(input_target_image, resized_input_target, size, INTER_LINEAR);
        
        // Used to create images for the paper
        Mat input_color;
        cap >> input_color;
        // If the image is empty
        if(input_color.empty()){
            cout <<  "End of video" << endl ;
            cap.release();
            video_T.release();
            video_D.release();
            return -1;}
        
        resize(input_color, drawing_image, size, INTER_LINEAR);
        
        // Image processing the image
        Mat smoothed_target = f.image_processing(resized_input_target, blockSize, constant_subtactor, video_T);
        
        // Get all contours that are of square shape
        vector<vector<Point> > squares = f.find_squares(smoothed_target, argv);
        
        // After finding squares, we associate each square with the template found
        temp_mat m;
        vector<float> percentages;
        m.find_contours_templates(squares, smoothed_target, argc, argv, tracking_map, percentages);
        
        // Print out percentages
        // cout << tracking_map.size() << " , " << percentages.size() << endl;
        
        unordered_map<string, Point2f > itr;
        // Iterate over the contours and draw in the bounding rectangle on the measured contour
        for (std::pair<string,vector<Point> > element : tracking_map){
            // Gray scale image
            cv::rectangle(resized_input_target, cv::boundingRect(element.second), cv::Scalar(200,200,200), 2);
            cv::putText(resized_input_target, element.first, element.second[1], cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255,255,255));
            
            // Color image
            cv::rectangle(drawing_image, cv::boundingRect(element.second), cv::Scalar(0,0,255), 2);
            cv::putText(drawing_image, element.first, element.second[1], cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(0,255,255));
        }
        
        // Drawing the contours!s
        int i=0;
        // Drawing all the contours found in measurement
        for(i = 0; i < squares.size(); i++ ){
            // Check if the lines isn't on the edge of the picture
            const Point* p = &squares[i][0];
            int n = (int)squares[i].size();
            //dont detect the border
            if (p-> x > 3 && p->y > 3)
                // Connect the four verticies with a line
                polylines(resized_input_target, &p, &n, 1, true, cv::Scalar(160,160,160), 2, LINE_AA);
                // Drawing on color image
                polylines(drawing_image, &p, &n, 1, true, cv::Scalar(255,0, 0), 2, LINE_AA);
        }
        
        imshow("drawing" , drawing_image);
        imshow("gray" , resized_input_target);
        moveWindow("gray" , 0,200);
        video_D.write(drawing_image);
        video_G.write(resized_input_target);
        waitKey(5);
        
        // Outputting the time elapsed
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        std::cout << "Elapsed time: " << elapsed.count() << " s\n";
    }
    return 0;
}

