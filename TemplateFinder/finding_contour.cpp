//  finding_contour.cpp
//  TemplateFinder

// Visual and Autonomous Exploration Systems Research Laboratory
// Version 1.0 by Qasim Mahmood on 12/30/18
// email-adress: mahmood_qasim@hotmail.com

#include <stdio.h>
#include <iostream>
#include <ctype.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <chrono>
#include "finding_contour.hpp"
#include <unordered_map>

using namespace std;
using namespace cv;


// Get angle to find squares
double fi_con::angle( Point pt1, Point pt2, Point pt0 ){
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    
    
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// Returns the largest ratio using all of the sides in the polygon
double fi_con::ratio_check( Point pt1, Point pt2, Point pt0 ){
    
    double dx1 = pow(pt0.x - pt1.x,2);
    double dy1 = pow(pt0.y - pt1.y,2);
    double len1 = sqrt(dx1 + dy1);
    
    double dx2 = pow(pt0.x - pt2.x,2);
    double dy2 = pow(pt0.y - pt2.y,2);
    double len2 = sqrt(dx2 + dy2);

    double val1 = abs(len1/len2);
    double val2 = abs(len2/len1);
    
    double ratio = MAX(val1, val2);
    
    return ratio;
}

// Nearest neighbors to clean up noise
void fi_con::nearestNeighbor(Mat image, int max){
    int row=1, col=1, NN=0;
    for(row=1; row<image.rows-1; row++){
        for(col=1; col<image.cols-1; col++){
            NN = 0;
            NN += image.at<uchar>(row-1,col-1);
            NN += image.at<uchar>(row-1,col);
            NN += image.at<uchar>(row-1,col+1);
            NN += image.at<uchar>(row,col-1);
            NN += image.at<uchar>(row,col+1);
            NN += image.at<uchar>(row+1,col-1);
            NN += image.at<uchar>(row+1,col);
            NN += image.at<uchar>(row+1,col+1);
            if( NN<(4*max) ){image.at<uchar>(row,col) = 0;}
        }
    }
    return;
}

// Process the image: equalize histrogram, blur, threshold, return binarized image
Mat fi_con::image_processing(Mat input_target_image, int blockSize, int constant_subtractor, VideoWriter video_T){
    
    Mat contrast_enhanced_target = Mat::zeros( input_target_image.size(), input_target_image.type());
    Mat smoothed_target = Mat::zeros( input_target_image.size(), input_target_image.type());
    Mat thresh_smoothed_enhanced_target = Mat::zeros( input_target_image.size(), input_target_image.type());
    
    // Contrast image by equalizing histogram
    equalizeHist(input_target_image, contrast_enhanced_target);
    
    // Apply blur to image
    bilateralFilter(contrast_enhanced_target, smoothed_target, 10, 50.0, 50.0);
    
    // Find min/max of image for Nearest neighbor
    double min, max;
    minMaxLoc(smoothed_target, &min, &max);
    
    // Apply a threshold to the image
    adaptiveThreshold(smoothed_target, smoothed_target, 256, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blockSize, constant_subtractor);
    
//    imshow("smoothed" , smoothed_target);
//    waitKey(0);

    // Perform nearest neighbor
    nearestNeighbor(smoothed_target, max);
    
    // Displaying images
    imshow("smoothed" , smoothed_target);
    video_T.write(smoothed_target);
    moveWindow("smoothed" , 370,50);
    
    return smoothed_target;
}

// Find contours that look like squares: get all contours, find ones that looks like squares, return squares contours
vector<vector<Point> > fi_con::find_squares(Mat smoothed_target, const char * argv[]){

    RNG rng(123);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    float maxC, maxR, minA, maxA;
    sscanf(argv[2], "%f", &maxC);
    sscanf(argv[3], "%f", &maxR);
    sscanf(argv[4], "%f", &minA);
    sscanf(argv[5], "%f", &maxA);
    
    // Find contours in the image
    findContours( smoothed_target, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    vector<vector<Point> > squares;
    vector<Point> approx;
    
    // Test the contours with our criteria to find squares of ceratin size
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // approximate contour with accuracy proportional to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.035, true);

        // The square contours should have 4 vertices after approximation relatively large area (to filter out noisy contours) and be convex. Note: absolute value of an area is used because area may be positive or negative - in accordance with the contour orientation
        if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > minA && isContourConvex(Mat(approx)) && fabs(contourArea(Mat(approx))) < maxA)
        {
            double maxCosine = 0;
            double maxRatio = 0;
            for( int j = 2; j < 5; j++ ){ 
                // Find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                double ratio_now = ratio_check(approx[j%4], approx[j-2], approx[j-1]);
                maxRatio = MAX(maxRatio, ratio_now);
                maxCosine = MAX(maxCosine, cosine);
            }
            
            // if cosines of all angles are small (all angles are ~90 degree) then write quandrange vertices to resultant sequence
            // We also check the lengths of the ratio of the sides
            if( maxCosine < maxC && maxRatio < maxR) {
//                cout << maxCosine << " " << maxRatio << endl;
                squares.push_back(approx);
            }
        }
    }
    return squares;
    
}
