//  template_matching.cpp
//  TemplateFinder

// Visual and Autonomous Exploration Systems Research Laboratory
// Version 1.0 by Qasim Mahmood on 12/30/18
// email-adress: mahmood_qasim@hotmail.com

#include "kalman.hpp"
#include <iostream>
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
#include "template_matching.hpp"
#include <unordered_map>

using namespace std;
using namespace cv;

// Here we compare the two ROIs (template image and target cluster) We increment for a match, decrement for a mismatch
// We are using XOR, so if 01 and 10 are mismatch, 00 and 11 are matching
// We also filter out the image outside the template warping since the bounding box is a straight rectangle but the target might be off axis
// For this, we filter out any grey values, the total pixels are only ones which are within the tempalte/target cluster, not the surrounding areas
float temp_mat::compare_images(Mat image_roi_smoothed, Mat image_roi_template){
    float matched = 0.0;
    float total = 0.0;
    int row=0, col=0;
    
    // For each pixel, if it is not grey, then compare (this filters out the pixel comparisons outside of the template / target cluster
    for(row=1; row<image_roi_smoothed.rows-1; row++){
        for(col=1; col<image_roi_smoothed.cols-1; col++){
            if(image_roi_template.at<uchar>(row,col) != 128){
                total++;
                if(image_roi_template.at<uchar>(row,col) == image_roi_smoothed.at<uchar>(row,col)){matched++;}
                else{matched--;}
            }
        }
    }
    // Return the total matched over the total pixels
    return (matched/total);
}

void temp_mat::find_contours_templates(vector<vector<Point> > squares, Mat smoothed_target, int argc, const char * argv[], unordered_map<string, vector<Point> > &tracking_map, vector<float> &percentages){
    
    // Import all template names
    vector<string> all_templates;
    all_templates.assign(argv + 8, argv + argc);
    
    float maximum_matched = 0.0;
    vector<float>  max_match;
    int best_template = -1;
    Point2f best_coords[4];
    int contour_number = 0;
    
    tracking_map.clear();
    percentages.clear(); 
    
    // Running through each contour, for each contour, use each template, for each template, check for all directions
    for(contour_number=0; contour_number<squares.size(); contour_number++){
        int template_number = 0;
        // Running through all of the templates
        for (template_number=0; template_number<all_templates.size(); template_number++){
            
            // Creating an image matrix for the template to be opened
            Mat  input_template_image;
            // Reading the template in grayscale
            input_template_image = imread(all_templates[template_number].c_str(), IMREAD_GRAYSCALE);
            
            // Converting the template to warp to cluster found
            Point2f srcTri[4];
            Point2f dstTri[4];
            Mat warp_mat(2,4,CV_32FC1);
            Mat src = input_template_image;
            Mat warp_dst = Mat::zeros(smoothed_target.rows, smoothed_target.cols, smoothed_target.type());
            
            int k = 0;
            int rotation[4] = {0,0,0,0};
            
            // Check all four rotations
            for(k=0; k<4; k++){
                // Setting up which of the four rotation we want to try for this templates
                switch(k){
                    case 0: rotation[0] = 3; rotation[1] = 2; rotation[2] = 1; rotation[3] = 0;
                        break;
                    case 1: rotation[0] = 0; rotation[1] = 3; rotation[2] = 2; rotation[3] = 1;
                        break;
                    case 2: rotation[0] = 1; rotation[1] = 0; rotation[2] = 3; rotation[3] = 2;
                        break;
                    case 3: rotation[0] = 2; rotation[1] = 1; rotation[2] = 0; rotation[3] = 3;
                        break;
                }
                
                // Warping the template to the template corner coords to a blank black image into the region of interest
                srcTri[0] = Point2f(0,0);
                srcTri[1] = Point2f(src.cols-1, 0);
                srcTri[2] = Point2f(src.cols-1, src.rows-1);
                srcTri[3] = Point2f(0, src.rows-1);
                
                dstTri[0] = Point2f(squares[contour_number][rotation[0]].x,squares[contour_number][rotation[0]].y);
                dstTri[1] = Point2f(squares[contour_number][rotation[1]].x,squares[contour_number][rotation[1]].y);
                dstTri[2] = Point2f(squares[contour_number][rotation[2]].x,squares[contour_number][rotation[2]].y);
                dstTri[3] = Point2f(squares[contour_number][rotation[3]].x,squares[contour_number][rotation[3]].y);
                
                // Calcualtion of the matrix to do the transformation
                warp_mat = getPerspectiveTransform(srcTri, dstTri);
                
                // Finally, we grab the template, use the matrix generated above, and put it into the region of interest
                // Here we also set the extra pixels, outside of the tempalte, to gray, so that we can compares only the relavant pixels
                warpPerspective(src, warp_dst, warp_mat, smoothed_target.size(), 1, BORDER_CONSTANT, 128);
                
                // Now that we have warped, we have to get only the ROI to save us time/energy in comparing matches
                
                // First create image mask for SMOOTHED TARGET
                Mat mask = Mat::zeros(smoothed_target.size(),CV_8UC1);
                for(size_t i=0; i<squares.size();i++){
                    cv::drawContours( mask, squares, i, cv::Scalar(255,255,255), cv::FILLED);
                }
                
                // We clean up the image, and mask out the noise
                Mat mask_out_smooth = smoothed_target & mask;
                Rect roi = boundingRect(squares[contour_number]); // Bounded region
                
                // Here we create a bounding box for the ROI in mask_out_smooth
                Mat image_roi_smooth = mask_out_smooth(roi);
                // Here we create a bounding box for the ROI in warp_dst
                Mat image_roi_template = warp_dst(roi);
                
                // Compare the two images in the ROI, return the matching score, only compares what is inside the bounding box, and only that is within the contour
                float current = compare_images(image_roi_smooth, image_roi_template);
                
                // Keeping the best template, storing the coordinates
                if(maximum_matched < current){
                    best_template = template_number;
                    maximum_matched = current;
                    
                    best_coords[0] = Point2f(squares[contour_number][rotation[0]].x,squares[contour_number][rotation[0]].y);
                    best_coords[1] = Point2f(squares[contour_number][rotation[1]].x,squares[contour_number][rotation[1]].y);
                    best_coords[2] = Point2f(squares[contour_number][rotation[2]].x,squares[contour_number][rotation[2]].y);
                    best_coords[3] = Point2f(squares[contour_number][rotation[3]].x,squares[contour_number][rotation[3]].y);
                }
            }
        }
    
        // Only assign a name if it is larger than 70% match 
        if(maximum_matched > 0.70) {
            // Create vector with 4 points for the contour
            vector<Point> best_coordinates;
            best_coordinates.push_back(best_coords[0]);
            best_coordinates.push_back(best_coords[1]);
            best_coordinates.push_back(best_coords[2]);
            best_coordinates.push_back(best_coords[3]);
            
            // Add name and coords to tracking function
            string template_name = "temp" + to_string(best_template+1);
            
            // Check if the pattern has already been found, if so, if percentage of match is higher, then replace
            int i=0;
            int flag=0;
            unordered_map<string, Point2f > itr;
            // Loop iterating through all of the map, using this instead of find because we also need index i for percentages vector
            for (std::pair<string,vector<Point> > element : tracking_map){ // Checking all patterns in tracking_map so far
                i++;
                if(template_name == element.first){ // Found template previously!
                    if(percentages[i] > maximum_matched){ // If new match larger, then swap, otherwise ignore
                        tracking_map.erase(element.first); // Remove the template match
                        percentages.erase(percentages.begin() + i); // Remove the percentage
                        tracking_map.insert( make_pair(template_name, best_coordinates) ); // Push back new template match
                        percentages.push_back(maximum_matched); // Push back a new percentage
                        flag=1;
                        cout << "Found template again! Keeping max best" << endl;
                    }
                }
            }
            
            // If not found, add to the tracking
            if(flag == 0){
                tracking_map.insert( make_pair(template_name, best_coordinates) );
                percentages.push_back(maximum_matched);
            }
        }
        
//        cout << "There are a total of: " << all_templates.size() << " templates" << endl << endl;
//        cout << "The best template is: pat" << best_template+1 << endl << endl;
//        cout << "The score for this is: " << maximum_matched << endl << endl;
//        cout << "The coordinates of this match are for: " << best_coords[0] << "," << best_coords[1] << "," <<  best_coords[2] << "," << best_coords[3] << endl << endl << endl;
        
        // Resetting the values for the next contour
        maximum_matched = 0.0;
        best_template = -1;
    }
    return; 
}
