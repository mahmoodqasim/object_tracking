//
//  template_matching.hpp
//  TemplateFinder
//
//  Created by Qasim Mahmood on 10/8/18.
//  Copyright © 2018 Qasim Mahmood. All rights reserved.
//

#ifndef template_matching_hpp
#define template_matching_hpp

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

//void find_contours_templates(vector<vector<Point> > squares, Mat smoothed_target, int argc, const char * argv[]);

class temp_mat{
public:
    void find_contours_templates(vector<vector<Point> > squares, Mat smoothed_target, int argc, const char * argv[], unordered_map<string, vector<Point> > &tracking_map, vector<float> &percentages);
    float compare_images(Mat image_roi_smoothed, Mat image_roi_template);

};


#endif /* template_matching_hpp */
