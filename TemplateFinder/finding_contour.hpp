#ifndef finding_contour_hpp
#define finding_contour_hpp

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

class fi_con {
    public:
    
    // Functions to image processing
    void nearestNeighbor(Mat image, int max);
    double angle(Point pt1, Point pt2, Point pt0);
    double ratio_check(Point pt1, Point pt2, Point pt0);
    Mat image_processing(Mat input_target_image, int blockSize, int constant_subtractor, VideoWriter video_T);
    vector<vector<Point> > find_squares(Mat smoothed_target, const char *argv[]);
    
};

#endif /* finding_contour_hpp */


