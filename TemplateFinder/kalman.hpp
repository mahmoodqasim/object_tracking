//
//  kalman.hpp
//  TemplateFinder
//
//  Created by Qasim Mahmood on 10/7/18.
//  Copyright © 2018 Qasim Mahmood. All rights reserved.
//

#ifndef kalman_hpp
#define kalman_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class kal_fil{
    public:
    
    void setUpKalman(KalmanFilter kf, Mat state, Mat meas, int stateSize, int measSize, unsigned int type);
    
}; 


#endif /* kalman_hpp */



