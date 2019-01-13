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
//    // Timing the code
//    auto start = std::chrono::high_resolution_clock::now();
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
//    string folder = "frames/";
//    string suffix = ".jpg";
//    int counter = 0;
//
//    // Loop over all frames in folder
//    while(1) {
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
////        imshow("frame"  , input_target_image);
////        waitKey(0);
//
//        // If the image is empty, then do this
//        if(input_target_image.empty()){
//            cout <<  "Could not open or find the image" << endl ;
//            return -1;}
//
//        // Resizing the image down 3 times, placing into resized Mat
//        Size size(input_target_image.size().width/3, input_target_image.size().height/3);
//        resize(input_target_image, resized_input_target_image, size, INTER_LINEAR);
//
//        Mat contrast_enhanced_target = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//        Mat smoothed_target = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//        Mat thresh_smoothed_enhanced_target = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//
//        // Contrast image by equalizing histogram
//        equalizeHist(resized_input_target_image, contrast_enhanced_target);
//    //    imshow( "equalized_window", contrast_enhanced_target );
//    //    waitKey(0);
//
//        // Apply blur to image
//        bilateralFilter(contrast_enhanced_target, smoothed_target, 12, 50.0, 50.0);
//    //    imshow( "Blurred Target Image", smoothed_target);
//    //    waitKey(0);
//
//    //////////////////////////////////////////////////////////////////////////////////////////s//////////////////////
//
//        int histSize = 256;
//        float range[] = {0, 256};
//        const float* histRange = { range };
//        bool uniform = true; bool accumulate = false;
//        Mat gray_hist;
//
//        // Calculate the histogram after contrasting
//        calcHist( &smoothed_target, 1, 0, Mat(), gray_hist, 1, &histSize, &histRange, uniform, accumulate );
//
//    //    imshow("equal" , smoothed_target);
//    //    waitKey(0);
//
//        int hist_w = 512; int hist_h = 400;
//        int bin_w = cvRound( (double) hist_w/histSize );
//        Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 255,255,255) );
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
//    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////    //  If you want to print out the histogram
////        normalize(gray_hist, gray_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
////        for( int i = 1; i < histSize; i++ ){
////            line( histImage, Point( bin_w*(i-1), hist_h - cvRound(gray_hist.at<float>(i-1)) ) , Point( bin_w*(i), hist_h - cvRound(gray_hist.at<float>(i)) ), Scalar( 0, 255, 0), 2, 8, 0  );
////
////        }
////        imshow("calcHist Demo", histImage );
////        waitKey(0);
////
//    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
//        // This is drawing the contours! Don't really need this, only for display
//        Mat image = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//        for(size_t i = 0; i < squares.size(); i++ )
//        {
//            // Check if the lines isn't on the edge of the picture
//            const Point* p = &squares[i][0];
//            int n = (int)squares[i].size();
//            //dont detect the border
//            if (p-> x > 3 && p->y > 3)
//                // Connect the four verticies with a line
//                polylines(image, &p, &n, 1, true, Scalar(170,170,170), 2, LINE_AA);
//        }
//
//
//        // Import all template names
//        vector<string> all_templates;
//        all_templates.assign(argv + 4, argv + argc);
//
//        float maximum_matched = 0.0;
//        int best_template = NULL;
//        Point2f best_coords[4];
//        int contour_number = 0;
//
//        // Running through each contour, for each contour, use each template, for each template, check for all directions
//        for(contour_number=0; contour_number<squares.size(); contour_number++){
//            int template_number = 0;
//            // Running through all of the templates
//            for (template_number=0; template_number<all_templates.size(); template_number++){
//
//                // Creating an image matrix for the template to be opened
//                Mat  input_template_image;
//                // Reading the template in grayscale
//                input_template_image = imread(all_templates[template_number].c_str(), IMREAD_GRAYSCALE);
//
//                // Converting the template to warp to cluster found
//                Point2f srcTri[4];
//                Point2f dstTri[4];
//                Mat warp_mat(2,4,CV_32FC1);
//                Mat src = input_template_image;
//                Mat warp_dst = Mat::zeros(smoothed_target.rows, smoothed_target.cols, smoothed_target.type());
//
//                int k = 0;
//                int rotation[4] = {0,0,0,0};
//
//                // Check all four rotations
//                for(k=0; k<4; k++){
//                    // Setting up which of the four rotation we want to try for this template
//                    switch(k){
//                        case 0: rotation[0] = 3; rotation[1] = 2; rotation[2] = 1; rotation[3] = 0;
//                            break;
//                        case 1: rotation[0] = 0; rotation[1] = 3; rotation[2] = 2; rotation[3] = 1;
//                            break;
//                        case 2: rotation[0] = 1; rotation[1] = 0; rotation[2] = 3; rotation[3] = 2;
//                            break;
//                        case 3: rotation[0] = 2; rotation[1] = 1; rotation[2] = 0; rotation[3] = 3;
//                            break;
//                    }
//
//                    // Warping the template to the template corner coords to a blank black image into the region of interest
//                    srcTri[0] = Point2f(0,0);
//                    srcTri[1] = Point2f(src.cols-1, 0);
//                    srcTri[2] = Point2f(src.cols-1, src.rows-1);
//                    srcTri[3] = Point2f(0, src.rows-1);
//
//                    dstTri[0] = Point2f(squares[contour_number][rotation[0]].x,squares[contour_number][rotation[0]].y);
//                    dstTri[1] = Point2f(squares[contour_number][rotation[1]].x,squares[contour_number][rotation[1]].y);
//                    dstTri[2] = Point2f(squares[contour_number][rotation[2]].x,squares[contour_number][rotation[2]].y);
//                    dstTri[3] = Point2f(squares[contour_number][rotation[3]].x,squares[contour_number][rotation[3]].y);
//
//                    // Calcualtion of the matrix to do the transformation
//                    warp_mat = getPerspectiveTransform(srcTri, dstTri);
//
//                    // Finally, we grab the template, use the matrix generated above, and put it into the region of interest
//                    // Here we also set the extra pixels, outside of the tempalte, to gray, so that we can compares only the relavant pixels
//                    warpPerspective(src, warp_dst, warp_mat, smoothed_target.size(), 1, BORDER_CONSTANT, 128);
//
//                    // Now that we have warped, we have to get only the ROI to save us time/energy in comparing matches
//
//                    // First create image mask for SMOOTHED TARGET
//                    Mat mask = Mat::zeros(smoothed_target.size(),CV_8UC1);
//                    for(size_t i=0; i<squares.size();i++){
//                        cv::drawContours( mask, squares, i, cv::Scalar(255,255,255), cv::FILLED);
//                    }
//
//                    // We clean up the image, and mask out the noise
//                    Mat mask_out_smooth = smoothed_target & mask;
//                    Rect roi = boundingRect(squares[contour_number]); // Bounded region
//
//                    // Here we create a bounding box for the ROI in mask_out_smooth
//                    Mat image_roi_smooth = mask_out_smooth(roi);
//                    // Here we create a bounding box for the ROI in warp_dst
//                    Mat image_roi_template = warp_dst(roi);
//
//                    // UNCOMMENT TO SHOW DR. FINK YAYAYAYAYA :D
//                    imshow("template" , image_roi_template);
//                    imshow("target" , image_roi_smooth);
//                    imshow("all targets" , mask_out_smooth);
//                    waitKey(0);
//
//                    // Compare the two images in the ROI, return the matching score, only compares what is inside the bounding box, and only that is within the contour
//                    float current = compare_images(image_roi_smooth, image_roi_template);
//
//                    // Keeping the best template, storing the coordinates
//                    if(maximum_matched < current){
//                        best_template = template_number;
//                        maximum_matched = current;
//                        best_coords[0] = Point2f(squares[contour_number][rotation[0]].x,squares[contour_number][rotation[0]].y);
//                        best_coords[1] = Point2f(squares[contour_number][rotation[1]].x,squares[contour_number][rotation[1]].y);
//                        best_coords[2] = Point2f(squares[contour_number][rotation[2]].x,squares[contour_number][rotation[2]].y);
//                        best_coords[3] = Point2f(squares[contour_number][rotation[3]].x,squares[contour_number][rotation[3]].y);
//                    }
//                }
//            }
//
//            cout << "There are a total of: " << all_templates.size() << " templates" << endl << endl;
//            cout << "The best template is: pat" << best_template+1 << endl << endl;
//            cout << "The score for this is: " << maximum_matched << endl << endl;
//            cout << "The coordinates of this match are for: " << best_coords[0] << "," << best_coords[1] << "," <<  best_coords[2] << "," << best_coords[3] << endl << endl << endl;
//
//            // Resetting the values for the next template
//            maximum_matched = 0.0;
//            best_template = NULL;
//        }
//
//        // Outputting the time elapsed
//        auto finish = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double> elapsed = finish - start;
//        std::cout << "Elapsed time: " << elapsed.count() << " s\n";
//
//        // mask_out_smooth (our image with all the targets we want to identify, masked out the noise)
//        // image_roi (region of where we want to narrow our convolution to
//        // warp_dst (input template that is the size of smoothed targets)
//
//    ///////////////////////////////////////////////////////////////////////////////////////////////////
//
//    //    // Use below to go through images in a directory!
//    //    String folderpath = "d:/mymages/*.jpg"
//    //    vector<String> filenames;
//    //    cv::glob(folderpath, filenames);
//    //
//    //    for (size_t i=0; i<filenames.size(); i++)
//    //    {
//    //        Mat im = imread(filenames[i]);
//    //        ... your processing here
//    //    }
//        counter++;
//    }
//
//    return 0;
//}
//
