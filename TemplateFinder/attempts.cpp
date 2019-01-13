//
// Visual and Autonomous Exploration Systems Research Laboratory
// Created by Qasim Mahmood on 3/25/18.
// Copyright © 2018 Qasim Mahmood. All rights reserved.
// This code has the severeal attempts for finding the targets in the given image
// For example, there are versions of k-means, blob detection, hough line transform,
// probablistic line transform, contouring finding, etc. 
// This code is only useful for Qasim, won't be very useful for others since it's just scrap code.
//

// #include <stdio.h>

//    // Show our contrast enhranced image
//    imshow( "Input Target Image", smoothed_target);
//    // Wait for a keystroke in the window
//    waitKey(0);


//    Mat src = smoothed_target;
//    Mat samples(smoothed_target.rows * smoothed_target.cols, 1, CV_32F);
//    for( int y = 0; y < smoothed_target.rows; y++ )
//        for( int x = 0; x < smoothed_target.cols; x++ )
//            samples.at<float>(y + x*smoothed_target.rows) = smoothed_target.at<uchar>(y,x);
//
//
//    int clusterCount = 2;
//    Mat labels;
//    int attempts = 5;
//    Mat centers;
//    double compactness = kmeans(samples, clusterCount, labels, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0), attempts, 0, centers );
//
//
//    Mat new_image( smoothed_target.size(), smoothed_target.type() );
//    for( int y = 0; y < smoothed_target.rows; y++ )
//        for( int x = 0; x < smoothed_target.cols; x++ )
//        {
//            int cluster_idx = labels.at<int>(y + x*smoothed_target.rows, 0);
//            new_image.at<uchar>(y,x) = centers.at<float>(cluster_idx, 0);
//        }
//    imshow( "clustered image", new_image );
//    waitKey( 0 );
//
//
//    cout << centers << endl;
//    cout << compactness << endl;


////////////////////////////////////////
///////// K means using OpenCV /////////
////////////////////////////////////////
//    int numberOfClusters = 2;
//    int n = smoothed_target.rows * smoothed_target.cols;
//    Mat data = smoothed_target.reshape(1, n);
//    data.convertTo(data, CV_32F);
//    TermCriteria criteria {CV_TERMCRIT_ITER, 10, 0.1};
//
//    Mat labels;
//    Mat1f centers;
//    double compactness = kmeans(data, numberOfClusters, labels, criteria, 6, cv::KMEANS_RANDOM_CENTERS, centers);
//
//
//    for (int i = 0; i < n; ++i)
//    {
//        data.at<float>(i, 0) = centers(labels.at<int>(i), 1);
//    }
//
//
//    Mat reduced = data.reshape(1, smoothed_target.rows);
//    reduced.convertTo(reduced, CV_8U);
//
//    show_result(labels, centers, smoothed_target.rows, smoothed_target.cols);
//
//    imshow("Reduced", reduced);
//    waitKey();

/////////////////////////////////////////////
/////// Probablistic Hough Lines ////////////
/////////////////////////////////////////////
//    Mat src = smoothed_target;
//    Mat cdst;
//    Mat test;
//    Mat test2;
//
//    // Edge detection
//    Canny(src, test, 50, 200, 3);
//    dilate(test, test2, Mat());
//
////    imshow("Edge Detection" , test2);
////    waitKey(0);
//    // Copy edges to the images that will display the results in BGR
//    cvtColor(test2, cdst, COLOR_GRAY2BGR);
//
//    // Probabilistic Line Transform
//    vector<Vec4i> linesP; // will hold the results of the detection
//    HoughLinesP(test2, linesP, 1, CV_PI/180, 40, 40, 4 ); // runs the actual detection
//    // Draw the lines
//    for( size_t i = 0; i < linesP.size(); i++ ){
//        Vec4i l = linesP[i];
//        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
//    }
//
//    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdst);
//    waitKey(0);

// Next we want to try other types of detection
// - Edge detection
// - Blob detection
// - Polygon fitting
// - Expanding the white region method??
//
// Take pictures at different times of day
// What happens when you add another template and only want to detect 3
// Find test cases
/////////////////////////////////////////////
/////////////////////////////////////////////
/////// Now trying blob dection //////
///////////////////////////////////

//    bitwise_not(smoothed_target, thresh_smoothed_enhanced_target);
//    smoothed_target = thresh_smoothed_enhanced_target;
//
//    erode(smoothed_target, thresh_smoothed_enhanced_target, Mat());
//    smoothed_target = thresh_smoothed_enhanced_target;
//    erode(smoothed_target, thresh_smoothed_enhanced_target, Mat());
//    smoothed_target = thresh_smoothed_enhanced_target;
//    erode(smoothed_target, thresh_smoothed_enhanced_target, Mat());
//    smoothed_target = thresh_smoothed_enhanced_target;
//    erode(smoothed_target, thresh_smoothed_enhanced_target, Mat());
//    smoothed_target = thresh_smoothed_enhanced_target;
//    erode(smoothed_target, thresh_smoothed_enhanced_target, Mat());
//    smoothed_target = thresh_smoothed_enhanced_target;
//    erode(smoothed_target, thresh_smoothed_enhanced_target, Mat());
//    smoothed_target = thresh_smoothed_enhanced_target;
//
////    imshow("smoothed Targe", smoothed_target);
////    waitKey(0);
//
//
//    Mat im = smoothed_target;
//
//    SimpleBlobDetector::Params params;
//
////    params.filterByColor = true;
////    params.blobColor=0;
//
//    params.filterByArea = true;
//    params.minArea = 500;
//
////    params.filterByCircularity = true;
////    params.maxCircularity = 0.785;
//
////    params.filterByConvexity = true;
////    params.minConvexity = 0.5;
////    params.filterByInertia = false;
//
//    // Set up the detector with default parameters.
//    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
//
//
//    // Detect blobs.
//    std::vector<KeyPoint> keypoints;
//
//    detector->detect( im, keypoints);
//
//
//    // Draw detected blobs as red circles.
//    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
//    Mat im_with_keypoints = Mat::zeros( resized_input_target_image.size(), resized_input_target_image.type());
//    drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
//
//    // Show blobs
//    imshow("keypoints", im_with_keypoints );
//    waitKey(0);


////////////////////////////////////////////////

//    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//    for( size_t i = 0; i< contours.size(); i++ )
//    {
//        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
//    }
//    namedWindow( "Contours", WINDOW_AUTOSIZE );
//    imshow( "Contours", drawing );
//    waitKey(0);
// ///////////////////
//
//    Mat src_copy = smoothed_target;
//    Mat threshold_output;
////    vector<vector<Point> > contours;
////    vector<Vec4i> hierarchy;
//
//
//    /// Detect edges using Threshold
//    threshold( src_copy, threshold_output, thresh, 255, THRESH_BINARY );
//
//    /// Find contours
//    findContours( threshold_output, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
//
//    vector<vector<Point> > contours2;
//    int i =0;
//    for(i=0; i<contours.size();i++){
//        if(contours[i].size() <=200){
//            contours2.push_back(contours[i]);
//        }
//    }
//
//    cout << 'hi' << endl;
//    contours = contours2;
//
//    /// Find the convex hull object for each contour
//    vector<vector<Point>> hull( contours.size() );
//    for( int i = 0; i < contours.size(); i++ )
//    {  convexHull( Mat(contours[i]), hull[i], false ); }
//
////    /// Draw contours + hull results
//    drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
////    for( int i = 0; i< contours.size(); i++ )
////    {
////        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
////        //drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
////        drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
////    }
////
////    /// Show in a window
////    namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
////    imshow( "Hull demo", drawing );
////    waitKey(0);
//
//
//    int k=0;
//    for(k=0; k<contours.size(); k++)
//    {
//        if(hull[k].size() >= 17){
//        RotatedRect rotated = minAreaRect(hull[k]);
//        Point2f rect_points[4];
//        rotated.points( rect_points );
//        for( int j = 0; j < 4; j++ ) {
//            line( drawing, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,255), 1.5, 8 );
//        }
//    }
//    }
//        namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
//        imshow( "Hull demo", drawing );
//        waitKey(0);
////
////
////
////
//

////////////////////////////////////////////////////////
// Cropping out the region to match, want to grab the region bounded by the squares we found.
// Extract that and then use templateMatch function from OpenCV
////////////////////////////////////////////////////////
//// Black output
//Mat imgOut = Mat::ones(resized_input_target_image.size(), resized_input_target_image.type());
//imgOut = Scalar(1.0,1.0,1.0);
//
//
//// Input triangle
//vector <Point2f> tri1;
//tri1.push_back(Point2f(360,200));
//tri1.push_back(Point2d(60,250));
//tri1.push_back(Point2f(450,400));
//
//// Output triangle
//vector <Point2f> tri2;
//tri2.push_back(Point2f(400,200));
//tri2.push_back(Point2f(160,270));
//tri2.push_back(Point2f(400,400));
//
//// Find bounding rectangle for each triangle
//Rect r1 = boundingRect(tri1);
//Rect r2 = boundingRect(tri2);
//
//// Offset points by left top corner of the respective rectangles
//vector<Point2f> tri1Cropped, tri2Cropped;
//vector<Point> tri2CroppedInt;
//
//for(int i = 0; i < 3; i++)
//{
//    tri1Cropped.push_back( Point2f( tri1[i].x - r1.x, tri1[i].y -  r1.y) );
//    tri2Cropped.push_back( Point2f( tri2[i].x - r2.x, tri2[i].y - r2.y) );
//
//    // fillConvexPoly needs a vector of Point and not Point2f
//    tri2CroppedInt.push_back( Point((int)(tri2[i].x - r2.x), (int)(tri2[i].y - r2.y)) );
//}
//
//// Apply warpImage to small rectangular patches
//Mat img1Cropped;
//resized_input_target_image(r1).copyTo(img1Cropped);
//
//// Given a pair of triangles, find the affine transform.
//Mat warpMat = getAffineTransform( tri1Cropped, tri2Cropped );
//
//// Apply the Affine Transform just found to the src image
//Mat img2Cropped = Mat::zeros(r2.height, r2.width, img1Cropped.type());
//warpAffine( img1Cropped, img2Cropped, warpMat, img2Cropped.size(), INTER_LINEAR, BORDER_REFLECT_101);
//
//// Get mask by filling triangle
//Mat mask = Mat::zeros(r2.height, r2.width, CV_32FC3);
//fillConvexPoly(mask, tri2CroppedInt, Scalar(1.0, 1.0, 1.0), 16, 0);
//
//// Copy triangular region of the rectangular patch to the output image
//multiply(img2Cropped,mask, img2Cropped);
//multiply(imgOut(r2), Scalar(1.0,1.0,1.0) - mask, imgOut(r2));
//imgOut(r2) = imgOut(r2) + img2Cropped;
//
//imshow("triangle" , imgOut);
//waitKey(0);

//// This is a self made contrast enhance function
//void contrastEnhance(float alpha, Mat image, Mat new_image){
//    /// Do the operation new_image(i,j) = (alpha*(image(i,j) + ave) - ave)
//    int x=0, y=0, sum=0, ave=0;
//    int temp;
//    
//    double min, max;
//    minMaxLoc(image, &min, &max);
//    if(min == max){
//        min = 0;
//        max = 1;
//    }
//    for(y = 0; y < image.cols; y++ ){
//        for(x = 0; x < image.rows; x++){
//            sum += image.at<uchar>(x,y);
//        }
//    }
//    ave = sum/(image.size().width * image.size().height);
//    for(y = 0; y < image.cols; y++ ){
//        for(x = 0; x < image.rows; x++){
//            temp = (short)(alpha*(image.at<uchar>(x,y) - ave) + ave);
//            temp = (temp > 255) ? 255 : temp;
//            temp = (temp < 0) ? 0 : temp;
//            new_image.at<uchar>(x,y) = temp;
//        }
//    }
//    return;
//}
