## This is a README for the Object Tracking Project. 

### Summary 
This iterative code has three main sections for each frame: image processing, finding high interest contours, and tempalte matching.
Three files are needed for this codebase: main1.cpp, finding_contour.cpp, and template_matching.cpp

The first 7 parameters are fixed, anything after is used as a template to check against (there needs to be at least 1 template, up to any ammount).

Usage: ./object_tracking video.m4v maxCosine maxRatio minArea maxArea blockSize constant_subtractor template1.jpg template2.jpg template3.jpg template4.jpg . . . 
Example Usage: ./object_tracking vid2.m4v 0.7 5.0 125 5000 91 -91 temp1.jpg temp2.jpg temp3.jpg 

User-defined Parameters:
- blockSize: Comes with the adaptive thresholding method. It is a off number chosen (1,3,5,7,...). The larger the number, the larger the region the adaptive threshold looks at when binarizing the current pixel to 0 or 1. A number too small will outline the edges, which is not necessary what we want. We use a number usually above 31 up to 351).
- constant_subtractor: Comes with the adaptive thresholding method. It is any integer (positive or negative) chosen to be subtracted from the image. We use a negative number (usually between -30 to -150, but can be different). It helps remove areas that aren't as definetly in their binarization.
- minArea/maxArea: If we know how far we are from the targets, then we can include some constraints to speed up the code but specificing a min/max area for the polygons found in the video. As a wide range, we can set the min to 50/100 and max to 10,000. If we know more about the size, then the min and max can be changed.
- maxCosine: This is a constraint on the largest angle in the polygon. If we know that we are directly overhead of the targets, then we know the angles of the targets will be closer to 90-degrees. If we are not above the targets (i.e. the camera is not parallel to the gorund), then we can relax this parameter to allow larger angles (i.e. templates which are more skewed).
- maxRatio: This parameter restricts the sides ratio of the polygons found in the image. For example, if we have a polygon who meets the size and angle contraints, it might still not be a high-interest target because one of the sides is 15 times the length of another side. This polygon is likely not one of the targets we are looking for.

The code follows the next three majjor steps iteratively for each frame in the video:  

#### Image processing:
- First we read in the image in grayscale  
- Resize the frame to be as close to 640 x as possible if it is larger than 640 pixels on either the length of width.
    - This is done by finding the shortest side of the frame and finding an integervalue that scales it down to 640 pixels.  
- Next, we equalize the histogram of the resized grayscale image using equalizeHist()   
- Then, we apply a bilateral filter blur to the image to preserive edges 
- Following that, we apply a adaptive thresholind method which has two parameters blockSize and constant_subtractor
    - The two parameters are explained above. These parameters should be tuned as the same parameters will not always be the best across all videos 
- Laslty, we perform a nearest neighbor method to remove white noise from the image
    - To remove the white noise, for each pixel look at the surrounding 8 pixels (excluding the last outer edge of the image). Then, if there are less than 4 white pixels surrounding a pixel, the pixel is set to black. If there are more, then it is left untouched.   
- This is the image that we will use to find the targets (it is now ready for finding contours which are converted to polygons in the next step).  

#### Finding Contours:
- Using the processed image in the last step above, we now find high interest contours in the image
- First, find all of the contours in the image using OpenCV findContours() 
    - This gives us all conturs, of all shapes and sizes of the white blobs in the image 
    - We also restrict the search to only give us the outer most contour if a subsequent one is found inside a contour (this helps us not find a contour inside a contour 
- Then, we fit a closed polygon to each contour in the image
    - This is done using OpenCV approxPolyDP() 
- Then, once we estimate a polygon, we check for several things before accepting the polygon as a high-interest polygon using several user-defined parameters 
- The parameters are number of vertices, minArea, maxArea, maxCosine, and maxRatio of the polygon.  
- The polygons are checked first to have exactly 4 vertices, then, we check if the bounded min/max area fits the polygon along with the largest angle for any vertex in the polygon to be no larger than defined using maxCosine. We also check if the sides are largely unproportional or not, i.e. we don't want a contour that is has 4 vertices but is 15 times longer on one side than another since this is likely not a template. This is done using the maxRatio paramter. 
- If the polygon withstands these checks, then it is admitted into a high-interest region of interest we want to check tempalte matching against. This is done in the next step. 

#### Template Matching;
- In this section, the remaining polygons are checked against all of the templates sent into the program.
- The four corners of each template are warped and mapped onto the four corners of the high-interest polygon. 
    - An overlap percentage is calculated by checking each pixel. If the pixel matches we add to the score, if they do not match, we penalize from the scrore. We only compare the overlap, nothing outside the high-interest region and template. 
        - Then, the total number of pixels compared is used to divide the score to give the overall percentage of match of that template. 
        - The template is then rotated to check a different orientation of the template. All four possibilites of the template are checked on top of the polygon. The highest matching orientation is kept.
        - This process is repeated for each template sent into the program. 
        - The highest matching template is kept and assigned as the identified pattern to the polygon. 
- If there is more than 1 polygon that has the same template matched as its highest match, then the highest matched polygon is assigned the template and the other is not assigned the target. This prevents from multiple patterns to be found when there is only one to be found. 
- If there is a polygon with <70% matched, then it is not considered identified. This is because if the polygon has <70% for all of the templates, though it is a high-interest region of interest, it is not found to be a good match with any of the templates sent in. 

Once all high-interest polygons are checked and assigned a name (or not if there is more then one found), then the next frame is taken and the process is repeated. 

During the procedure above, two video files are written, a thresholded video (after image processing) and a drawing video (after template matching). 

### Changing the code
- If changes are made to the code then the code must be recompiled with the following line in Mac OS terminal. 
    - g++ main1.cpp finding_contour.cpp template_matching.cpp -lopencv_dnn -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -std=c++14 -stdlib=libc++ -o object_tracking
- This will produce an executable called object_tracking 

------------------------------------------------ 

If OpenCV is not installed, I've added instructions for installing OpenCV below. To run this file the user will need to have the library OpenCV installed. Steps for setting up the enviornment are given below. These steps are adapted from a StackOverFlow post: https://stackoverflow.com/questions/34340578/installing-c-libraries-on-os-x 

Step 1: Install an Apple compatiable IDE throgh the App Store called Xcode. 

Step 2: Install Xcode's command line tools using xcode-select --install 

Step 3: Install Homebrew to install the OpenCV library for C++ using the link: https://brew.sh

Step 4: Use Homebrew to search for OpenCV by entering -> brew search opencv <- in terminal

Step 5: Install OpenCV using -> brew install opencv <- 

Step 6: Once you have OpenCV installed, you can use Xcode to edit files, for this case, we want to open the file in Xcode and edit line 16 by entering in the name of the image we wish to process. 

Step 7: Once you edit this line, some Xcode configuration needs to be done in Xcode. This is tricky. First, click on the project on the far left side of the screen. (It shoud be called what you named the project. )

Step 8: Click -> Build Settings <- in the middle of the screen, it is a tab

Step 9: Find the section for -> Search Paths <- and edit -> Header Search Paths <- and -> Library Search Paths <- with the following text:
Header Search Path: /usr/local/Cellar/opencv3/** 
Library Search Paths: /usr/local/Cellar/opencv3/**

Step 10: In the same window, find -> Other Linker Files <- and add the text:
-lopencv_calib3d
-lopencv_core
-lopencv_features2d
-lopencv_flann
-lopencv_highgui
-lopencv_improc
-lopencv_ml
-lopencv_objdetect
-lopencv_photo
-lopencv_stitching
-lopencv_superres
-lopencv_ts
-lopencv_video
-lopencv_videostab

