//#include <opencv2/opencv.hpp>
//#include <string>
//#include <iomanip>
//#include <sstream>
//
//using namespace cv;
//using namespace std;
//
//int main()
//{
//    string folder = "frames/";
//    string suffix = ".jpg";
//    int counter = 0;
//
//    Mat myImage;
//
//    while (1)
//    {
//        stringstream ss;
//        ss <<  setw(4) <<  setfill('0') << counter; // 0000, 0001, 0002, etc...
//        string number = ss.str();
//
//        string name = folder + number + suffix;
//        myImage =  imread(name);
//
//        imshow("hello" ,myImage);
//        waitKey(0);
//
//
//
//
//        counter++;
//    }
//    return 0;
//}
