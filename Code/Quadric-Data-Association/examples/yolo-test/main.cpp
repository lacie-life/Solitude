#include <random>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "gtsam_quadrics/models/yolo.h"

using namespace std;

int main(void) {

    std::cout << "Hello world !!!" << std::endl;
    // Read the image file as
    // imread("default.jpg");
    cv::Mat image = cv::imread("/home/lacie/Github/My-Researches/Code/Quadric-Data-Association/data/test.jpg",
                           cv::IMREAD_ANYCOLOR);

    Yolo model = Yolo("/home/lacie/Github/My-Researches/Code/Quadric-Data-Association/models/yolov5s.torchscript.pt",
                      "/home/lacie/Github/My-Researches/Code/Quadric-Data-Association/models/coco.names");

    // Error Handling
    if (image.empty()) {
        cout << "Image File "
             << "Not Found" << endl;
        // wait for any key press
        cin.get();
        return -1;
    }

    cv::Mat result = model.drawObject(image);

    // Show Image inside a window with
    // the name provided
    cv::imshow("Window Name", result);

    // Wait for any keystroke
    cv::waitKey(0);

    return 1;
}