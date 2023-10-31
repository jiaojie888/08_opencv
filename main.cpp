#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "./src/MyOpenCV.h"

using namespace cv;

std::string name[] = {"/Users/jiaojie/Pictures/",
                      "data/eu.jpeg",
                      "data/smarties.png",
                      "data/squirrel_cls.jpg",
                      "pic/huzhu.jpg",
                      "pic/970383650bb7557adce4f5c282161dc8.jpeg",
                      "pic/7473ec1ab252a77f0e9eb93adaedc20d.jpeg",
                      "data/lena_tmpl.jpg"
};

int main() {
    Mat image = imread(name[0] + name[7]);
    std::unique_ptr<MyOpenCV> myOpenCvPtr = std::make_unique<MyOpenCV>();
//    myOpenCvPtr->p0();
//    myOpenCvPtr->p1();
//    myOpenCvPtr->p2(m);
//    myOpenCvPtr->p4(image);
//    myOpenCvPtr->p5(image);
//    myOpenCvPtr->p6(image);
//    myOpenCvPtr->p8(image);
//    myOpenCvPtr->p9(image);
//    myOpenCvPtr->p10(image);
//    myOpenCvPtr->p11(image);
//    myOpenCvPtr->p12(image);
//    myOpenCvPtr->p14(image);
//    myOpenCvPtr->p15(image);
//    myOpenCvPtr->p18(image);
//    myOpenCvPtr->p19(image);
//    myOpenCvPtr->p20(image);
    myOpenCvPtr->p25(image);

    waitKey(0);
    destroyAllWindows();
}