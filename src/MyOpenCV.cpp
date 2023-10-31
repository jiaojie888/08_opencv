//
// Created by jiaojie on 2023/10/29.
//
#include <opencv2/opencv.hpp>
#include "MyOpenCV.h"

using namespace cv;

void MyOpenCV::p0() {
    std::string pic_path = "/Users/jiaojie/Pictures/thumbnail-001.jpg";
    Mat src = imread(pic_path, IMREAD_UNCHANGED);
    if (src.empty()) {
        printf("could not load image...\n");
        return;
    }
    namedWindow("input window", WINDOW_AUTOSIZE);
    imshow("input image", src);
}

void MyOpenCV::p1() {
    std::string path = "/Users/jiaojie/Pictures/eu.jpeg";
    Mat src = imread(path, IMREAD_UNCHANGED);
    if (src.empty()) {
        printf("could not load image...\n");
        return;
    }
    namedWindow("input window", WINDOW_AUTOSIZE);
    Mat gray, hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    cvtColor(src, gray, COLOR_BGR2GRAY);
    imshow("hsv", hsv);
    imshow("gray", gray);
    imwrite("/Users/jiaojie/Pictures/hsv.jpeg", hsv);
    imwrite("/Users/jiaojie/Pictures/gray.jpeg", gray);
}

void MyOpenCV::p2(Mat &m) {
    Mat m1, m2;
    m1 = m.clone();
    m.copyTo(m2);
    std::cout << m.rows << ", " << m.cols << ", " << m.channels() << std::endl;

    Mat m3 = Mat::zeros(Size(256, 256), CV_8UC3);
    m3 = Scalar(0, 0, 255);
    imshow("m3", m3);
    waitKey(0);

    Mat m4 = m3.clone();
    m4 = 127;
    imshow("m", m);
    imshow("m3", m3);
    imshow("m4", m4);
    waitKey(0);

    Mat kernel = (Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    imshow("kernel", kernel);
}


void MyOpenCV::p4(cv::Mat &image) {
    imshow("eee1", image);
    int w = image.cols;
    int h = image.rows;
    int channels = image.channels();
//    for (int row = 0; row < h; ++row) {
//        for (int col = 0; col < w; ++col) {
//            if (channels == 1) {
//                image.at<uchar>(row, col) = 255 - image.at<uchar>(row, col);
//            }
//            if (channels == 3) {
//                image.at<Vec3b>(row, col)[0] = 255 - image.at<Vec3b>(row, col)[0];
//                image.at<Vec3b>(row, col)[1] = 255 - image.at<Vec3b>(row, col)[1];
//                image.at<Vec3b>(row, col)[2] = 255 - image.at<Vec3b>(row, col)[2];
//            }
//        }
//    }
    for (int row = 0; row < h; ++row) {
        uchar *cur_row = image.ptr(row);
        for (int col = 0; col < w; ++col) {
            if (channels == 1) {
                *cur_row++ = 255 - *cur_row;
            }
            if (channels == 3) {
                *cur_row++ = 255 - *cur_row;
                *cur_row++ = 255 - *cur_row;
                *cur_row++ = 255 - *cur_row;
            }
        }
    }
    imshow("eee", image);
}

void MyOpenCV::p5(const Mat &image) {
    imshow("eee", image);
    image += Scalar(300, 300, 200);
    //multiply(image, Scalar(2,2,2), image);
    //multiply(image, Scalar(2,2,2), image);
    imshow("12", image);
}

static void on_track(int b, void *userdata) {
    Mat &image = *(Mat *) userdata;
    Mat m = Mat::zeros(image.size(), image.type());
    Mat dst = Mat::zeros(image.size(), image.type());
//    addWeighted(image, 1.0, m, 0, b, dst);
    imshow("adjust", dst);
}

static void on_track2(int b, void *userdata) {
    Mat &image = *(Mat *) userdata;
    Mat m = Mat::zeros(image.size(), image.type());
    Mat dst = Mat::zeros(image.size(), image.type());
    double c = (double) b / 100;
    addWeighted(image, c, m, 0.0, 0, dst);
    imshow("adjust", dst);
}

void MyOpenCV::p6(const Mat &image) {
    imshow("orig", image);
    namedWindow("adjust", WINDOW_AUTOSIZE);
    int lightness = 50;
    int contrast = 100;
    createTrackbar("lightness bar", "adjust", &lightness, 100, on_track, (void *) &image);
    createTrackbar("Contrast bar", "adjust", &contrast, 200, on_track2, (void *) &image);
    on_track(50, (void *) &image);
}

void MyOpenCV::p8(const Mat &image) {
    Mat dst = Mat::zeros(image.size(), image.type());
    imshow("111", image);
    while (true) {
        imshow("dst", dst);
        int i = waitKey(10);
        if (i == 49) {
            cvtColor(image, dst, COLOR_BGR2GRAY);
        }
        if (i == 50) {
            cvtColor(image, dst, COLOR_BGR2HSV);
        }
        if (i == 51) {
            add(image, Scalar(50, 50, 50), dst);
        }
        if (i == 27) {
            break;
        }
    }
}

void MyOpenCV::p9(const Mat &image) {
    Mat dst = Mat::zeros(image.size(), image.type());
    imshow("111", image);
    int colorMap[] = {
            COLORMAP_AUTUMN,
            COLORMAP_BONE,
            COLORMAP_JET,
            COLORMAP_WINTER,
            COLORMAP_RAINBOW,
            COLORMAP_OCEAN,
            COLORMAP_SUMMER,
            COLORMAP_SPRING,
            COLORMAP_COOL,
            COLORMAP_HSV,
            COLORMAP_PINK,
            COLORMAP_HOT,
            COLORMAP_PARULA,
            COLORMAP_MAGMA,
            COLORMAP_INFERNO,
            COLORMAP_PLASMA,
            COLORMAP_VIRIDIS,
            COLORMAP_CIVIDIS,
            COLORMAP_TWILIGHT,
            COLORMAP_TWILIGHT_SHIFTED,
            COLORMAP_TURBO,
            COLORMAP_DEEPGREEN
    };
    int len = sizeof colorMap / sizeof colorMap[0];
    std::cout << len << std::endl;
    int index = 0;
    while (true) {
        imshow("dst", dst);
        int i = waitKey(1000);
        if (i == 27) {
            break;
        }
        applyColorMap(image, dst, colorMap[index % len]);
        index++;
    }
}

void MyOpenCV::p10(const Mat &image) {
    Mat m1 = Mat::zeros(Size(300, 300), CV_8UC3);
    Mat m2 = Mat::zeros(Size(300, 300), CV_8UC3);
    Mat m3 = Mat::zeros(Size(300, 300), CV_8UC3);
    circle(m1, Point(100, 100), 100, Scalar(255, 0, 255), -1, LINE_AA, 0);
    circle(m2, Point(200, 100), 100, Scalar(255, 255, 0), -1, LINE_AA, 0);
    circle(m3, Point(150, 200), 100, Scalar(0, 255, 255), -1, LINE_AA, 0);
    imshow("m1", m1);
    imshow("m2", m2);
    imshow("m3", m3);
    Mat dst = ~image;
    bitwise_xor(m1, m2, dst);
    bitwise_xor(m3, dst, dst);
    imshow("m4", dst);
}

void MyOpenCV::p11(const Mat &image) {
    imshow("image", image);
    int cols = image.cols;
    int rows = image.rows;
    int channels = image.channels();
    Mat b = Mat::zeros(image.size(), CV_8UC1);
    Mat g = Mat::zeros(image.size(), CV_8UC1);
    Mat r = Mat::zeros(image.size(), CV_8UC1);
//    std::cout << b << std::endl;
    for (int row = 0; row < rows; ++row) {
        const uchar *cur_row = image.ptr(row);
        uchar *b_row = b.ptr<uchar>(row);
        uchar *g_row = g.ptr<uchar>(row);
        uchar *r_row = r.ptr<uchar>(row);
        for (int col = 0; col < cols; ++col) {
            if (channels == 3) {
                *b_row++ = *cur_row++;
                *g_row++ = *cur_row++;
                *r_row++ = *cur_row++;
            }
        }
    }
    imshow("b", b);
    imshow("g", g);
    imshow("r", r);

    std::vector<Mat> v;

//    split(image, v);
//    imshow("0", v[0]);
//    imshow("1", v[1]);
//    imshow("2", v[2]);

    Mat dst = Mat::zeros(image.size(), CV_8UC3);
//    merge(std::vector<Mat>{b, r, g}, dst);
    int from_to[] = {0, 1, 1, 2, 2, 0};
    mixChannels(&image, 1, &dst, 1, from_to, 3);
    imshow("dst", dst);
}

static Mat mask;

void onTrack(int b, void *userdata) {
    Mat &image = *(Mat *) userdata;

    Mat hsv = Mat::zeros(image.size(), image.type());
    cvtColor(image, hsv, COLOR_BGR2HSV);
    imshow("hsv", hsv);

    inRange(hsv, Scalar(50, 45, b), Scalar(90, 255, 255), mask);
    imshow("mask", mask);
}

void MyOpenCV::p12(const Mat &image) {
    imshow("image", image);
    namedWindow("mask", WINDOW_AUTOSIZE);

    int x = 40;
    createTrackbar("green", "mask", &x, 255, onTrack, (void *) &image);
    onTrack(x, (void *) &image);

    Mat dst = Mat::zeros(image.size(), image.type());
    dst = Scalar(0, 0, 255);
    image.copyTo(dst, ~mask);
    imshow("dst", dst);

    std::vector<Mat> vMat;
    split(image, vMat);

    double min, max;
    Point minP, maxP;
    minMaxLoc(vMat[0], &min, &max, &minP, &maxP);
    Mat mean, stddev;
    meanStdDev(vMat[0], mean, stddev);
    std::cout << min << std::endl;
    std::cout << max << std::endl;
    std::cout << minP << std::endl;
    std::cout << maxP << std::endl;
    std::cout << mean << std::endl;
    std::cout << stddev << std::endl;
}

void MyOpenCV::p14(const Mat &image) {
    imshow("image", image);
    Mat bg = Mat::zeros(image.size(), image.type());
    bg = Scalar(255, 255, 255);
    Rect rect(230, 140, 350, 160);
    rectangle(bg, rect, Scalar(0, 0, 255), -1, LINE_AA, 0);
    imshow("image2", image);

    circle(bg, Point(300, 345), 50, Scalar(0, 255, 0));
    imshow("image3", image);
    imshow("b", bg);

    bitwise_and(image, bg, bg);
    imshow("bg", bg);

    line(bg, Point(), Point(image.cols, image.rows), Scalar(233, 233, 44));
    imshow("bg2", bg);

    Mat rand = Mat::zeros(Size(500, 500), CV_8UC3);
    RNG rng(233);
    while (true) {
        int x = rng.uniform(0, rand.cols);
        int y = rng.uniform(0, rand.rows);
        int x1 = rng.uniform(0, rand.cols);
        int y1 = rng.uniform(0, rand.rows);
        rand = Scalar(0, 0, 0);
        line(rand,
             Point(x, y),
             Point(x1, y1),
             Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
        imshow("rand", rand);
        int i = waitKey(100);
        if (i == 27) {
            break;
        }
    }
}

static Point sp(-1, -1);
static Point ep(-1, -1);
static Mat temp;

static void on_draw(int event, int x, int y, int flag, void *userdata) {
    Mat &mat = *(Mat *) userdata;
    if (event == EVENT_LBUTTONDOWN) {
        std::cout << x << ", " << y << std::endl;
        sp.x = x;
        sp.y = y;
    } else if (event == EVENT_LBUTTONUP) {
        std::cout << x << ", " << y << std::endl;
        ep.x = x;
        ep.y = y;
        int dx = ep.x - sp.x;
        int dy = ep.y - sp.y;
        if (dx > 0 && dy > 0) {
            rectangle(mat, Rect(sp.x, sp.y, dx, dy), Scalar());
            temp.copyTo(mat);
            imshow("ext", mat(Rect(sp.x, sp.y, dx, dy)));
            sp.x = -1;
            sp.y = -1;
        }
    } else if (event == EVENT_MOUSEMOVE) {
        if (sp.x > 0 && sp.y > 0) {
            ep.x = x;
            ep.y = y;
            int dx = ep.x - sp.x;
            int dy = ep.y - sp.y;
            if (dx > 0 && dy > 0) {
                temp.copyTo(mat);
                rectangle(mat, Rect(sp.x, sp.y, dx, dy), Scalar(255, 255, 255));
            }
        }
    }
    imshow("mouse", mat);
}

void MyOpenCV::p15(const Mat &image) {
//    imshow("image", image);
//    std::vector<Point> vp = {Point(10, 10), Point(30, 30), Point(20, 300), Point(40, 66)};
//    std::vector<std::vector<Point>> v;
//    v.emplace_back(vp);
//    drawContours(image, v, 0, Scalar(255, 0, 0), -1, LINE_AA);
//    imshow("ddd", image);

    Mat mat = Mat::zeros(Size(500, 500), CV_8UC3);
    mat = Scalar(255, 255, 255);
    imshow("mouse", mat);

    namedWindow("mouse", WINDOW_AUTOSIZE);
    setMouseCallback("mouse", on_draw, (void *) &image);
    imshow("mouse", image);
    temp = image.clone();
}

void MyOpenCV::p18(const Mat &image) {
    Mat d;
    image.convertTo(d, CV_32F);
    imshow("d", d);
    Mat dst = Mat::zeros(image.size(), image.type());
    normalize(d, dst, 1.0, 0, NORM_MINMAX);
    imshow("dst", dst);
}

void MyOpenCV::p19(const Mat &image) {
    imshow("image", image);
    Mat zoomIn, zoomOut;
    int w = image.cols;
    int h = image.rows;
    resize(image, zoomIn, Size(w / 2, h / 2), 0, 0, INTER_LINEAR);
    resize(image, zoomOut, Size(w * 2, h * 2), 0, 0, INTER_LINEAR);
//    imshow("in", zoomIn);
//    imshow("out", zoomOut);

    flip(image, image, 0);
    imshow("fimage", image);

    Mat M, dst;
    M = getRotationMatrix2D(Point2f(w / 2, h / 2), 45, 1);
    double cos = abs(M.at<double>(0, 0));
    double sin = abs(M.at<double>(0, 1));
    int nw = cos * w + sin * h;
    int nh = sin * w + cos * h;
    M.at<double>(0, 2) += (nw - w) / 2;
    M.at<double>(1, 2) += (nh - h) / 2;
    warpAffine(image, dst, M, Size(nw, nh), INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 0));
    imshow("dst", dst);
}

VideoCapture cap("/Users/jiaojie/Movies/output.mp4");
//VideoCapture cap(0);
VideoWriter videoWriter("/Users/jiaojie/Movies/output1.mp4",
                        cap.get(CAP_PROP_FOURCC), 30,
                        Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT)));

void on_delay(int delay, void *userdata) {
    Mat &frame = *(Mat *) userdata;
    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    double count = cap.get(CAP_PROP_FRAME_COUNT);
    double fps = cap.get(CAP_PROP_FPS);
    std::cout << "width = " << width << std::endl;
    std::cout << "height = " << height << std::endl;
    std::cout << "count = " << count << std::endl;
    std::cout << "fps = " << fps << std::endl;
    while (true) {
        cap.read(frame);
        if (frame.empty()) {
            break;
        }
        flip(frame, frame, 1);
        cvtColor(frame, frame, COLOR_BGR2HSV);
        imshow("frame", frame);
        videoWriter.write(frame);
        int c = waitKey(delay);
        if (c == 27) {
            break;
        }
    }
    cap.release();
    videoWriter.release();
}

void MyOpenCV::p20(const Mat &image) {
    namedWindow("frame", WINDOW_AUTOSIZE);
    Mat frame;
    int delay = 20;
    createTrackbar("delay time(ms)", "frame", &delay, 1000, on_delay, &frame);
    on_delay(delay, &frame);
}

void MyOpenCV::p25(const Mat &image) {
    imshow("image", image);
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    Mat dst;
    equalizeHist(gray, dst);
//    imshow("dst", dst);

    Mat blur1;
    blur(image, blur1, Size(11, 11));
//    imshow("blur1", blur1);

    Mat gauss;
    GaussianBlur(image, gauss, Size(11, 11), 15);
//    imshow("gauss", gauss);

    Mat gaussdouble;
    bilateralFilter(image, gaussdouble, 0, 100, 10);
    imshow("gaussdouble", gaussdouble);
}


void MyOpenCV::test() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return;
    }

    Mat frame;
    while (true) {
        cap >> frame;
        imshow("Camera", frame);
        if (waitKey(1) == 'q') {
            break;
        }
    }

    cap.release();
    destroyAllWindows();

    std::cout << "Hello, World!" << std::endl;
    std::string image_path = "/Users/jiaojie/Pictures/pic/Snipaste_2023-06-06_09-41-10.png";
    Mat img = imread(image_path, IMREAD_COLOR);

    imshow("Display window", img);
}


void MyOpenCV::addMasaike() {

    cv::VideoCapture cap("/Users/jiaojie/Movies/output.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file." << std::endl;
        return;
    }

    cv::CascadeClassifier faceCascade;
    if (!faceCascade.load(
            "/opt/homebrew/Cellar/opencv/4.8.1_1/share/opencv4/haarcascades/haarcascade_eye_tree_eyeglasses.xml")) {
        std::cerr << "Error: Could not load face cascade." << std::endl;
        return;
    }

    cv::Mat frame;
    while (cap.read(frame)) {
        std::vector<cv::Rect> faces;
        faceCascade.detectMultiScale(frame, faces, 1.1, 3, 0, cv::Size(30, 30));

        for (const cv::Rect &face: faces) {
            // 在人脸区域应用马赛克效果
            cv::Mat faceRegion = frame(face);
            cv::Mat mosaic;
            cv::resize(faceRegion, mosaic, cv::Size(faceRegion.cols / 10, faceRegion.rows / 10), 0, 0,
                       cv::INTER_NEAREST);
            cv::resize(mosaic, faceRegion, faceRegion.size(), 0, 0, cv::INTER_NEAREST);
        }

        // 在这里可以显示或保存处理后的帧
        // 显示处理后的帧
        cv::imshow("Video", frame);

        // 处理窗口事件
        int key = cv::waitKey(100);
        if (key == 27) {
            break;  // 退出循环
        }
    }
    cap.release();
}