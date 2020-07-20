const cv::Scalar colorData[] =
{cv::Scalar(  0,   0, 255), cv::Scalar(255,   0,   0), cv::Scalar(255,   0, 255),
 cv::Scalar(  0, 255,   0), cv::Scalar(  0, 255, 255), cv::Scalar(255, 255,   0),
 cv::Scalar(255, 255, 255)};

struct pointHistory {
    cv::Point2f p;
    int ttl;
};

struct areaRectangle {
        cv::Point2f min;
        cv::Point2f max;
};

struct areaBox {
        cv::Point2f p0, p1, p2, p3;
};

const float PI = 3.14159;

