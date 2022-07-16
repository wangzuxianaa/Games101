#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 6) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }   
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // 递归完成贝塞尔曲线的绘制
    if(control_points.size() == 1)
        return control_points[0];
    
    std::vector<cv::Point2f> points;
    for(int i = 0; i < control_points.size() - 1; i++) {
        float pointx = t*control_points[i].x + (1 - t)*control_points[i+1].x;
        float pointy = t*control_points[i].y + (1 - t)*control_points[i+1].y;
        cv::Point2f point{pointx, pointy};
        points.push_back(point);
    }
    return recursive_bezier(points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t = 0.0; t < 1.0; t += 0.001)
    {
        // 递归得到需要画的点
        cv::Point2f point = recursive_bezier(control_points, t);
        float x_floor = floor(point.x);
        float x_ceil = ceil(point.x);
        float y_floor = floor(point.y);
        float y_ceil = ceil(point.y);
        // 遍历对应点身旁的九个像素
        for(int i = x_floor - 1; i <= x_ceil; i++) {
            for(int j = y_floor - 1; j <= y_ceil; j++) {
                // else if(i == x_floor && j == y_floor) {
                //     window.at<cv::Vec3b>(j + 0.5, i + 0.5)[1] =  255;
                // }
                // else {
                //     double dis2 = sqrt(pow(i + 0.5 - point.x, 2) + pow(j + 0.5 - point.y, 2));

                //     window.at<cv::Vec3b>(j + 0.5, i + 0.5)[1] =  255;
                // }
                // 所要绘制点和像素点的距离
                double dis1 = pow(i + 0.5 - point.x, 2) + pow(j + 0.5 - point.y, 2);
                // 绘制 公式为255*(1 - 根号2 / 3 * dis1)进行填充
                window.at<cv::Vec3b>(j + 0.5, i + 0.5)[1] =  std::fmax(255 * (1 - sqrt(2) / (float)3 * dis1), window.at<cv::Vec3b>(j + 0.5, i + 0.5)[1]);
            }
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 6) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
