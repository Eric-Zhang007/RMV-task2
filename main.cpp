#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "ceres/ceres.h"
#include "glog/logging.h"

struct RawTrajectoryPoint {
    double t;
    cv::Point pos;
};

struct TrajectoryResidual {
    TrajectoryResidual(double delta_t, double x_obs, double y_obs, double x0, double y0)
        : delta_t_(delta_t), x_obs_(x_obs), y_obs_(y_obs), x0_(x0), y0_(y0) {}

    template <typename T>
    bool operator()(const T* const params, T* residual) const {
        const T vx0 = params[0];
        const T vy0 = params[1];
        const T g   = params[2];
        const T k   = params[3];

        const T k_safe = ceres::abs(k) < T(1e-6) ? T(1e-6) : k;

        T dt = T(delta_t_);
        T one_minus_exp = T(1.0) - ceres::exp(-k_safe * dt);

        T x_predicted = T(x0_) + (vx0 / k_safe) * one_minus_exp;
        T y_predicted = T(y0_) + (vy0 / k_safe + g / (k_safe * k_safe)) * one_minus_exp - (g / k_safe) * dt;

        residual[0] = T(x_obs_) - x_predicted;
        residual[1] = T(y_obs_) - y_predicted;

        return true;
    }

private:
    const double delta_t_;
    const double x_obs_;
    const double y_obs_;
    const double x0_;
    const double y0_;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    std::cout << "--- Part 1: Extracting trajectory from video ---" << std::endl;

    std::string video_path = "video.mp4";
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened()) {
        std::cerr << "错误: 无法打开视频文件 '" << video_path << "'" << std::endl;
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps == 0) {
        std::cerr << "警告: 无法获取视频FPS, 使用默认值 60。" << std::endl;
        fps = 60.0;
    }
    double dt = 1.0 / fps;
    int frame_count = 0;
    std::vector<RawTrajectoryPoint> raw_data;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        cv::Scalar lower_color(90, 50, 50);
        cv::Scalar upper_color(130, 255, 255);
        cv::inRange(hsv, lower_color, upper_color, mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            auto max_contour = *std::max_element(contours.begin(), contours.end(),
                [](const auto& a, const auto& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            cv::Point2f center_f;
            float radius;
            cv::minEnclosingCircle(max_contour, center_f, radius);

            if (radius > 5) {
                cv::Moments M = cv::moments(max_contour);
                if (M.m00 > 0) {
                    cv::Point center(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
                    raw_data.push_back({frame_count * dt, center});
                }
            }
        }
        frame_count++;
    }
    cap.release();
    std::cout << "视频处理完成. 提取到 " << raw_data.size() << " 个数据点." << std::endl;

    if (raw_data.size() < 5) {
        std::cerr << "错误: 提取的数据点过少，无法进行拟合。" << std::endl;
        return -1;
    }
    
    std::cout << "\n--- Part 2: Fitting parameters with Ceres Solver ---" << std::endl;

    const double t0 = raw_data[0].t;
    const double x0 = raw_data[0].pos.x;
    const double y0 = raw_data[0].pos.y;

    double params[4] = {100.0, 100.0, 500.0, 0.1};

    ceres::Problem problem;

    for (const auto& point : raw_data) {
        double delta_t = point.t - t0;
        double x_obs = point.pos.x;
        double y_obs = point.pos.y;
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<TrajectoryResidual, 2, 4>( 
                new TrajectoryResidual(delta_t, x_obs, y_obs, x0, y0)
            );
        problem.AddResidualBlock(cost_function, nullptr, params);
    }

    problem.SetParameterLowerBound(params, 2, 100.0);
    problem.SetParameterUpperBound(params, 2, 1000.0);
    problem.SetParameterLowerBound(params, 3, 0.01);
    problem.SetParameterUpperBound(params, 3, 1.0);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);


    std::cout << "\n--- Part 3: Results ---" << std::endl;
    std::cout << summary.BriefReport() << std::endl;
    
    std::cout << "\n拟合完成!" << std::endl;
    std::cout << "初始位置 (x0, y0): (" << x0 << ", " << y0 << ")" << std::endl;
    std::cout << "最终拟合参数:" << std::endl;
    std::cout << "  - 初始水平速度 (vx0): " << params[0] << " px/s" << std::endl;
    std::cout << "  - 初始垂直速度 (vy0): " << params[1] << " px/s" << std::endl;
    std::cout << "  - 重力加速度 (g):   " << params[2] << " px/s^2" << std::endl;
    std::cout << "  - 空气阻力系数 (k): " << params[3] << " 1/s" << std::endl;

    return 0;
}
