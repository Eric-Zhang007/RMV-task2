#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include <cmath>

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

    std::string video_path = "video.mp4";
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened()) {
        std::cerr << "Fail to open Video '" << video_path << "'" << std::endl;
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps == 0) {
        std::cerr << "Waring: Fail to get fps, using 60 by default." << std::endl;
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

        cv::Mat hsv, mask, processed;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        cv::Scalar lower_color(90, 50, 50);
        cv::Scalar upper_color(130, 255, 255);
        cv::inRange(hsv, lower_color, upper_color, mask);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, processed, cv::MORPH_OPEN, kernel);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(processed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            auto max_contour = *std::max_element(contours.begin(), contours.end(),
                [](const auto& a, const auto& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            cv::Point2f center_f;
            float radius;
            cv::minEnclosingCircle(max_contour, center_f, radius);

            if (radius > 3) {
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
    std::cout << "Video processed, getting " << raw_data.size() << " points." << std::endl;

    if (raw_data.size() < 5) {
        std::cerr << "Error: too few points." << std::endl;
        return -1;
    }

    const double t0 = raw_data[0].t;
    const double x0 = raw_data[0].pos.x;
    const double y0 = 720.0-raw_data[0].pos.y;

    double params[4] = { 250.0, 350.0, 500.0, 0.1 };
    
    ceres::Problem problem;
    
    for (const auto& point : raw_data) {
        double delta_t = point.t - t0;
        double x_obs = point.pos.x;
        double y_obs = 720.0-point.pos.y;
        
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<TrajectoryResidual, 2, 4>( 
                new TrajectoryResidual(delta_t, x_obs, y_obs, x0, y0)
            );
        ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
        problem.AddResidualBlock(cost_function, loss, params);
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

    const double PI = 3.141592653589793;
    std::cout << summary.BriefReport() << std::endl;
    
    std::cout << "\nFitting finished!" << std::endl;
    std::cout << "Initial position (x0, y0): (" << x0 << ", " << y0 << ")" <<"(Using buttom left point as (0, 0))"<< std::endl;
    std::cout << "Final results:" << std::endl;
    std::cout << "  - Initial horizontal spped (vx0): " << params[0] << " px/s" << std::endl;
    std::cout << "  - Initial vertical speed (vy0): " << params[1] << " px/s" << std::endl;
    std::cout << "  - Initial speed (v0): "<< std::sqrt(std::pow(params[0], 2.0) + std::pow(params[1], 2.0)) << " px/s"<< std::endl;
    std::cout << "  - Initial speed angle: "<<std::atan(params[1]/params[0])/PI*180<<" deg"<<std::endl;
    std::cout << "  - Gravitational acceleration (g):   " << params[2] << " px/s^2" << std::endl;
    std::cout << "  - Coefficient of air resistance (k): " << params[3] << " 1/s" << std::endl;
    

    return 0;
}
