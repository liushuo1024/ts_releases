#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>

using namespace Eigen;
using namespace std;
namespace zcnest
{
    namespace control
    {
        class OMVController
        {
        public:
            OMVController(double L_passive_base, double L_passive_steer, double L_passive_tread);
            ~OMVController();
            int cal_theta(Vector2d start_p, Vector2d end_p, double &theta);
            double regular_theta(double theta);
            double rotation_90_clockwise(double theta);
            int parse_theta(double theta, int &num, double &offset);
            double regular_passive_theta(double theta, double &last_theta);

            void ComputeCtrlCmd(int mode, const double R, const double v, Vector4d &cmd);

            void ComputeCtrlCmdTwist(const double v, const double omega, Vector4d &cmd);
            
        private:
            Vector2d left_wheel_p;
            Vector2d right_wheel_p;
            Vector2d steering_wheel_p;
            int cal_R(double rad, double L_base, double &R);

            double last_left_wheel_theta_;
            double last_right_wheel_theta_;
            double last_steering_theta_;
            double last_steering_speed_;
            double L_base;

            double k_thera_;
            double k_pose_;
            double k_soft_;
        };
    } // namespace control
} // namespace zcnest