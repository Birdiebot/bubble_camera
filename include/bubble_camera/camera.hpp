/*
 * @Author: Ligcox
 * @Date: 2022-01-27 06:41:01
 * @FilePath: /bubble_camera/include/bubble_camera/camera.hpp
 * @LastEditors: Ligcox
 * @LastEditTime: 2022-05-12 03:27:47
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#pragma once
#include <iostream>
#include <exception>
#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <stdlib.h>
#include <ctime>
#include <string>
#include <sys/time.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace std::chrono_literals;

#include "bubble_camera/gxapi/GxIAPI.h"
#include "bubble_camera/gxapi/DxImageProc.h"
#include "bubble_camera/gxapi/gx_utils.h"

const int VINPUT_SOURCE_UNDEFINED = 0;
const int VINPUT_SOURCE_CAMERA = 1;
const int VINPUT_SOURCE_VIDEO = 2;
const int VINPUT_SOURCE_DAHENG_CAM = 3;
const int VINPUT_SOURCE_HIKROBOT_CAM = 4;

const int IMAGE_OPEN_ERROR = 1;
const int IMAGE_READ_ERROR = 2;

enum class CamParamType
{
    Width,
    Height,
    AutoExposure,
    Exposure,
    Brightness,
    AutoWhiteBalance,
    WhiteBalance,
    Gain,
    Gamma,
    Contrast,
    Saturation,
    Hue,
    Fps
};

class vInput : public rclcpp::Node
{
public:
    explicit vInput(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    ~vInput();

private:
    rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(
        10ms, std::bind(&vInput::publishRawImage, this));

    sensor_msgs::msg::CameraInfo camera_info_msg;

    rmw_qos_profile_t imgraw_qos =
        {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false};
    bool use_image_qos = false;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImagePtr img_msg_ptr = std::make_shared<cv_bridge::CvImage>();

    // device type
    int type_;

private:
    // for opencv l4v device
    cv::VideoCapture source_;
    std::string resource_;
    int frame_count_;
    int seq_ = 0; // sequence ID: consecutively increasing ID

public:
    void publishRawImage();
    void error_process(const int &error_code);

public:
    GX_STATUS daheng_status;
    u_char *RGB_image_buf_;
    GX_DEV_HANDLE hDevice = nullptr;
    uint32_t nDeviceNum;
    PGX_FRAME_BUFFER pFrameBuffer = nullptr;
    int init_daheng();
    bool getGxapiFrame(cv::Mat &image_raw);

private:
    // image process function
    bool image_process(cv::Mat &image);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

private:
    double angle_;
    double scale_;
};
