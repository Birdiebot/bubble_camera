/*
 * @Author: Ligcox
 * @Date: 2022-01-27 06:41:01
 * @FilePath: /bubble/src/bubble_camera/include/bubble_camera/camera.hpp
 * @LastEditors: HarryWen
 * @LastEditTime: 2022-07-08 12:14:59
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

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace std::chrono_literals;

#define MAX_IMAGE_DATA_SIZE (3 * 1280 * 1024)

#include "bubble_camera/mvs/MvCameraControl.h"

const int VINPUT_SOURCE_UNDEFINED = 0;
const int VINPUT_SOURCE_CAMERA = 1;
const int VINPUT_SOURCE_VIDEO = 2;
const int VINPUT_SOURCE_DAHENG_CAM = 3;
const int VINPUT_SOURCE_HIKROBOT_CAM = 4;

const int IMAGE_OPEN_ERROR = 1;
const int IMAGE_READ_ERROR = 2;

class vInput : public rclcpp::Node
{
public:
    explicit vInput(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    ~vInput();

    void publishRawImage();
    void error_process(const int &error_code);

public:
    int init_hikrobot();
    bool getHikrobotFrame(cv::Mat &image_raw);
    void setHikrobotExposureTime(int exposure_time);
    void setHikrobotGainAuto(int gain_auto);
    bool printDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
    void *handle = nullptr;
    unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
    unsigned char *m_pBufForOutImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

private:
    double angle_;
    double scale_;
    int exposure_time_;
    int gain_auto_;

    // device type
    int type_;
    // for opencv l4v device
    cv::VideoCapture source_;
    std::string resource_;
    int frame_count_;
    int seq_ = 0; // sequence ID: consecutively increasing ID

    rclcpp::TimerBase::SharedPtr timer_;

    bool use_image_qos = false;
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
    image_transport::Publisher image_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr img_msg_ptr = std::make_shared<cv_bridge::CvImage>();
    // image process function
    bool image_process(cv::Mat &image);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};
