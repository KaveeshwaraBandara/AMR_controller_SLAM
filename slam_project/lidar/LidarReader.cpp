#include "LidarReader.hpp"
#include <iostream>
#include <cmath>

using namespace sl;

LidarReader::LidarReader(const char* port, unsigned int baudrate)
    : port_(port), baudrate_(baudrate), drv_(nullptr), channel_(nullptr) {}

LidarReader::~LidarReader() {
    stop();
    if (drv_) {
        delete drv_;
        drv_ = nullptr;
    }
}

bool LidarReader::connect() {
    drv_ = *createLidarDriver();
    if (!drv_) return false;

    channel_ = *createSerialPortChannel(port_, baudrate_);
    if (SL_IS_FAIL(drv_->connect(channel_))) return false;

    sl_lidar_response_device_info_t devinfo;
    if (SL_IS_FAIL(drv_->getDeviceInfo(devinfo))) return false;

    drv_->setMotorSpeed();
    drv_->startScan(0, 1);
    return true;
}

std::vector<std::pair<float, float>> LidarReader::getScan() {
    std::vector<std::pair<float, float>> scan;
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);
  sl_result res = drv_->grabScanDataHq(nodes, count);
  std::cout << "grabScanDataHq result: " << res << ", count: " << count << std::endl;

    if (SL_IS_OK(drv_->grabScanDataHq(nodes, count))) {
        drv_->ascendScanData(nodes, count);
        for (size_t i = 0; i < count; ++i) {
            float angle = (nodes[i].angle_z_q14 * 90.f) / 16384.f;
            float dist = nodes[i].dist_mm_q2 / 4000.0f;
            if (nodes[i].dist_mm_q2 != 0)
            if(angle <=90 || angle>=270){
                scan.emplace_back(angle, dist);
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[i].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    angle,
                    dist,
                    nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        }
        }
    }
    return scan;
}

void LidarReader::stop() {
    if (drv_) {
        drv_->stop();
        drv_->setMotorSpeed(0);
    }
}

