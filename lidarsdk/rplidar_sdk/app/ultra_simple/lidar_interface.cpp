#include "lidar_interface.h"
#include <iostream>
#include <signal.h>
#include <string.h>
#include <stdlib.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x) ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms >= 1000) { usleep(1000 * 1000); ms -= 1000; }
    if (ms != 0) usleep(ms * 1000);
}
#endif

using namespace sl;

static ILidarDriver* drv = nullptr;
static IChannel* _channel = nullptr;
static int opt_channel_type = CHANNEL_TYPE_SERIALPORT;
static bool ctrl_c_pressed = false;

static void ctrlc(int) {
    ctrl_c_pressed = true;
}

bool checkHealth() {
    sl_lidar_response_device_health_t health;
    sl_result result = drv->getHealth(health);
    if (SL_IS_OK(result)) {
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            std::cerr << "LIDAR internal error.\n";
            return false;
        }
        return true;
    }
    std::cerr << "Failed to get LIDAR health.\n";
    return false;
}

bool slamtec::initializeLidar(int argc, const char* argv[]) {
    const char* opt_channel_param_first = nullptr;
    sl_u32 opt_channel_param_second = 0;
    sl_u32 baudrateArray[2] = {115200, 256000};
    bool useArgcBaudrate = false;

    if (argc < 4) {
        std::cerr << "Usage: <program> --channel --serial <port> [baudrate]\n";
        return false;
    }

    if (strcmp(argv[1], "--channel") != 0) return false;

    if (strcmp(argv[2], "--serial") == 0) {
        opt_channel_param_first = argv[3];
        if (argc > 4) {
            opt_channel_param_second = strtoul(argv[4], NULL, 10);
            useArgcBaudrate = true;
        }
        opt_channel_type = CHANNEL_TYPE_SERIALPORT;
    } else {
        std::cerr << "Unsupported channel type.\n";
        return false;
    }

    if (!opt_channel_param_first) {
#ifdef _WIN32
        opt_channel_param_first = "\\\\.\\com3";
#else
        opt_channel_param_first = "/dev/ttyUSB0";
#endif
    }

    drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "Failed to create driver.\n";
        return false;
    }

    if (useArgcBaudrate) {
        _channel = *createSerialPortChannel(opt_channel_param_first, opt_channel_param_second);
        if (!SL_IS_OK(drv->connect(_channel))) return false;
    } else {
        for (sl_u32 baud : baudrateArray) {
            _channel = *createSerialPortChannel(opt_channel_param_first, baud);
            if (SL_IS_OK(drv->connect(_channel))) break;
        }
    }

    sl_lidar_response_device_info_t devinfo;
    if (!SL_IS_OK(drv->getDeviceInfo(devinfo))) {
        std::cerr << "Failed to get device info.\n";
        return false;
    }

    std::cout << "Connected to SLAMTEC LIDAR:\n";
    for (int i = 0; i < 16; ++i) printf("%02X", devinfo.serialnum[i]);
    std::cout << "\nFirmware: " << (devinfo.firmware_version >> 8) << "." << (devinfo.firmware_version & 0xFF)
              << ", Hardware: " << int(devinfo.hardware_version) << "\n";

    if (!checkHealth()) return false;

    signal(SIGINT, ctrlc);
    drv->setMotorSpeed();
    drv->startScan(0, 1);
    return true;
}

void slamtec::readLidarData() {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result result = drv->grabScanDataHq(nodes, count);

    if (SL_IS_OK(result)) {
        drv->ascendScanData(nodes, count);
        for (size_t i = 0; i < count; ++i) {
            printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                   (nodes[i].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ",
                   (nodes[i].angle_z_q14 * 90.f) / 16384.f,
                   nodes[i].dist_mm_q2 / 4.0f,
                   nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        }
    }

    if (ctrl_c_pressed) {
        slamtec::cleanupLidar();
        exit(0);
    }
}

void slamtec::cleanupLidar() {
    if (drv) {
        drv->stop();
        delay(200);
        drv->setMotorSpeed(0);
        delete drv;
        drv = nullptr;
    }
}

