#include <depthai/depthai.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

// Print a matrix returned by DepthAI, one row per line.
void print_matrix(const std::vector<std::vector<float>>& matrix)
{
    for(const auto& row : matrix) {
        for(float value : row) {
            std::cout << std::setw(14) << value << ' ';
        }
        std::cout << '\n';
    }
}

// Read raw IMU measurements for a short, stationary bench test.
void test_imu(dai::Device& device)
{
    dai::Pipeline pipeline;

    auto imu = pipeline.create<dai::node::IMU>();
    auto output = pipeline.create<dai::node::XLinkOut>();

    output->setStreamName("imu");

    // Requested rates; actual supported rates may differ.
    // Raw acceleration includes gravity.
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER, 100);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 100);
    imu->enableFirmwareUpdate(false);

    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    imu->out.link(output->input);

    device.startPipeline(pipeline);

    // Keep buffering bounded; discard older messages if the host falls behind.
    auto queue = device.getOutputQueue("imu", 8, false);

    std::cout << "\nKeep the camera stationary."
              << "\nReading IMU for 20 seconds...\n";
    std::cout << std::fixed << std::setprecision(4);

    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds(20);
    auto next_print = std::chrono::steady_clock::now();

    bool received = false;

    while(std::chrono::steady_clock::now() < end) {
        auto data = queue->tryGet<dai::IMUData>();

        if(!data || data->packets.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        received = true;

        // Display only the latest packet, at approximately 5 Hz.
        const auto now = std::chrono::steady_clock::now();
        if(now < next_print) {
            continue;
        }
        next_print = now + std::chrono::milliseconds(200);

        const auto& packet = data->packets.back();
        const auto& acceleration = packet.acceleroMeter;
        const auto& gyro = packet.gyroscope;

        // Preserve separate acquisition timestamps.
        // These are device-clock seconds, not wall-clock time.
        const double acceleration_time =
            std::chrono::duration<double>(
                acceleration.getTimestampDevice().time_since_epoch()).count();

        const double gyro_time =
            std::chrono::duration<double>(
                gyro.getTimestampDevice().time_since_epoch()).count();

        const double magnitude = std::sqrt(
            acceleration.x * acceleration.x +
            acceleration.y * acceleration.y +
            acceleration.z * acceleration.z);

        std::cout << "A t=" << acceleration_time
                  << " xyz=[" << acceleration.x
                  << ", " << acceleration.y
                  << ", " << acceleration.z
                  << "] norm=" << magnitude
                  << " m/s^2 | G t=" << gyro_time
                  << " xyz=[" << gyro.x
                  << ", " << gyro.y
                  << ", " << gyro.z
                  << "] rad/s\n";
    }

    if(!received) {
        throw std::runtime_error("No IMU packets received in 20 seconds");
    }

    std::cout << "IMU test finished.\n";
}

int main()
{
    try {
        // Connect and load the camera's runtime into memory.
        dai::Device device;

        std::cout << "Device ID: " << device.getMxId() << '\n';
        std::cout << "IMU: " << device.getConnectedIMU() << '\n';

        const auto speed = device.getUsbSpeed();

        std::cout << "USB speed: ";
        switch(speed) {
            case dai::UsbSpeed::HIGH:
                std::cout << "HIGH (USB 2)\n";
                break;
            case dai::UsbSpeed::SUPER:
                std::cout << "SUPER (USB 3)\n";
                break;
            case dai::UsbSpeed::SUPER_PLUS:
                std::cout << "SUPER_PLUS (USB 3)\n";
                break;
            default:
                std::cout << "Other: " << static_cast<int>(speed) << '\n';
                break;
        }

        for(const auto& camera : device.getCameraSensorNames()) {
            std::cout << "Camera socket "
                      << static_cast<int>(camera.first)
                      << ": " << camera.second << '\n';
        }

        // Read calibration without modifying the device EEPROM.
        auto calibration = device.readCalibration();

        std::cout << std::fixed << std::setprecision(8);

        const std::string filename =
            "oak_calibration_" + device.getMxId() + ".json";

        if(!calibration.eepromToJsonFile(filename)) {
            throw std::runtime_error("Could not export calibration JSON");
        }

        std::cout << "\nCalibration saved to: " << filename << '\n';

        // Inspect the left and right monochrome cameras.
        for(auto socket : {
                dai::CameraBoardSocket::CAM_B,
                dai::CameraBoardSocket::CAM_C}) {

            std::cout << "\nCamera socket "
                      << static_cast<int>(socket) << '\n';

            auto [intrinsics, width, height] =
                calibration.getDefaultIntrinsics(socket);

            std::cout << "Calibration resolution: "
                      << width << " x " << height << '\n';

            std::cout << "Intrinsic matrix:\n";
            print_matrix(intrinsics);

            std::cout << "Distortion coefficients:\n";
            for(float value :
                calibration.getDistortionCoefficients(socket)) {
                std::cout << value << ' ';
            }
            std::cout << '\n';

            // Missing IMU extrinsics should not prevent the streaming test.
            try {
                std::cout << "Camera -> IMU transform"
                          << " (translation in centimeters):\n";

                print_matrix(
                    calibration.getCameraToImuExtrinsics(socket, false));
            }
            catch(const std::exception& error) {
                std::cout << "Camera-to-IMU data unavailable: "
                          << error.what() << '\n';
            }
        }

        std::cout << "\nCAM_B -> CAM_C transform"
                  << " (translation in centimeters):\n";

        print_matrix(calibration.getCameraExtrinsics(
            dai::CameraBoardSocket::CAM_B,
            dai::CameraBoardSocket::CAM_C,
            false));

        test_imu(device);
    }
    catch(const std::exception& error) {
        std::cerr << "Camera inspection failed: "
                  << error.what() << '\n';
        return 1;
    }

    return 0;
}