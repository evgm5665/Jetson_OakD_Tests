#include <depthai/depthai.hpp>

#include <chrono>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

int main()
{
    try {
        dai::Pipeline pipeline;

        auto imu = pipeline.create<dai::node::IMU>();
        auto output = pipeline.create<dai::node::XLinkOut>();

        imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER, 100);
        imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 100);
        imu->enableFirmwareUpdate(false);
        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(10);

        output->setStreamName("imu");
        imu->out.link(output->input);

        // Give each recording a separate filename.
        const auto recording_id =
            std::chrono::system_clock::now().time_since_epoch().count();

        const std::string filename =
            "imu_motion_" + std::to_string(recording_id) + ".csv";

        std::ofstream csv(filename);
        if(!csv) {
            throw std::runtime_error("Cannot create recording file");
        }

        csv << "sensor,phase,sequence,device_time_s,x,y,z\n";
        csv << std::setprecision(12);

        dai::Device device(pipeline);
        auto queue = device.getOutputQueue("imu", 32, false);

        // Acceleration and gyro reports are recorded separately.
        // Repeated reports in combined packets are skipped by sequence number.
        bool have_acceleration = false;
        bool have_gyro = false;
        int32_t last_acceleration_sequence = 0;
        int32_t last_gyro_sequence = 0;

        unsigned long acceleration_count = 0;
        unsigned long gyro_count = 0;

        const auto start = std::chrono::steady_clock::now();
        int previous_phase = -1;

        while(true) {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();

            if(elapsed >= 25.0) {
                break;
            }

            const int phase =
                elapsed < 5.0 ? 0 : (elapsed < 15.0 ? 1 : 2);

            if(phase != previous_phase) {
                if(phase == 0) {
                    std::cout
                        << "STILL: lenses upward; leave untouched for 5 seconds."
                        << std::endl;
                } else if(phase == 1) {
                    std::cout
                        << "CLOCKWISE: viewed from above, turn slowly about "
                        << "90 degrees. Keep lenses upward. Stop when done; "
                        << "do not turn back."
                        << std::endl;
                } else {
                    std::cout
                        << "STILL: leave it in its new position until finished."
                        << std::endl;
                }

                previous_phase = phase;
            }

            auto data = queue->tryGet<dai::IMUData>();

            if(!data) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            for(const auto& packet : data->packets) {
                const auto& a = packet.acceleroMeter;
                const auto& g = packet.gyroscope;

                if(!have_acceleration ||
                   a.sequence != last_acceleration_sequence) {

                    const double timestamp = std::chrono::duration<double>(
                        a.getTimestampDevice().time_since_epoch()).count();

                    csv << "acceleration," << phase << ',' << a.sequence
                        << ',' << timestamp
                        << ',' << a.x << ',' << a.y << ',' << a.z << '\n';

                    last_acceleration_sequence = a.sequence;
                    have_acceleration = true;
                    ++acceleration_count;
                }

                if(!have_gyro || g.sequence != last_gyro_sequence) {
                    const double timestamp = std::chrono::duration<double>(
                        g.getTimestampDevice().time_since_epoch()).count();

                    csv << "gyro," << phase << ',' << g.sequence
                        << ',' << timestamp
                        << ',' << g.x << ',' << g.y << ',' << g.z << '\n';

                    last_gyro_sequence = g.sequence;
                    have_gyro = true;
                    ++gyro_count;
                }
            }
        }

        csv.close();
        if(!csv) {
            throw std::runtime_error("Recording file write failed");
        }

        std::cout << "Saved: " << filename
                  << "\nAcceleration reports: " << acceleration_count
                  << "\nGyro reports: " << gyro_count << '\n';

        if(acceleration_count == 0 || gyro_count == 0) {
            throw std::runtime_error("One or both IMU streams were missing");
        }
    }
    catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}