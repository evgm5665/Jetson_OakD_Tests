#include <depthai/depthai.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

using Clock = std::chrono::steady_clock;

struct Stats {
    uint64_t count = 0, gaps = 0, backwards = 0;
    int64_t sequence = 0;
    double first = 0, last = 0;
    void add(int64_t seq, double time) {
        if(count) {
            if(seq > sequence + 1) gaps += seq - sequence - 1;
            if(time <= last || seq <= sequence) ++backwards;
        } else first = time;
        sequence = seq;
        last = time;
        ++count;
    }
    void print(std::ostream& out, const std::string& name) const {
        out << name << ": reports=" << count
            << ", received_Hz=" << (count > 1 && last > first ? (count-1)/(last-first) : 0)
            << ", skipped_sequence_numbers=" << gaps
            << ", nonmonotonic=" << backwards << '\n';
    }
};

double seconds(Clock::time_point time) {
    return std::chrono::duration<double>(time.time_since_epoch()).count();
}

void save_image(const std::filesystem::path& path, const dai::ImgFrame& image) {
    const auto& bytes = image.getData();
    if(image.getType() != dai::ImgFrame::Type::RAW8 ||
       bytes.size() != static_cast<size_t>(image.getWidth()) * image.getHeight())
        throw std::runtime_error("Unexpected grayscale image format/size");
    std::ofstream file(path, std::ios::binary);
    file << "P5\n" << image.getWidth() << ' ' << image.getHeight() << "\n255\n";
    file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    if(!file) throw std::runtime_error("Image save failed");
}

int main() {
    try {
        const auto id = std::chrono::system_clock::now().time_since_epoch().count();
        const auto directory = std::filesystem::path("stereo_imu_" + std::to_string(id));
        std::filesystem::create_directory(directory);
        std::ofstream frames(directory / "frames.csv");
        std::ofstream reports(directory / "imu.csv");
        std::ofstream pairs(directory / "pairs.csv");
        std::ofstream summary(directory / "summary.txt");
        if(!frames || !reports || !pairs || !summary) throw std::runtime_error("Cannot create output files");
        frames << "camera,sequence,device_time_s,width,height\n";
        reports << "sensor,sequence,device_time_s,x,y,z\n";
        pairs << "sequence,left_time_s,right_time_s,right_minus_left_us\n";
        frames << std::setprecision(12);
        reports << std::setprecision(12);
        pairs << std::setprecision(12);

        dai::Pipeline pipeline;
        for(const auto& entry : {std::make_pair(dai::CameraBoardSocket::CAM_B, "left"),
                                  std::make_pair(dai::CameraBoardSocket::CAM_C, "right")}) {
            auto camera = pipeline.create<dai::node::MonoCamera>();
            camera->setBoardSocket(entry.first);
            camera->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
            camera->setFps(20);
            auto out = pipeline.create<dai::node::XLinkOut>();
            out->setStreamName(entry.second);
            camera->out.link(out->input);
        }
        auto imu = pipeline.create<dai::node::IMU>();
        imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER, 100);
        imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 100);
        imu->enableFirmwareUpdate(false);
        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(10);
        auto imu_out = pipeline.create<dai::node::XLinkOut>();
        imu_out->setStreamName("imu");
        imu->out.link(imu_out->input);

        dai::Device device(pipeline);
        auto calibration = device.readCalibration();
        calibration.eepromToJsonFile((directory / "calibration.json").string());
        summary << "device=" << device.getMxId() << "\nIMU=" << device.getConnectedIMU()
                << "\nDepthAI=2.32.0\nUSB_speed_enum=" << static_cast<int>(device.getUsbSpeed())
                << "\ncam0=CAM_B\ncam1=CAM_C\nimages=unrectified RAW8 1280x800, requested 20 Hz"
                << "\nacceleration=ACCELEROMETER, m/s^2, requested 100 Hz"
                << "\ngyro=GYROSCOPE_RAW, rad/s, requested 100 Hz"
                << "\ntimestamps=device clock; image timestamp uses SDK default exposure reference"
                << "\nNo camera-IMU time offset or noise calibration is performed.\n";
        auto left = device.getOutputQueue("left", 8, false);
        auto right = device.getOutputQueue("right", 8, false);
        auto inertial = device.getOutputQueue("imu", 64, false);
        Stats ls, rs, as, gs;
        std::map<int64_t, std::shared_ptr<dai::ImgFrame>> lbuf, rbuf;
        uint64_t paired = 0, discarded = 0;
        double maximum_skew_us = 0;
        const auto start = Clock::now();
        std::cout << "Leave camera stationary, lenses unobstructed. Recording for 20 seconds.\n" << std::flush;

        while(Clock::now() - start < std::chrono::seconds(20)) {
            auto drain_camera = [&](auto queue, const char* name, Stats& stats, auto& buffer) {
                for(int n = 0; n < 16; ++n) {
                    auto image = queue->template tryGet<dai::ImgFrame>();
                    if(!image) break;
                    const auto seq = image->getSequenceNum();
                    const double time = seconds(image->getTimestampDevice());
                    if(image->getWidth() != 1280 || image->getHeight() != 800 ||
                       image->getType() != dai::ImgFrame::Type::RAW8)
                        throw std::runtime_error("Unexpected camera stream geometry/type");
                    stats.add(seq, time);
                    frames << name << ',' << seq << ',' << time << ','
                           << image->getWidth() << ',' << image->getHeight() << '\n';
                    buffer[seq] = image;
                    if(buffer.size() > 16) { buffer.erase(buffer.begin()); ++discarded; }
                }
            };
            drain_camera(left, "left", ls, lbuf);
            drain_camera(right, "right", rs, rbuf);
            while(!lbuf.empty() && !rbuf.empty()) {
                auto l = lbuf.begin(); auto r = rbuf.begin();
                if(l->first < r->first) { lbuf.erase(l); ++discarded; continue; }
                if(r->first < l->first) { rbuf.erase(r); ++discarded; continue; }
                const double lt = seconds(l->second->getTimestampDevice());
                const double rt = seconds(r->second->getTimestampDevice());
                const double skew = (rt-lt)*1e6;
                maximum_skew_us = std::max(maximum_skew_us, std::abs(skew));
                pairs << l->first << ',' << lt << ',' << rt << ',' << skew << '\n';
                if(paired == 0) {
                    save_image(directory / "left.pgm", *l->second);
                    save_image(directory / "right.pgm", *r->second);
                }
                ++paired;
                lbuf.erase(l); rbuf.erase(r);
            }
            for(int n = 0; n < 128; ++n) {
                auto data = inertial->tryGet<dai::IMUData>();
                if(!data) break;
                for(const auto& packet : data->packets) {
                    auto record = [&](const auto& report, const char* name, Stats& stats) {
                        if(stats.count && report.sequence == stats.sequence) return;
                        const double time = seconds(report.getTimestampDevice());
                        stats.add(report.sequence, time);
                        reports << name << ',' << report.sequence << ',' << time << ','
                                << report.x << ',' << report.y << ',' << report.z << '\n';
                    };
                    record(packet.acceleroMeter, "acceleration", as);
                    record(packet.gyroscope, "gyro", gs);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for(auto* out : {static_cast<std::ostream*>(&summary), static_cast<std::ostream*>(&std::cout)}) {
            ls.print(*out, "left"); rs.print(*out, "right");
            as.print(*out, "acceleration"); gs.print(*out, "gyro");
            *out << "paired_frames=" << paired << ", max_abs_pair_skew_us=" << maximum_skew_us
                 << ", discarded_unpaired_frames=" << discarded
                 << ", pending_unpaired_frames=" << lbuf.size()+rbuf.size() << '\n';
        }
        frames.close(); reports.close(); pairs.close(); summary.close();
        if(!frames || !reports || !pairs || !summary) throw std::runtime_error("Output write failed");
        std::cout << "Saved folder: " << std::filesystem::absolute(directory) << '\n' << std::flush;
        device.close();
        std::cout << "Device close returned.\n";
        if(paired == 0 || as.count == 0 || gs.count == 0) return 2;
    } catch(const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
