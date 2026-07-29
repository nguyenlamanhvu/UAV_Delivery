#pragma once

#include <drake/common/value.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/sensors/image.h>
#include <drake/systems/sensors/image_writer.h>
#include "gstreamer/rtsp_server.h"


#include <chrono>
#include <condition_variable>
#include <mutex>
#include <array>
#include <atomic>
#include <thread>
#include <string>
#include <functional>
#include <fstream>
#include <vector>
#include <cstdint>

#define DEBUG_TIME_ELAPSED 0
#define TEST_IMAGE_BUFFER 0

/**
 * System that gets Drake images.
 */
class UavImageSystem : public drake::systems::LeafSystem<double> {
public:
    explicit UavImageSystem(const int frequency, const int width, 
                              const int height, const int channels,
                              const int num_cameras,
                              const std::string& address, const std::string& port,
                              const bool step_by_step_mode,
                              const bool use_fine_tune_camera = false,
                              const bool mode_collect_data = false);

    const drake::systems::InputPort<double>& get_input_port_image_1() const {
        return this->get_input_port(image_ports_.at(0));
    }
    ~UavImageSystem();

private:
    struct TimeStamp {
        uint32_t sec;
        uint32_t nanosec;
    };

    struct ImageWorkItem {
        drake::systems::sensors::ImageRgba8U drake_image;
        TimeStamp timestamp;
    };

    struct ImageWriterReader {
        ImageWorkItem slot1;
        ImageWorkItem slot2;
        std::atomic<uint8_t> active{0};
        mutable std::mutex write_mtx;
    };

    drake::systems::EventStatus ProcessAllCameras(
        const drake::systems::Context<double>& context) const;
    drake::systems::EventStatus ProcessCameraGeneric(
        const drake::systems::Context<double>& context,
        std::size_t camera_index,
        const std::string& camera_name) const;
    const ImageWorkItem& GetImage(const ImageWriterReader& image_buffer);
    std::vector<uint8_t> ToVector(const drake::systems::sensors::ImageRgba8U& image) const;
    std::vector<drake::systems::InputPortIndex> image_ports_;
    mutable std::vector<ImageWriterReader> image_buffers_;
    mutable std::vector<int64_t> last_export_time_;
    std::chrono::duration<double> period_;
    bool step_by_step_mode_;
    bool mode_collect_data_;
    int num_cameras_;
    std::thread rtsp_gstreamer_thread_;
    mutable std::mutex queue_mutex_;
    mutable std::condition_variable queue_cv_;
    mutable std::atomic<const drake::systems::Context<double>*> image_context_{nullptr};
    mutable RtspServer rtsp_gstreamer_server;
    mutable std::vector<uint64_t> seq_id_;

#if DEBUG_TIME_ELAPSED
    mutable std::chrono::high_resolution_clock::time_point begin_time{
        std::chrono::high_resolution_clock::now()};
    mutable std::vector<std::ofstream> log_files_;
#endif

#if TEST_IMAGE_BUFFER
    mutable std::thread test_read_thread_;
    void TestImageBuffer(void);
#endif
};
