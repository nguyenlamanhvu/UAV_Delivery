#include "uav_image_system.h"

#include <drake/common/text_logging.h>
#include <sstream>
#include <iostream>
#include <stdexcept>

UavImageSystem::UavImageSystem(const int frequency, const int width, 
                                   const int height, const int channels,
                                   const int num_cameras,
                                   const std::string& address, const std::string& port,
                                   const bool step_by_step_mode, const bool use_fine_tune_camera,
                                   const bool mode_collect_data)
    : image_ports_(),
      image_buffers_(num_cameras),
      last_export_time_(num_cameras, 0),
      period_(std::chrono::duration<double>(1.0 / frequency)),
      step_by_step_mode_(step_by_step_mode),
      mode_collect_data_(mode_collect_data),
      num_cameras_(num_cameras),
      // Set the frequency on the GStreamer server slightly higher than the image FPS. 
      // This action significantly improves the handling of lost frame behavior during image streaming.
      rtsp_gstreamer_server(static_cast<uint16_t>(height), static_cast<uint16_t>(width), 
                           static_cast<uint8_t>(channels), static_cast<uint8_t>(num_cameras), 
                           static_cast<uint16_t>(frequency * 1.5), address, port),
      seq_id_(num_cameras, 0) {

    if (num_cameras_ <= 0) {
        throw std::runtime_error("UavImageSystem requires at least one camera");
    }

    if (step_by_step_mode_ && !mode_collect_data_) {
        // Initialize the gstreamet rtsp server
        if (!rtsp_gstreamer_server.initialize_step_by_step()) {
            std::cerr << "Failed to initialize RTSP server step by step" << std::endl;
        }
    }
    else {
        // Initialize the gstreamet rtsp server
        if (!rtsp_gstreamer_server.initialize()) {
            std::cerr << "Failed to initialize RTSP server" << std::endl;
        }
    }

    image_ports_.reserve(num_cameras_);
    for (int i = 0; i < num_cameras_; ++i) {
        image_ports_.push_back(this->DeclareAbstractInputPort(
            "camera_" + std::to_string(i + 1),
            drake::Value<drake::systems::sensors::ImageRgba8U>{}).get_index());
        image_buffers_[i].slot1.drake_image = drake::systems::sensors::ImageRgba8U(width, height);
        image_buffers_[i].slot2.drake_image = drake::systems::sensors::ImageRgba8U(width, height);
    }

    //Check step by step mode
    if (step_by_step_mode_ && !mode_collect_data_) {
        drake::log()->info("UavImageSystem is in step by step mode\r\n");
        period_ = std::chrono::duration<double>(0.001); //Each step is 1ms
    }
    else {
        drake::log()->info("UavImageSystem is in normal mode\r\n");
    }

    this->DeclarePeriodicPublishEvent(0.001, 0.0, &UavImageSystem::ProcessAllCameras);
    if (use_fine_tune_camera) {
        std::cout << "Use fine tune camera" << std::endl;
    }

    rtsp_gstreamer_thread_ = std::thread([this] () {
        rtsp_gstreamer_server.run_server();
    });
    
#if TEST_IMAGE_BUFFER
    test_read_thread_ = std::thread(&UavImageSystem::TestImageBuffer, this);
#endif
    drake::log()->debug("UavImageSystem constructor\r\n"); 
#if DEBUG_TIME_ELAPSED
    begin_time = std::chrono::high_resolution_clock::now();
    log_files_.resize(num_cameras_);
    // Log file
    for (int i = 0; i < num_cameras_; ++i) {
        std::string cam_name = "camera_" + std::to_string(i + 1);
        log_files_[i].open("log_" + cam_name + ".csv", std::ios::app);
        
        if (log_files_[i].is_open()) {
            log_files_[i] << "timestamp_us,elapsed_us\n";
        }
    }    
#endif
}

UavImageSystem::~UavImageSystem() {
    if (rtsp_gstreamer_thread_.joinable()) {
        rtsp_gstreamer_thread_.join();
    }
#if TEST_IMAGE_BUFFER
    if (test_read_thread_.joinable()) {
        test_read_thread_.join();
    }
#endif
#if DEBUG_TIME_ELAPSED
    for (int i = 0; i < num_cameras_; ++i) {
        if (log_files_[i].is_open()) {
            log_files_[i].close();
        }
    }
#endif
}

// Generic camera processing function
drake::systems::EventStatus UavImageSystem::ProcessAllCameras(
    const drake::systems::Context<double>& context) const {
    for (int i = 0; i < num_cameras_; ++i) {
        ProcessCameraGeneric(context, static_cast<std::size_t>(i),
                             "camera_" + std::to_string(i + 1));
    }
    return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus UavImageSystem::ProcessCameraGeneric(
    const drake::systems::Context<double>& context,
    std::size_t camera_index,
    const std::string& camera_name) const {

    try {
        // Get simulation time directly from context
        double t = context.get_time();
        int64_t sim_time = static_cast<int64_t>(t * 1e6);
        
        // Check frequency based on time sim
        double time_interval = period_.count() * 1e6; //convert to us
        if (sim_time - last_export_time_[camera_index] >= time_interval || 
            (mode_collect_data_ && last_export_time_[camera_index] - sim_time >= 1000)) { // In data collection mode, export at least every 1ms to avoid long gaps
            seq_id_[camera_index]++;
#if DEBUG_TIME_ELAPSED
            auto start_time = std::chrono::high_resolution_clock::now();
#endif

            const drake::systems::sensors::ImageRgba8U* drake_image_ptr = nullptr;
            try {
                const auto& img_port = this->get_input_port(image_ports_.at(camera_index));
                drake_image_ptr = &img_port.Eval<drake::systems::sensors::ImageRgba8U>(context);
            } catch (const std::exception& e) {
                std::cerr << "error input port " << camera_name << ": " << e.what() << std::endl;
                return drake::systems::EventStatus::Succeeded();
            }

            if (drake_image_ptr) {
                auto& image_buffer = image_buffers_.at(camera_index);
                std::lock_guard<std::mutex> lock(image_buffer.write_mtx);

                // Determine inactive slot
                const uint8_t active_idx = image_buffer.active.load(std::memory_order_acquire);
                ImageWorkItem* target = (active_idx == 0) ? &image_buffer.slot2 : &image_buffer.slot1;

                // Copy Drake image into slot buffer
                target->drake_image = *drake_image_ptr;
                // Convert Drake image to std::vector<uint8_t>
                std::vector<uint8_t> image_data = ToVector(target->drake_image);
                
                uint64_t timestamp_us = static_cast<uint64_t>(sim_time * 1e3);
                // uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                //     std::chrono::high_resolution_clock::now().time_since_epoch()
                // ).count();
                // std::cout << "Sim_time " << camera_name << ": " << timestamp_us << std::endl;

                // Send to RTSP server
                rtsp_gstreamer_server.updateTimestamp(static_cast<uint8_t>(camera_index), timestamp_us, seq_id_[camera_index]);
                rtsp_gstreamer_server.updateImageValue(static_cast<uint8_t>(camera_index), image_data);

                // Publish by flipping the active index
                const uint8_t new_active = (active_idx == 0) ? 1 : 0;
                image_buffer.active.store(new_active, std::memory_order_release);
            }

#if DEBUG_TIME_ELAPSED
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed_us =
                std::chrono::duration<double, std::micro>(end_time - start_time).count();
            std::size_t id = camera_index;
            if (log_files_[id].is_open()) {
                log_files_[id] << sim_time << "," << elapsed_us << "\n";
            }
            // std::cout << "Camera " << camera_name << " : " << elapsed_us << std::endl;
#endif
            
            // Update last export time (note: this might cause race condition with multiple cameras)
            // Consider using per-camera timestamps if needed
            last_export_time_[camera_index] = sim_time;
        }
    } catch (const std::exception& e) {
        drake::log()->warn("Failed to process {}: {}", camera_name, e.what());
    }
    
    return drake::systems::EventStatus::Succeeded();
}

inline const UavImageSystem::ImageWorkItem& UavImageSystem::GetImage(const UavImageSystem::ImageWriterReader& image_buffer)
{
    const uint8_t idx = image_buffer.active.load(std::memory_order_acquire);
    return (idx == 0) ? image_buffer.slot1 : image_buffer.slot2;
}

#if TEST_IMAGE_BUFFER
void UavImageSystem::TestImageBuffer(void) {
    while (true) {
        auto start_time = std::chrono::steady_clock::now();

        const auto& image_buffer_1 = image_buffers_.front();
        uint8_t active_idx = image_buffer_1.active.load(std::memory_order_acquire);
        const ImageWorkItem* src = (active_idx == 0) ? &image_buffer_1.slot1 : &image_buffer_1.slot2;

        std::cout << "image_buffer_1 timestamp: " << src->timestamp.sec << "." << src->timestamp.nanosec << "\n";
        std::cout << "image_buffer_1 buffer size: " << src->drake_image.size() << "\n";
        std::cout << "Active slot: " << static_cast<int>(active_idx) << "\n";

        std::this_thread::sleep_until(start_time + std::chrono::milliseconds(20)); // 50Hz
    }
}
#endif

// Convert a Drake ImageRgba8U to a std::vector<uint8_t>
inline std::vector<uint8_t> UavImageSystem::ToVector(
    const drake::systems::sensors::ImageRgba8U& image) const {
    const int width = image.width();
    const int height = image.height();
    const int channels = image.kNumChannels; 

    std::vector<uint8_t> out;
    out.reserve(width * height * channels);

    const uint8_t* src = image.at(0, 0); // beginning of buffer
    out.insert(out.end(), src, src + (width * height * channels));

    return out;
}
