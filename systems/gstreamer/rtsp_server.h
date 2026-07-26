#ifndef RTSP_SERVER_H
#define RTSP_SERVER_H

// GStreamer includes for RTSP server functionality, RTP handling, and general GStreamer APIs
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtp/rtp.h>
#include <gst/gst.h>

#include <vector>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>
#include <atomic>

class RtspServer {
public:
    // Constructor: takes image height, width, channel count, camera number, frequency, server address and port
    RtspServer(const uint16_t& height, const uint16_t& width, const uint8_t& channels,
        const uint8_t& camera_number, const uint16_t& frequency,
        const std::string& address = "127.0.0.1", const std::string& port = "8554");
    RtspServer();

    // Destructor
    ~RtspServer();

    // Stores incoming image and timestamp data for a specific camera
    void updateImageValue(const uint8_t& camera_number, const std::vector<uint8_t>& image_data);
    void updateTimestamp(const uint8_t& camera_number, const uint64_t& timestamp, const uint64_t& seq_id);

    // Initializes the RTSP server (pipelines, factories, elements, etc)
    bool initialize(); 
    bool initialize_step_by_step();

    // Starts the GMainLoop and runs the server
    void run_server();  
    void stop();
    
    // A helper struct passed to callbacks to provide context
    struct MediaConfigContext {
        RtspServer* server = nullptr;
        int camera_number = 0;
        GstElement* appsrc = nullptr;
        uint16_t camera_frequency = 0;
    };
    
private:
    uint16_t height_;
    uint16_t width_;
    uint8_t channels_;
    uint8_t camera_number_;
    uint16_t frequency_;
    std::vector<std::vector<uint8_t>> image_data_;
    std::vector<std::mutex> image_data_mutex_;
    std::vector<uint64_t> timestamp_;
    std::vector<uint64_t> seq_id_;
    std::vector<std::mutex> timestamp_mutex_;
    std::vector<bool> new_frame_available_;
    std::vector<std::condition_variable> frame_cv_;
    std::vector<std::mutex> frame_cv_mutex_;
    std::vector<std::thread> streaming_threads_;
    std::atomic<bool> stop_streaming_{false};
    GMainLoop* loop_;
    std::string server_address_;
    std::string server_port_;

    void streamingThread(int camera_number, GstElement* appsrc);
    static GstPadProbeReturn timestampStreaming(GstPad* pad, GstPadProbeInfo* info, gpointer user_data);
    static void pad_probe_callback(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data);
};

#endif
