#include "rtsp_server.h"

RtspServer::RtspServer() {}

RtspServer::RtspServer(const uint16_t& height, const uint16_t& width, const uint8_t& channels,
                       const uint8_t& camera_number, const uint16_t& frequency,
                       const std::string& address, const std::string& port)
    : height_(height), width_(width), channels_(channels), camera_number_(camera_number), 
      frequency_(frequency), image_data_(camera_number), image_data_mutex_(camera_number), 
      timestamp_(camera_number), seq_id_(camera_number),timestamp_mutex_(camera_number), 
      new_frame_available_(camera_number, false), frame_cv_(camera_number), frame_cv_mutex_(camera_number),
      loop_(nullptr), server_address_(address), server_port_(port) {
    
    if (0 >= height_ || 0 >= width_ || 0 >= channels_ || 0 >= camera_number ||0 >= frequency_ ) {
        g_error("Image parameters, image channel, camera number and frequency must be higher than zero\n");
    }

    // Initialize the drake image values to black
    for (int i = 0; i < camera_number_; ++i) {
        image_data_[i].resize(height_ * width_ * channels_, 0);
    }
}

RtspServer::~RtspServer() {
    stop();
    if (loop_) {
        g_main_loop_unref(loop_);
        loop_ = nullptr;
    }
}

void RtspServer::updateImageValue(const uint8_t& camera_number, const std::vector<uint8_t>& image_data) {
    if (camera_number < 0 || camera_number >= camera_number_) {
        g_printerr("updateImageValue: invalid camera number %u\n", camera_number);
        return;
    }
    
    if (height_ * width_ * channels_ != image_data.size()) {
        g_printerr("updateImageValue: invalid image size of camera %u\n", camera_number);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(image_data_mutex_[camera_number]);
        image_data_[camera_number]        = image_data;
        new_frame_available_[camera_number] = true;
    }
    // Wake up streaming thread immediately — zero timer jitter
    frame_cv_[camera_number].notify_one();
}

void RtspServer::updateTimestamp(const uint8_t& camera_number, const uint64_t& timestamp, const uint64_t& seq_id) {
    if (camera_number < 0 || camera_number >= camera_number_) {
        g_printerr("updateImageValue: invalid camera number %u\n", camera_number);
        return;
    }

    std::lock_guard<std::mutex> lock(timestamp_mutex_[camera_number]);
    timestamp_[camera_number] = timestamp;
    seq_id_[camera_number] = seq_id;
}

void RtspServer::stop() {
    if (stop_streaming_.exchange(true)) return;

    // Wake all streaming threads
    for (int i = 0; i < camera_number_; ++i)
        frame_cv_[i].notify_all();

    // Join streaming threads
    for (auto& t : streaming_threads_)
        if (t.joinable()) t.join();
    streaming_threads_.clear();

    if (loop_) {
        g_main_loop_quit(loop_);
    }
}

void RtspServer::streamingThread(int camera_number, GstElement* appsrc) {
    while (!stop_streaming_.load()) {
        std::vector<uint8_t> frame_copy;

        // Block until a new frame is signaled or stop is requested
        {
            std::unique_lock<std::mutex> lock(frame_cv_mutex_[camera_number]);
            frame_cv_[camera_number].wait_for(lock, std::chrono::seconds(1), [&] {
                return new_frame_available_[camera_number] || stop_streaming_.load();
            });
            if (stop_streaming_.load()) break;
        }

        // Copy frame under image_data_mutex_ and reset flag
        {
            std::lock_guard<std::mutex> lock(image_data_mutex_[camera_number]);
            frame_copy = image_data_[camera_number];
            new_frame_available_[camera_number] = false;
        }

        if (frame_copy.empty()) continue;

        // Allocate GstBuffer and copy pixel data
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, frame_copy.size(), nullptr);
        if (!buffer) continue;

        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
            memcpy(map.data, frame_copy.data(), frame_copy.size());
            gst_buffer_unmap(buffer, &map);
        }

        // Push buffer to appsrc — pipeline picks it up immediately
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);

        if (ret != GST_FLOW_OK) {
            g_printerr("Camera %d: fatal flow error (%s), stopping thread\n",
                       camera_number + 1, gst_flow_get_name(ret));
            break;
        }
    }
}

static void destroy_media_context(gpointer data, GClosure*) {
    auto* ctx = static_cast<RtspServer::MediaConfigContext*>(data);
    if (ctx->appsrc) gst_object_unref(ctx->appsrc);
    delete ctx;
}

GstPadProbeReturn RtspServer::timestampStreaming(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
    auto* ctx = static_cast<MediaConfigContext*>(user_data);
    RtspServer* server = ctx->server;
    int camera_number = ctx->camera_number;
    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;

    if (!gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp_buffer)) {
        g_printerr("Server: Failed to map RTP buffer\n");
        return GST_PAD_PROBE_OK;
    }

    if (gst_rtp_buffer_get_marker(&rtp_buffer)) {
        uint64_t timestamp_copy, seq_id_copy;
        {
            std::lock_guard<std::mutex> lock(server->timestamp_mutex_[camera_number]);
            timestamp_copy = server->timestamp_[camera_number];
            seq_id_copy = server->seq_id_[camera_number];
        }
        // 16 bytes: [0-7] Timestamp, [8-15] Sequence ID
        guint8 data[16] = {0};
        for (size_t i = 0; i < 8; ++i) {
            data[7 - i] = static_cast<guint8>((timestamp_copy >> (i * 8)) & 0xFF);
            data[15 - i] = static_cast<guint8>((seq_id_copy >> (i * 8)) & 0xFF);
        }

        if (!gst_rtp_buffer_add_extension_onebyte_header(&rtp_buffer, 1, data, sizeof(data))) {
            g_printerr("Server: Failed to add RTP extension header\n");
        }
    }

    gst_rtp_buffer_unmap(&rtp_buffer);
    return GST_PAD_PROBE_OK;
}

void RtspServer::pad_probe_callback(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data) {
    auto* media_ctx = static_cast<MediaConfigContext*>(user_data);
    int camera_number = media_ctx->camera_number;
    g_print("Server: Client connected to /Drake_camera_%d port for image streaming\n", camera_number + 1);

    GstElement* pipeline = gst_rtsp_media_get_element(media);
    GstElement* appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "source");
    GstElement* payloader = gst_bin_get_by_name(GST_BIN(pipeline), "pay0");

    if (appsrc && payloader) {
        GstPad* src_pad = gst_element_get_static_pad(payloader, "src");
        if (src_pad) {
            g_object_set(appsrc, "format", GST_FORMAT_TIME, "is-live", TRUE, "do-timestamp", TRUE, nullptr);
            media_ctx->appsrc = appsrc;
            gst_object_ref(appsrc); // keep while timer runs

            gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, timestampStreaming, media_ctx, nullptr);
            gst_object_unref(src_pad);

            media_ctx->server->streaming_threads_.emplace_back(
                &RtspServer::streamingThread, media_ctx->server, camera_number, appsrc);
        } else {
            g_printerr("Server: Failed to get src pad for pay0\n");
        }    
        gst_object_unref(appsrc); 
        gst_object_unref(payloader);
    } else {
        g_printerr("Server: Failed to find appsrc and pay0 element\n");
    }

    gst_object_unref(pipeline);
}

bool RtspServer::initialize() {
    gst_init(nullptr, nullptr);
    loop_ = g_main_loop_new(nullptr, FALSE);

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_address(server, server_address_.c_str());
    gst_rtsp_server_set_service(server, server_port_.c_str());
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    for (int i = 0; i < camera_number_; ++i) {
        std::string pipeline_str = "appsrc name=source is-live=true format=3 ! "
            "video/x-raw,format=RGBA,width=" + std::to_string(width_) +
            ",height=" + std::to_string(height_) + ",framerate=" + std::to_string(frequency_) + "/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency speed-preset=ultrafast ! "
            "queue ! "
            "rtph264pay name=pay0";

        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_launch(factory, pipeline_str.c_str());
        gst_rtsp_media_factory_set_shared(factory, TRUE);

        std::string mount_path = "/Drake_camera_" + std::to_string(i + 1);
        gst_rtsp_mount_points_add_factory(mounts, mount_path.c_str(), factory);

        // Allocate and pass camera context
        auto* media_ctx = new MediaConfigContext{ this, i, nullptr, frequency_ };
        g_signal_connect_data(factory, "media-configure", G_CALLBACK(RtspServer::pad_probe_callback),
                              media_ctx, destroy_media_context, G_CONNECT_AFTER);
    }

    g_object_unref(mounts);

    if (!gst_rtsp_server_attach(server, nullptr)) {
        g_printerr("Failed to attach RTSP server\n");
        return false;
    }
    return true;
}

bool RtspServer::initialize_step_by_step() {
    gst_init(nullptr, nullptr);
    loop_ = g_main_loop_new(nullptr, FALSE);

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_address(server, server_address_.c_str());
    gst_rtsp_server_set_service(server, server_port_.c_str());
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    for (int i = 0; i < camera_number_; ++i) {
        std::string pipeline_str = "appsrc name=source is-live=true format=3 ! "
            "video/x-raw,format=RGBA,width=" + std::to_string(width_) +
            ",height=" + std::to_string(height_) + ",framerate=" + std::to_string(frequency_) + "/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "x264enc bitrate=2048 vbv-buf-capacity=100 tune=zerolatency speed-preset=ultrafast ! "
            "rtph264pay name=pay0 pt=96";

        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_launch(factory, pipeline_str.c_str());
        gst_rtsp_media_factory_set_shared(factory, TRUE);

        std::string mount_path = "/Drake_camera_" + std::to_string(i + 1);
        gst_rtsp_mount_points_add_factory(mounts, mount_path.c_str(), factory);

        // Allocate and pass camera context
        auto* media_ctx = new MediaConfigContext{ this, i, nullptr, frequency_ };
        g_signal_connect_data(factory, "media-configure", G_CALLBACK(RtspServer::pad_probe_callback),
                              media_ctx, destroy_media_context, G_CONNECT_AFTER);
    }

    g_object_unref(mounts);

    if (!gst_rtsp_server_attach(server, nullptr)) {
        g_printerr("Failed to attach RTSP server\n");
        return false;
    }
    return true;
}

void RtspServer::run_server() {
    if (!loop_) {
        g_printerr("Server: Main loop not initialized\n");
        return;
    }
    g_print("RTSP server running at rtsp://%s:%s/Drake_camera_[1-%u]\n", 
             server_address_.c_str(), server_port_.c_str(), camera_number_);
    g_main_loop_run(loop_);
    g_print("Server: run_server exited cleanly.\n");
}
