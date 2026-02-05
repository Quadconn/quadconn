#include <gst/gst.h>
#include <iostream>
#include <string>
#include <format> // Requires C++20

int main(int argc, char* argv[]) {
    // 1. Check for the IP argument
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <destination_ip>" << std::endl;
        std::cerr << "Example: " << argv[0] << " 67.67.67.67" << std::endl;
        return 1;
    }

    std::string host_ip = argv[1];

    // 2. Initialize GStreamer
    gst_init(&argc, &argv);

    // 3. Build the pipeline string dynamically
    // We use std::format to inject the host_ip into the udpsink host property
    std::string pipeline_str = std::format(
        "v4l2src device=/dev/video0 ! "
        "videoconvert ! videoscale ! "
        "video/x-raw,width=1280,height=720,framerate=30/1,format=I420 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "x264enc bitrate=2000 tune=zerolatency speed-preset=ultrafast ! "
        "rtph264pay mtu=1300 ! "
        "udpsink host={} port=5000 sync=false", 
        host_ip
    );

    std::cout << "Targeting Host IP: " << host_ip << std::endl;

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
        std::cerr << "Pipeline error: " << error->message << std::endl;
        g_error_free(error);
        return 1;
    }

    // 4. Run the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GstBus* bus = gst_element_get_bus(pipeline);
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus, GST_CLOCK_TIME_NONE, 
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS)
    );

    // Clean up
    if (msg != nullptr) gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}