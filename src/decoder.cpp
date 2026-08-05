#include <iostream>
#include <thread>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <optional>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
}

static enum AVPixelFormat get_hw_format(AVCodecContext*, const enum AVPixelFormat *pix_fmts) {
    const enum AVPixelFormat *p;
    for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == AV_PIX_FMT_CUDA) {
            return *p;
        }
    }
    return AV_PIX_FMT_NONE;
}

class H264DecoderNode : public rclcpp::Node {
private:
    struct PendingFrame {
        cv::Mat image;
        std_msgs::msg::Header header;
    };

    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVCodecParserContext* parser_ctx_ = nullptr;
    AVPacket* pkt_ = nullptr;
    AVFrame* hw_frame_ = nullptr;
    AVFrame* sw_frame_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;
    cv::Mat bgr_frame_; 
    AVBufferRef *hw_device_ctx_ = nullptr;
    enum AVHWDeviceType hw_type_ = AV_HWDEVICE_TYPE_NONE;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::thread publisher_thread_;
    std::optional<PendingFrame> latest_frame_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> stop_publisher_thread_{false};
    int publish_rate_hz_ = 30;
    int output_width_ = 1920;
    int output_height_ = 960;
    
    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;

    void InitFFmpegDecoder() {
        hw_type_ = AV_HWDEVICE_TYPE_CUDA;
        const char* decoder_name = "h264_cuvid";

        codec_ = avcodec_find_decoder_by_name(decoder_name);
        if (!codec_) {
            RCLCPP_WARN(this->get_logger(), "Hardware decoder not available, falling back to software");
            hw_type_ = AV_HWDEVICE_TYPE_NONE;
            codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
            if (!codec_) {
                RCLCPP_ERROR(this->get_logger(), "No H.264 decoder available");
                return;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Using hardware H.264 decoder (NVDEC)");
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            int err = av_hwdevice_ctx_create(&hw_device_ctx_, hw_type_, nullptr, nullptr, 0);
            if (err < 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to create hardware device context, falling back to software");
                hw_type_ = AV_HWDEVICE_TYPE_NONE;
                codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
                if (!codec_) {
                    RCLCPP_ERROR(this->get_logger(), "No H.264 decoder available");
                    return;
                }
            }
        }

        parser_ctx_ = av_parser_init(codec_->id);
        if (!parser_ctx_) {
            CleanupFFmpegDecoder();
            return;
        }

        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!codec_ctx_) {
            CleanupFFmpegDecoder();
            return;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = get_hw_format;
        }

        // Compressed message stamps are passed to FFmpeg as nanosecond PTS.
        // The decoded AVFrame PTS therefore identifies the source ROS header.
        codec_ctx_->pkt_timebase = AVRational{1, 1000000000};

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
            CleanupFFmpegDecoder();
            return;
        }

        pkt_ = av_packet_alloc();
        if (!pkt_) {
            CleanupFFmpegDecoder();
            return;
        }

        hw_frame_ = av_frame_alloc();
        if (!hw_frame_) {
            CleanupFFmpegDecoder();
            return;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            sw_frame_ = av_frame_alloc();
            if (!sw_frame_) {
                CleanupFFmpegDecoder();
                return;
            }
        }
    }

    void PublisherThreadLoop() {
        const auto publish_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_));
        auto next_publish_time = std::chrono::steady_clock::now() + publish_period;

        while (!stop_publisher_thread_) {
            std::optional<PendingFrame> frame_to_publish;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait_until(lock, next_publish_time, [this] {
                    return stop_publisher_thread_.load();
                });

                if (stop_publisher_thread_) {
                    break;
                }

                // Move out the newest decoded frame and clear the slot. Frames
                // overwritten before this point are intentionally dropped.
                frame_to_publish = std::move(latest_frame_);
                latest_frame_.reset();
            }

            if (frame_to_publish && !frame_to_publish->image.empty() && publisher_) {
                auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
                cv_bridge::CvImage cv_image(
                    frame_to_publish->header,
                    sensor_msgs::image_encodings::BGR8,
                    frame_to_publish->image);
                cv_image.toImageMsg(*img_msg);
                publisher_->publish(std::move(img_msg));
            }

            next_publish_time += publish_period;
            const auto now = std::chrono::steady_clock::now();
            if (next_publish_time < now) {
                next_publish_time = now + publish_period;
            }
        }
    }

    void DecodeAndDisplayPacket(AVPacket* packet) {
        int ret = avcodec_send_packet(codec_ctx_, packet);
        if (ret < 0) {
            return;
        }

        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_ctx_, hw_frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return;
            } else if (ret < 0) {
                return;
            }

            AVFrame* frame_to_display = hw_frame_;

            if (hw_frame_->format == AV_PIX_FMT_CUDA || hw_frame_->format == AV_PIX_FMT_VAAPI) {
                if (av_hwframe_transfer_data(sw_frame_, hw_frame_, 0) < 0) {
                    av_frame_unref(hw_frame_);
                    continue;
                }
                frame_to_display = sw_frame_;
            }

            if (!sws_ctx_ && frame_to_display->width > 0 && frame_to_display->height > 0) {
                sws_ctx_ = sws_getContext(
                    frame_to_display->width, frame_to_display->height, (AVPixelFormat)frame_to_display->format,
                    output_width_, output_height_, AV_PIX_FMT_BGR24,
                    SWS_BILINEAR, nullptr, nullptr, nullptr);
                
                if (!sws_ctx_) {
                    av_frame_unref(hw_frame_);
                    if (frame_to_display == sw_frame_) av_frame_unref(sw_frame_);
                    return; 
                }
                bgr_frame_.create(output_height_, output_width_, CV_8UC3);
            }

            if (sws_ctx_ && !bgr_frame_.empty()) {
                uint8_t* dst_data[4] = { bgr_frame_.data, nullptr, nullptr, nullptr };
                int dst_linesize[4] = { static_cast<int>(bgr_frame_.step[0]), 0, 0, 0 };

                sws_scale(sws_ctx_,
                            (const uint8_t* const*)frame_to_display->data, frame_to_display->linesize,
                            0, frame_to_display->height,
                            dst_data, dst_linesize);

                // Apply frame skipping after decoding
                bool should_publish = true;
                
                if (skip_frame_ > 0 && !i_frame_only_) {
                    // Skip frame logic (only when not in i_frame_only mode)
                    should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
                }
                
                if (should_publish) {
                    const int64_t frame_pts = hw_frame_->best_effort_timestamp;
                    if (frame_pts == AV_NOPTS_VALUE) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), 5000,
                            "Decoded frame has no source timestamp; dropping it");
                    } else {
                        std_msgs::msg::Header header;
                        header.stamp = rclcpp::Time(frame_pts, RCL_SYSTEM_TIME);
                        header.frame_id = "camera_frame";

                        PendingFrame pending{bgr_frame_.clone(), std::move(header)};
                        {
                            std::lock_guard<std::mutex> lock(queue_mutex_);
                            latest_frame_ = std::move(pending);
                        }
                    }
                }
            }
            
            av_frame_unref(hw_frame_);
            if (frame_to_display == sw_frame_) {
                av_frame_unref(sw_frame_);
            }
        }
    }

    void CleanupFFmpegDecoder() {
        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }
        if (sw_frame_) {
            av_frame_free(&sw_frame_);
            sw_frame_ = nullptr;
        }
        if (hw_frame_) {
            av_frame_free(&hw_frame_);
            hw_frame_ = nullptr;
        }
        if (pkt_) {
            av_packet_free(&pkt_);
            pkt_ = nullptr;
        }
        if (codec_ctx_) {
            avcodec_close(codec_ctx_); 
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        if (parser_ctx_) {
            av_parser_close(parser_ctx_);
            parser_ctx_ = nullptr;
        }
        if (hw_device_ctx_) {
            av_buffer_unref(&hw_device_ctx_);
            hw_device_ctx_ = nullptr;
        }
        codec_ = nullptr;
    }

    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (msg->format != "h264") {
            return;
        }

        if (!codec_ctx_ || !parser_ctx_ || !pkt_ || !hw_frame_) {
            return;
        }

        const uint8_t* cur_data = msg->data.data();
        size_t remaining_size = msg->data.size();
        const int64_t message_pts = rclcpp::Time(msg->header.stamp).nanoseconds();
        bool first_parser_input = true;

        while (remaining_size > 0) {
            const int64_t parser_pts = first_parser_input ? message_pts : AV_NOPTS_VALUE;
            int bytes_parsed = av_parser_parse2(parser_ctx_, codec_ctx_,
                                                &pkt_->data, &pkt_->size,
                                                cur_data, static_cast<int>(remaining_size),
                                                parser_pts, parser_pts, 0);
            if (bytes_parsed < 0) {
                break; 
            }
            first_parser_input = false;
            cur_data += bytes_parsed;
            remaining_size -= bytes_parsed;

            if (pkt_->size > 0) {
                pkt_->pts = parser_ctx_->pts;
                pkt_->dts = parser_ctx_->dts;

                // Check if this is an I-frame when i_frame_only mode is enabled
                if (i_frame_only_) {
                    // Parse NAL unit type from H.264 stream
                    // The parser sets keyframe flag for I-frames
                    if (parser_ctx_->key_frame == 1) {
                        DecodeAndDisplayPacket(pkt_);
                    }
                } else {
                    DecodeAndDisplayPacket(pkt_);
                }
            }

            if (bytes_parsed == 0 && pkt_->size == 0) {
                break;
            }
        }
    }

public:
    H264DecoderNode() : Node("h264_decoder_node") {
        this->declare_parameter("compressed_topic", "/dual_fisheye/image/compressed");
        this->declare_parameter("uncompressed_topic", "/dual_fisheye/image");
        this->declare_parameter("skip_frame", 0);
        this->declare_parameter("i_frame_only", false);
        this->declare_parameter("publish_rate_hz", 30);
        this->declare_parameter("output_resolution", "1920x960");

        std::string subscribe_topic = this->get_parameter("compressed_topic").as_string();
        std::string publish_topic = this->get_parameter("uncompressed_topic").as_string();
        skip_frame_ = this->get_parameter("skip_frame").as_int();
        i_frame_only_ = this->get_parameter("i_frame_only").as_bool();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_int();
        const auto output_resolution = this->get_parameter("output_resolution").as_string();

        if (output_resolution == "1920x960") {
            output_width_ = 1920;
            output_height_ = 960;
        } else if (output_resolution == "3840x1920") {
            output_width_ = 3840;
            output_height_ = 1920;
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Unsupported output_resolution '%s'; using 1920x960",
                output_resolution.c_str());
            output_width_ = 1920;
            output_height_ = 960;
        }

        if (publish_rate_hz_ != 10 && publish_rate_hz_ != 30) {
            RCLCPP_WARN(
                this->get_logger(),
                "publish_rate_hz must be 10 or 30; using 30");
            publish_rate_hz_ = 30;
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            subscribe_topic, 10,
            std::bind(&H264DecoderNode::compressed_image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            publish_topic, rclcpp::QoS(1).reliable());

        publisher_thread_ = std::thread(&H264DecoderNode::PublisherThreadLoop, this);
        
        InitFFmpegDecoder();

        RCLCPP_INFO(this->get_logger(), "H.264 Decoder Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", subscribe_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", publish_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Skip frame: %d, I-frame only: %s", skip_frame_, i_frame_only_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Latest-frame publish rate: %d Hz", publish_rate_hz_);
        RCLCPP_INFO(this->get_logger(), "Decoded output resolution: %dx%d", output_width_, output_height_);
    }

    ~H264DecoderNode() {
        stop_publisher_thread_ = true;
        queue_cv_.notify_one();
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
        }
        CleanupFFmpegDecoder();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H264DecoderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
