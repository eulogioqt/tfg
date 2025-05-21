#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <memory>
#include <portaudio.h>
#include <thread>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>

class FaceNode : public rclcpp::Node
{
public:
    FaceNode() : Node("face_node")
    {
        declare_parameter<double>("send_interval_sec", 0.1);
        get_parameter("send_interval_sec", chunk_send_interval_);
        RCLCPP_INFO(get_logger(), "Intervalo de env\u00edo configurado: %.2f s", chunk_send_interval_);

        serial_out_ = std::make_unique<std::ofstream>("/dev/ttyUSB1");
        serial_in_fd_ = open("/dev/ttyUSB1", O_RDONLY | O_NONBLOCK);

        if (!serial_out_->is_open() || serial_in_fd_ == -1)
        {
            RCLCPP_ERROR(get_logger(), "No se pudo abrir /dev/ttyUSB1");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Puerto serie /dev/ttyUSB1 abierto correctamente");
            send_to_esp32("idle");

            serial_thread_ = std::thread([this]()
                                         {
                std::string buffer;
                char c;
                while (rclcpp::ok())
                {
                    ssize_t len = read(serial_in_fd_, &c, 1);
                    if (len > 0)
                    {
                        if (c == '0') {
                            RCLCPP_INFO(get_logger(), "%s", buffer.c_str());
                            buffer.clear();
                        } else {
                            buffer += c;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                } });
        }

        mode_subscription_ = create_subscription<std_msgs::msg::String>(
            "face/mode", 10,
            std::bind(&FaceNode::mode_callback, this, std::placeholders::_1));

        start_audio_monitoring();
    }

    ~FaceNode()
    {
        running_ = false;
        if (audio_thread_.joinable())
            audio_thread_.join();

        if (serial_thread_.joinable())
            serial_thread_.join();

        if (serial_out_ && serial_out_->is_open())
            serial_out_->close();

        if (serial_in_fd_ != -1)
            close(serial_in_fd_);
    }

private:
    void mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        static const std::vector<std::string> valid_modes = {"idle", "listening", "thinking", "speaking"};
        static const std::vector<std::string> valid_emotions = {"happy", "surprised", "sad", "angry", "bored", "suspicious", "neutral"};

        if (std::find(valid_modes.begin(), valid_modes.end(), msg->data) != valid_modes.end())
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            current_mode_ = msg->data;
            RCLCPP_INFO(get_logger(), "Modo actualizado a: %s", current_mode_.c_str());
            for (int i = 0; i < 20; i++) // mirar esto y subir baudios
                send_to_esp32(current_mode_);
        }
        else if (std::find(valid_emotions.begin(), valid_emotions.end(), msg->data) != valid_emotions.end())
        {
            RCLCPP_INFO(get_logger(), "Emoción recibida: %s", msg->data.c_str());
            for (int i = 0; i < 20; i++) // mirar esto y subir baudios
                send_to_esp32(msg->data);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Mensaje recibido no reconocido: %s", msg->data.c_str());
        }
    }

    void start_audio_monitoring()
    {
        Pa_Initialize();

        int pulse_device = -1;
        int numDevices = Pa_GetDeviceCount();
        for (int i = 0; i < numDevices; ++i)
        {
            const PaDeviceInfo *info = Pa_GetDeviceInfo(i);
            if (info && std::string(info->name).find("pulse") != std::string::npos)
            {
                RCLCPP_INFO(get_logger(), "Usando dispositivo PortAudio: %s", info->name);
                pulse_device = i;
                break;
            }
        }

        if (pulse_device == -1)
        {
            RCLCPP_ERROR(get_logger(), "No se encontr\u00f3 el dispositivo 'pulse' en PortAudio.");
            return;
        }

        const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(pulse_device);
        sampleRate_ = deviceInfo->defaultSampleRate;
        size_t frames_per_chunk = 1024;

        PaStream *stream;
        PaStreamParameters inputParams;
        inputParams.device = pulse_device;
        inputParams.channelCount = 1;
        inputParams.sampleFormat = paInt16;
        inputParams.suggestedLatency = deviceInfo->defaultLowInputLatency;
        inputParams.hostApiSpecificStreamInfo = nullptr;

        PaError err = Pa_OpenStream(
            &stream,
            &inputParams,
            nullptr,
            sampleRate_,
            frames_per_chunk,
            paClipOff,
            nullptr,
            nullptr);
        if (err != paNoError)
        {
            RCLCPP_ERROR(get_logger(), "No se pudo abrir el stream de audio: %s", Pa_GetErrorText(err));
            return;
        }

        Pa_StartStream(stream);
        RCLCPP_INFO(get_logger(), "Captura de audio iniciada con el dispositivo 'pulse'.");

        audio_thread_ = std::thread([this, stream, frames_per_chunk]()
                                    {
            std::vector<int16_t> buffer(frames_per_chunk);
            auto chunk_start = std::chrono::steady_clock::now();
            std::vector<int16_t> current_chunk;

            while (rclcpp::ok() && running_)
            {
                std::string mode;
                {
                    std::lock_guard<std::mutex> lock(mode_mutex_);
                    mode = current_mode_;
                }
                if (mode != "speaking")
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }

                Pa_ReadStream(stream, buffer.data(), frames_per_chunk);
                current_chunk.insert(current_chunk.end(), buffer.begin(), buffer.end());

                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - chunk_start).count();

                if (elapsed >= chunk_send_interval_)
                {
                    handle_chunk(current_chunk);
                    current_chunk.clear();
                    chunk_start = now;
                }
            }

            Pa_StopStream(stream);
            Pa_CloseStream(stream);
            Pa_Terminate(); });
    }

    void handle_chunk(const std::vector<int16_t> &samples)
    {
        if (samples.empty())
            return;

        double sum_sq = 0.0;
        for (int16_t s : samples)
            sum_sq += static_cast<double>(s) * s;

        double rms = std::sqrt(sum_sq / samples.size());

        if (min_rms_ == -1.0 || rms < min_rms_)
            min_rms_ = rms;

        if (rms > max_rms_)
            max_rms_ = rms;

        auto now = std::chrono::steady_clock::now();
        double seconds_since_last_send = std::chrono::duration<double>(now - last_send_time_).count();

        if (seconds_since_last_send > 3.0)
        {
            RCLCPP_INFO(get_logger(), "3 segundos de silencio detectados. Reiniciando min y max rms.");
            max_rms_ = 0.0;
            min_rms_ = -1.0;
            last_send_time_ = now;
        }

        double normalized = (max_rms_ > 0.0 && min_rms_ >= 0.0) ? std::clamp((rms - min_rms_) / (max_rms_ - min_rms_), 0.0, 1.0) : 0.0;

        const int num_levels = 6;
        int level = static_cast<int>(normalized * (num_levels - 1) + 0.5);
        level = std::clamp(level, 0, num_levels - 1);

        static const char *LEVEL_NAMES[] = {
            "low", "medium_low", "medium", "medium_high", "high", "full"};

        const char *level_str = LEVEL_NAMES[level];
        RCLCPP_INFO(get_logger(), "RMS: %.2f | Norm: %.2f | Nivel: %s", rms, normalized, level_str);

        send_to_esp32(level_str);
        last_send_time_ = now;
    }

    void send_to_esp32(const std::string &level)
    {
        if (serial_out_ && serial_out_->is_open())
        {
            *serial_out_ << level << "0";
            serial_out_->flush();
        }
    }

    std::unique_ptr<std::ofstream> serial_out_;
    int serial_in_fd_ = -1;
    std::thread serial_thread_;
    std::thread audio_thread_;
    bool running_ = true;

    double sampleRate_ = 48000;
    double chunk_send_interval_ = 0.1;
    double min_rms_ = -1.0;
    double max_rms_ = 0.0;

    std::chrono::steady_clock::time_point last_send_time_ = std::chrono::steady_clock::now();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
    std::string current_mode_ = "idle";
    std::mutex mode_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceNode>());
    rclcpp::shutdown();
    return 0;
}