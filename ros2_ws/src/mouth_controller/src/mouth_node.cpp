#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <memory>
#include <portaudio.h>
#include <thread>
#include <fftw3.h>
#include <chrono>
#include <cstdint>


class MouthNode : public rclcpp::Node
{
public:
    MouthNode() : Node("mouth_node")
    {
        declare_parameter<double>("send_interval_sec", 0.1);  // valor por defecto
        get_parameter("send_interval_sec", chunk_send_interval_);
        RCLCPP_INFO(get_logger(), "Intervalo de envío configurado: %.2f s", chunk_send_interval_);

        serial_ = std::make_unique<std::ofstream>("/dev/ttyUSB1");
        if (!serial_->is_open())
        {
            RCLCPP_ERROR(get_logger(), "No se pudo abrir /dev/ttyUSB1");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Puerto serie /dev/ttyUSB1 abierto correctamente");
            send_to_esp32("color_white");
        }

        start_audio_monitoring();
    }

    ~MouthNode()
    {
        running_ = false;
        if (audio_thread_.joinable())
            audio_thread_.join();

        if (serial_ && serial_->is_open())
            serial_->close();
    }

private:
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
            RCLCPP_ERROR(get_logger(), "No se encontró el dispositivo 'pulse' en PortAudio.");
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
            nullptr
        );
        if (err != paNoError)
        {
            RCLCPP_ERROR(get_logger(), "No se pudo abrir el stream de audio: %s", Pa_GetErrorText(err));
            return;
        }

        Pa_StartStream(stream);
        RCLCPP_INFO(get_logger(), "Captura de audio iniciada con el dispositivo 'pulse'.");

        audio_thread_ = std::thread([this, stream, frames_per_chunk]() {
            std::vector<int16_t> buffer(frames_per_chunk);
            while (rclcpp::ok() && running_)
            {
                Pa_ReadStream(stream, buffer.data(), frames_per_chunk);
                accumulated_samples_.insert(accumulated_samples_.end(), buffer.begin(), buffer.end());

                size_t target_samples = static_cast<size_t>(chunk_send_interval_ * sampleRate_);
                if (accumulated_samples_.size() >= target_samples)
                {
                    std::vector<int16_t> chunk(accumulated_samples_.begin(), accumulated_samples_.begin() + target_samples);
                    handle_chunk(chunk, sampleRate_);
                    accumulated_samples_.erase(accumulated_samples_.begin(), accumulated_samples_.begin() + target_samples);
                }
            }

            Pa_StopStream(stream);
            Pa_CloseStream(stream);
            Pa_Terminate();
        });
    }

    void handle_chunk(const std::vector<int16_t> &samples, double sampleRate)
    {
        if (samples.empty())
            return;

        // Calcular RMS directamente desde la señal
        double sum_sq = 0.0;
        for (int16_t s : samples)
            sum_sq += static_cast<double>(s) * s;

        double rms = std::sqrt(sum_sq / samples.size());

        // Normalizar entre 0 y 32768 (máximo valor int16_t posible en módulo)
        double normalized = std::clamp(rms / 32768.0, 0.0, 1.0);

        // Escalar a N niveles (en este caso 5 niveles: 0 a 4)
        const int num_levels = 5;
        int level = static_cast<int>(normalized * (num_levels - 1) + 0.5);

        // Asegurar que no se sale del rango
        level = std::clamp(level, 0, num_levels - 1);

        static const char *LEVEL_NAMES[] = {
            "low", "medium-low", "medium", "medium-high", "high"
        };

        const char *level_str = LEVEL_NAMES[level];

        RCLCPP_INFO(get_logger(), "RMS: %.2f | Nivel: %s", rms, level_str);

        send_to_esp32(level_str);
    }


    void send_to_esp32(const std::string &level)
    {
        if (serial_ && serial_->is_open())
        {
            *serial_ << level << "\n";
            serial_->flush();
        }
    }

    std::unique_ptr<std::ofstream> serial_;
    std::thread audio_thread_;
    bool running_ = true;

    double sampleRate_ = 48000;
    double chunk_send_interval_ = 0.5;
    std::vector<int16_t> accumulated_samples_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouthNode>());
    rclcpp::shutdown();
    return 0;
}
