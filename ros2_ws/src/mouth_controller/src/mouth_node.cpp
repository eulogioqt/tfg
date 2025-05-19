#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>

class MouthNode : public rclcpp::Node
{
public:
    MouthNode() : Node("mouth_node")
    {
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MouthNode::on_timer, this));

        rng_ = std::mt19937(rd_());
        dist_ = std::uniform_int_distribution<int16_t>(-32768, 32767);
    }

private:
    void on_timer()
    {
        const int SAMPLE_RATE = 48000;
        const int SAMPLES = SAMPLE_RATE / 10; // 100ms

        std::vector<int16_t> samples(SAMPLES);

        double time = this->get_clock()->now().seconds();
        double amplitude = 16384.0 + 16384.0 * std::sin(2 * M_PI * 0.5 * time);

        for (int i = 0; i < SAMPLES; ++i)
        {
            double t = static_cast<double>(i) / SAMPLE_RATE;
            samples[i] = static_cast<int16_t>(amplitude * std::sin(2 * M_PI * 440.0 * t));
        }

        handle_chunk(samples);
    }

    void handle_chunk(const std::vector<int16_t> &samples)
    {
        if (samples.empty())
            return;

        double sum_sq = 0.0;
        for (auto s : samples)
            sum_sq += static_cast<double>(s) * s;

        double rms = std::sqrt(sum_sq / samples.size());
        double normalized = std::clamp(rms / 32768.0, 0.0, 1.0);

        int level = static_cast<int>(normalized * 4.999); // 0, 1, 2, 3, 4

        static const char *LEVEL_NAMES[] = {
            "low", "medium-low", "medium", "medium_high", "high"};

        const char *level_str = LEVEL_NAMES[level];

        RCLCPP_INFO(get_logger(), "Enviando a la boca nivel de intensidad: %s", level_str);
        send_to_esp32(level_str);
    }

    void send_to_esp32(const std::string &level)
    {
        std::ofstream serial("/dev/ttyUSB1");
        if (!serial.is_open())
        {
            RCLCPP_ERROR(get_logger(), "No se pudo abrir /dev/ttyUSB1");
            return;
        }
        serial << level << "\n";
        serial.flush();
        serial.close();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd_;
    std::mt19937 rng_;
    std::uniform_int_distribution<int16_t> dist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouthNode>());
    rclcpp::shutdown();
    return 0;
}
