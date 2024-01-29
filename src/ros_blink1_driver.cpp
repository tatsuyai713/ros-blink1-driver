#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "blink1-lib.h"

class Blink1Driver : public rclcpp::Node
{
public:
    Blink1Driver() : Node("ros_blink1_driver_node")
    {
        // Initialize parameters
        fade_time_ = 1.0;
        brightness_ = 0;
        millis_ = 100;
        ledn_ = 0;
        numDevicesToUse_ = 0;

        // Get parameters from launch file
        this->declare_parameter("fade_time", rclcpp::ParameterValue(1.0));
        fade_time_ = this->get_parameter("fade_time").as_double();
        millis_ = static_cast<int>(fade_time_ * 1000); // Convert fade_time to milliseconds

        // Set up subscription
        subscription_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
            "blink1_color", 10, std::bind(&Blink1Driver::topicCallback, this, std::placeholders::_1));

        // Enumerate blink(1) devices
        count_ = blink1_enumerate();
        if (count_ == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "No blink(1) devices found");
            rclcpp::shutdown();
            return;
        }
        numDevicesToUse_ = count_;

        dev_ = blink1_openById(deviceIds_[0]);
        if (dev_ == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open blink(1), bad id or serial number");
            rclcpp::shutdown();
            return;
        }
    }

private:
    // Member variables
    blink1_device *dev_;
    uint32_t deviceIds_[blink1_max_devices];
    int count_;
    int numDevicesToUse_;
    int ledn_;
    int brightness_;
    int millis_;
    double fade_time_;

    // Subscription callback
    void topicCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
    {
        uint8_t r = msg->r;
        uint8_t g = msg->g;
        uint8_t b = msg->b;

        blink1_adjustBrightness(brightness_, &r, &g, &b);
        RCLCPP_INFO(this->get_logger(), "Received color: %d %d %d", r, g, b);

        blink1_fadeToRGBForDevices(millis_, r, g, b, ledn_);
    }

    // Function to fade to RGB for devices
    int blink1_fadeToRGBForDevices(uint16_t mils, uint8_t rr, uint8_t gg, uint8_t bb, uint8_t nn)
    {
        blink1_device *d;
        int rc;
        for (int i = 0; i < numDevicesToUse_; i++)
        {
            d = blink1_openById(deviceIds_[i]);
            if (d == NULL)
                continue;

            if (nn == 0)
            {
                rc = blink1_fadeToRGB(d, mils, rr, gg, bb);
            }
            else
            {
                rc = blink1_fadeToRGBN(d, mils, rr, gg, bb, nn);
            }

            if (rc == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Error on fadeToRGBForDevices");
            }

            blink1_close(d);
        }
        return 0; // FIXME
    }

    // Subscription
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Blink1Driver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
