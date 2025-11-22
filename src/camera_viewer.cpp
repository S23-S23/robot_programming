#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <algorithm>

//cv_bridge가 빌드가 안돼요 ㅠ 
class CameraViewer : public rclcpp::Node
{
public:
    CameraViewer() : Node("camera_viewer")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CameraViewer::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_flip", 10);

        RCLCPP_INFO(this->get_logger(), "Camera viewer node started - subscribing to /camera/image_raw and publishing to /camera/image_flip");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto flipped_msg = std::make_shared<sensor_msgs::msg::Image>();

        flipped_msg->header = msg->header;
        flipped_msg->height = msg->height;
        flipped_msg->width = msg->width;
        flipped_msg->encoding = msg->encoding;
        flipped_msg->is_bigendian = msg->is_bigendian;
        flipped_msg->step = msg->step;

        flipped_msg->data.resize(msg->data.size());

        for (uint32_t row = 0; row < msg->height; ++row)
        {
            uint32_t flipped_row = msg->height - 1 - row;

            for (uint32_t col = 0; col < msg->width; ++col)
            {
                uint32_t flipped_col = msg->width - 1 - col;

                uint32_t bytes_per_pixel = msg->step / msg->width;
                uint32_t src_idx = row * msg->step + col * bytes_per_pixel;
                uint32_t dst_idx = flipped_row * msg->step + flipped_col * bytes_per_pixel;

                for (uint32_t b = 0; b < bytes_per_pixel; ++b)
                {
                    flipped_msg->data[dst_idx + b] = msg->data[src_idx + b];
                }
            }
        }

        publisher_->publish(*flipped_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraViewer>());
    rclcpp::shutdown();
    return 0;
}
