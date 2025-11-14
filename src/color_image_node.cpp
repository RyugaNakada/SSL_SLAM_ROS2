// カラー画像取得・保存ノード
// ROS2 port by: ry223

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

class ColorImageNode : public rclcpp::Node
{
public:
    ColorImageNode() : Node("color_image_node"), save_counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Color Image Node Started");

        // パラメータ宣言
        this->declare_parameter<bool>("save_images", false);
        this->declare_parameter<std::string>("save_directory", "/tmp/ssl_slam_images");
        this->declare_parameter<int>("save_interval", 10);  // フレームごとに保存

        save_images_ = this->get_parameter("save_images").as_bool();
        save_directory_ = this->get_parameter("save_directory").as_string();
        save_interval_ = this->get_parameter("save_interval").as_int();

        RCLCPP_INFO(this->get_logger(), "Save images: %s", save_images_ ? "enabled" : "disabled");
        if (save_images_)
        {
            RCLCPP_INFO(this->get_logger(), "Save directory: %s", save_directory_.c_str());
            RCLCPP_INFO(this->get_logger(), "Save interval: %d frames", save_interval_);
        }

        // Subscriber作成
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&ColorImageNode::image_callback, this, std::placeholders::_1));

        // Publisher作成（リレー用）
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/ssl_slam/color_image", 10);

        RCLCPP_INFO(this->get_logger(), "Color Image Node initialization complete");
    }

private:
    // ROS2通信
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    // パラメータ
    bool save_images_;
    std::string save_directory_;
    int save_interval_;
    int save_counter_;

    // 画像コールバック
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 画像をリレー
        image_publisher_->publish(*msg);

        // 画像保存が有効な場合
        if (save_images_ && save_counter_ % save_interval_ == 0)
        {
            try
            {
                // ROS画像をOpenCV形式に変換
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

                // ファイル名生成（タイムスタンプ使用）
                auto timestamp = rclcpp::Time(msg->header.stamp).seconds();
                std::string filename = save_directory_ + "/image_" +
                                     std::to_string(static_cast<long>(timestamp * 1000)) + ".png";

                // 画像保存
                cv::imwrite(filename, cv_ptr->image);

                RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
            }
            catch (cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        save_counter_++;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ColorImageNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
