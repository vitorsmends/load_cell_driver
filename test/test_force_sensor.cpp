#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "load_cell_driver/force_sensor.hpp"

using namespace load_cell_driver;

class ForceSensorTest : public ::testing::Test
{
protected:
  std::shared_ptr<ForceSensorNode> node_;

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("enable_serial", false);
    node_ = std::make_shared<ForceSensorNode>(rclcpp::NodeOptions());
  }

  void TearDown() override
  {
    node_.reset();
  }
};

TEST_F(ForceSensorTest, ExtractPacketValidData)
{
  std::string packet = "id:0,force:1.5;id:1,force:2.5";
  auto result = node_->extractPacket(packet);

  ASSERT_EQ(result.size(), 2);
  EXPECT_EQ(result[0].first, 0);
  EXPECT_FLOAT_EQ(result[0].second, 1.5f);
  EXPECT_EQ(result[1].first, 1);
  EXPECT_FLOAT_EQ(result[1].second, 2.5f);
}

TEST_F(ForceSensorTest, ExtractPacketInvalidData)
{
  std::string packet = "id:x,force:abc;id:2,force:4.4";
  auto result = node_->extractPacket(packet);

  ASSERT_EQ(result.size(), 1);
  EXPECT_EQ(result[0].first, 2);
  EXPECT_FLOAT_EQ(result[0].second, 4.4f);
}
