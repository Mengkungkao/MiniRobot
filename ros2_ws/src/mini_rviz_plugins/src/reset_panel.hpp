// RViz panel for putting the base back to a known state.
//
// One Reset button runs the sequence you almost always want -- latch the
// emergency stop, clear it, zero the odometry -- plus individual buttons for
// the pieces. Everything here is a std_srvs/Trigger the serial bridge exposes.
//
// The reset starts by latching the e-stop rather than just sending a zero
// velocity. The bridge stops streaming VEL while latched and the base brakes,
// so motion has provably ceased before the odometry origin is moved underneath
// it. Clearing it again is the next step, so the robot ends up ready to drive.
//
// What this panel deliberately CANNOT do:
//   - clear the SLAM map. slam_toolbox (Humble) has no reset service; its Clear
//     service only drops manual pose-graph edits. Discarding a map means
//     restarting the node.
//   - stop a duplicate stack. Two slam_toolbox or Nav2 instances are processes,
//     not services, and no button can see them. Use
//     scripts/reset_robot_stack.sh for that.
#pragma once

#include <deque>
#include <vector>

#include <QLabel>
#include <QPushButton>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mini_rviz_plugins
{

class ResetPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ResetPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void resetAll();
  void emergencyStop();
  void clearEstop();
  void resetOdometry();
  void zeroYaw();
  void calibrateImu();

private:
  using Trigger = std_srvs::srv::Trigger;

  struct Step
  {
    QString label;                                  // shown while it runs
    rclcpp::Client<Trigger>::SharedPtr client;
  };

  rclcpp::Client<Trigger>::SharedPtr clientFor(const std::string & service);
  // Runs the steps one after another, each starting when the previous replies.
  // Sequencing through callbacks rather than waiting keeps the Qt event loop
  // free; blocking here would freeze RViz, and spinning the node from this
  // thread would fight RViz's own executor.
  void runSequence(std::vector<Step> steps);
  void startNextStep();
  void setStatus(const QString & text);
  void setButtonsEnabled(bool enabled);

  QPushButton * reset_button_ = nullptr;
  QPushButton * estop_button_ = nullptr;
  QPushButton * clear_estop_button_ = nullptr;
  QPushButton * reset_odom_button_ = nullptr;
  QPushButton * zero_yaw_button_ = nullptr;
  QPushButton * calibrate_imu_button_ = nullptr;
  QLabel * status_label_ = nullptr;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr estop_client_;
  rclcpp::Client<Trigger>::SharedPtr clear_estop_client_;
  rclcpp::Client<Trigger>::SharedPtr reset_odom_client_;
  rclcpp::Client<Trigger>::SharedPtr zero_yaw_client_;
  rclcpp::Client<Trigger>::SharedPtr calibrate_imu_client_;

  std::deque<Step> pending_;
};

}  // namespace mini_rviz_plugins
