#include "reset_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace mini_rviz_plugins
{

ResetPanel::ResetPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  reset_button_ = new QPushButton(QStringLiteral("RESET BASE"), this);
  reset_button_->setToolTip(QStringLiteral(
    "E-stop, clear it, then zero the odometry.\n"
    "Does NOT clear the SLAM map or stop a duplicate stack."));
  reset_button_->setMinimumHeight(36);
  reset_button_->setStyleSheet(QStringLiteral("font-weight: bold;"));

  estop_button_ = new QPushButton(QStringLiteral("E-STOP"), this);
  estop_button_->setToolTip(QStringLiteral(
    "Latch the emergency stop. The base brakes and refuses velocity\n"
    "commands until it is cleared."));
  estop_button_->setMinimumHeight(36);
  estop_button_->setStyleSheet(QStringLiteral("font-weight: bold; color: #b00;"));

  clear_estop_button_ = new QPushButton(QStringLiteral("Clear E-Stop"), this);
  reset_odom_button_ = new QPushButton(QStringLiteral("Reset Odometry"), this);
  zero_yaw_button_ = new QPushButton(QStringLiteral("Zero Yaw"), this);
  calibrate_imu_button_ = new QPushButton(QStringLiteral("Calibrate IMU"), this);
  calibrate_imu_button_->setToolTip(QStringLiteral(
    "Re-measure the gyro zero-rate offset. Keep the robot completely still.\n"
    "Worth doing once the board has been powered a few minutes: the\n"
    "calibration it does at boot runs before the chip has warmed up and\n"
    "leaves several times more drift."));

  status_label_ = new QLabel(QStringLiteral("Waiting for the base..."), this);
  status_label_->setWordWrap(true);

  auto * top_row = new QHBoxLayout();
  top_row->addWidget(reset_button_, 1);
  top_row->addWidget(estop_button_, 1);

  auto * grid = new QHBoxLayout();
  grid->addWidget(clear_estop_button_);
  grid->addWidget(reset_odom_button_);

  auto * grid2 = new QHBoxLayout();
  grid2->addWidget(zero_yaw_button_);
  grid2->addWidget(calibrate_imu_button_);

  auto * layout = new QVBoxLayout();
  layout->addLayout(top_row);
  layout->addLayout(grid);
  layout->addLayout(grid2);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(reset_button_, &QPushButton::clicked, this, &ResetPanel::resetAll);
  connect(estop_button_, &QPushButton::clicked, this, &ResetPanel::emergencyStop);
  connect(clear_estop_button_, &QPushButton::clicked, this, &ResetPanel::clearEstop);
  connect(reset_odom_button_, &QPushButton::clicked, this, &ResetPanel::resetOdometry);
  connect(zero_yaw_button_, &QPushButton::clicked, this, &ResetPanel::zeroYaw);
  connect(calibrate_imu_button_, &QPushButton::clicked, this, &ResetPanel::calibrateImu);
}

void ResetPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  estop_client_ = clientFor("/base/emergency_stop");
  clear_estop_client_ = clientFor("/base/clear_emergency_stop");
  reset_odom_client_ = clientFor("/base/reset_odometry");
  zero_yaw_client_ = clientFor("/base/zero_yaw");
  calibrate_imu_client_ = clientFor("/base/calibrate_imu");
  setStatus(QStringLiteral("Ready."));
}

rclcpp::Client<ResetPanel::Trigger>::SharedPtr ResetPanel::clientFor(const std::string & service)
{
  return node_->create_client<Trigger>(service);
}

void ResetPanel::setStatus(const QString & text)
{
  // Service responses arrive on RViz's ROS executor thread, so every widget
  // touch goes through the Qt event loop.
  QMetaObject::invokeMethod(
    status_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void ResetPanel::setButtonsEnabled(bool enabled)
{
  for (QPushButton * button : {reset_button_, estop_button_, clear_estop_button_,
                               reset_odom_button_, zero_yaw_button_, calibrate_imu_button_})
  {
    QMetaObject::invokeMethod(button, "setEnabled", Qt::QueuedConnection, Q_ARG(bool, enabled));
  }
}

void ResetPanel::runSequence(std::vector<Step> steps)
{
  if (!pending_.empty()) {
    setStatus(QStringLiteral("Still working on the previous request."));
    return;
  }
  pending_.assign(steps.begin(), steps.end());
  setButtonsEnabled(false);
  startNextStep();
}

void ResetPanel::startNextStep()
{
  if (pending_.empty()) {
    setButtonsEnabled(true);
    return;
  }
  const Step step = pending_.front();

  if (!step.client || !step.client->service_is_ready()) {
    // The bridge is not running, or not running yet. Say which service is
    // missing rather than leaving the panel looking hung.
    setStatus(QStringLiteral("%1: the base is not connected "
                             "(is robot.launch.py running?)").arg(step.label));
    pending_.clear();
    setButtonsEnabled(true);
    return;
  }

  setStatus(QStringLiteral("%1...").arg(step.label));
  auto request = std::make_shared<Trigger::Request>();
  step.client->async_send_request(
    request,
    [this, step](rclcpp::Client<Trigger>::SharedFuture future) {
      QString text;
      try {
        const auto response = future.get();
        text = response->success
          ? QStringLiteral("%1: ok").arg(step.label)
          : QStringLiteral("%1: FAILED - %2").arg(step.label,
              QString::fromStdString(response->message));
        if (!response->success) {
          pending_.clear();
        }
      } catch (const std::exception & exc) {
        text = QStringLiteral("%1: error - %2").arg(step.label, QString::fromUtf8(exc.what()));
        pending_.clear();
      }
      setStatus(text);
      if (!pending_.empty()) {
        pending_.pop_front();
      }
      if (pending_.empty()) {
        setButtonsEnabled(true);
      } else {
        startNextStep();
      }
    });
}

void ResetPanel::resetAll()
{
  runSequence({
    {QStringLiteral("E-stop"), estop_client_},
    {QStringLiteral("Clear e-stop"), clear_estop_client_},
    {QStringLiteral("Reset odometry"), reset_odom_client_},
  });
}

void ResetPanel::emergencyStop()
{
  runSequence({{QStringLiteral("E-stop"), estop_client_}});
}

void ResetPanel::clearEstop()
{
  runSequence({{QStringLiteral("Clear e-stop"), clear_estop_client_}});
}

void ResetPanel::resetOdometry()
{
  runSequence({{QStringLiteral("Reset odometry"), reset_odom_client_}});
}

void ResetPanel::zeroYaw()
{
  runSequence({{QStringLiteral("Zero yaw"), zero_yaw_client_}});
}

void ResetPanel::calibrateImu()
{
  runSequence({{QStringLiteral("Calibrate IMU (keep still)"), calibrate_imu_client_}});
}

}  // namespace mini_rviz_plugins

PLUGINLIB_EXPORT_CLASS(mini_rviz_plugins::ResetPanel, rviz_common::Panel)
