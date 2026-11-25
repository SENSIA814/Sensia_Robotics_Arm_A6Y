#pragma once
#include <chrono>
#include <QJsonObject>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
class QLabel;
class QCheckBox;
class QPushButton;
class QSlider;
class QSpinBox;
class QTextBrowser;
namespace a6y_rviz_panel {
class PlanningPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit PlanningPanel(QWidget * parent = nullptr);
  void onInitialize() override;
protected:
  bool eventFilter(QObject * watched, QEvent * event) override;
private:
  friend class PlanningPanelCloseTest;
  void send(const QString & command, const QJsonValue & value = QJsonValue());
  void refresh(const QJsonObject & state);
  QJsonObject settings() const;
  QLabel * identity_;
  QLabel * execution_hint_;
  QTextBrowser * status_;
  QLabel * feedback_;
  QLabel * opening_label_;
  QCheckBox * permit_;
  QSpinBox * speed_;
  QSpinBox * acceleration_;
  QCheckBox * position_only_;
  QSlider * opening_;
  QPushButton * plan_;
  QPushButton * execute_;
  QPushButton * reset_;
  QPushButton * grip_plan_;
  QPushButton * grip_execute_;
  QPushButton * open_;
  QPushButton * close_;
  QPushButton * stop_;
  QPushButton * release_;
  QPushButton * emergency_;
  QPushButton * detached_close_;
  bool close_allowed_ = false;
  bool close_requested_ = false;
  bool display_only_close_ = false;
  std::chrono::steady_clock::time_point received_ = std::chrono::steady_clock::now();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}
