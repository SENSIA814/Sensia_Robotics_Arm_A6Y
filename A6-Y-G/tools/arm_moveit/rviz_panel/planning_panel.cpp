#include "planning_panel.hpp"
#include <QCheckBox>
#include <QCloseEvent>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QTextBrowser>
#include <QVBoxLayout>
#include <cerrno>
#include <csignal>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace a6y_rviz_panel {
PlanningPanel::PlanningPanel(QWidget * parent) : rviz_common::Panel(parent) {
  setMinimumWidth(320);
  auto layout = new QVBoxLayout(this);
  identity_ = new QLabel("等待所选机械臂会话…", this);
  identity_->setWordWrap(true);
  identity_->setStyleSheet("font-size:16px;font-weight:bold;");
  layout->addWidget(identity_);
  permit_ = new QCheckBox("已核对零位与方向，允许真机执行", this);
  layout->addWidget(permit_);
  connect(permit_, &QCheckBox::toggled, this, [this](bool value) {send("permit", value);});
  execution_hint_ = new QLabel(this);
  execution_hint_->setWordWrap(true);
  layout->addWidget(execution_hint_);
  auto speed_row = new QHBoxLayout();
  speed_row->addWidget(new QLabel("速度", this));
  speed_ = new QSpinBox(this);
  speed_->setRange(1, 100); speed_->setValue(10); speed_->setSuffix(" %");
  speed_row->addWidget(speed_);
  speed_row->addWidget(new QLabel("加速度", this));
  acceleration_ = new QSpinBox(this);
  acceleration_->setRange(1, 100); acceleration_->setValue(10); acceleration_->setSuffix(" %");
  speed_row->addWidget(acceleration_);
  layout->addLayout(speed_row);
  position_only_ = new QCheckBox("仅位置（可选；关闭 RPY 姿态控制）", this);
  position_only_->setChecked(false);
  layout->addWidget(position_only_);
  connect(speed_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) {send("settings", settings());});
  connect(acceleration_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) {send("settings", settings());});
  connect(position_only_, &QCheckBox::toggled, this, [this](bool) {send("settings", settings());});
  auto arm = new QGroupBox("末端位姿 · MoveIt", this);
  auto arm_layout = new QVBoxLayout(arm);
  auto hint = new QLabel("Interact：箭头拖 XYZ，圆环拖 RPY。\n红环 Roll · 绿环 Pitch · 蓝环 Yaw。\n绿色模型跟随；红球不可达。", arm);
  hint->setWordWrap(true); arm_layout->addWidget(hint);
  reset_ = new QPushButton("目标复位到当前末端", arm);
  arm_layout->addWidget(reset_);
  connect(reset_, &QPushButton::clicked, this, [this] {send("reset");});
  auto arm_row = new QHBoxLayout();
  plan_ = new QPushButton("① 规划 Plan", arm);
  execute_ = new QPushButton("② 执行 Execute", arm);
  arm_row->addWidget(plan_); arm_row->addWidget(execute_); arm_layout->addLayout(arm_row);
  connect(plan_, &QPushButton::clicked, this, [this] {send("plan", settings());});
  connect(execute_, &QPushButton::clicked, this, [this] {send("execute_arm");});
  layout->addWidget(arm);
  auto grip = new QGroupBox("夹爪开合 · MoveIt", this);
  auto grip_layout = new QVBoxLayout(grip);
  opening_label_ = new QLabel("目标开度：等待反馈", grip);
  grip_layout->addWidget(opening_label_);
  opening_ = new QSlider(Qt::Horizontal, grip); opening_->setRange(0, 100);
  grip_layout->addWidget(opening_);
  // Dragging changes only the preview; never send a trajectory or enable here.
  connect(opening_, &QSlider::valueChanged, this, [this](int value) {
    opening_label_->setText(QString("目标开度：%1%（电子限位内行程）").arg(value));
  });
  connect(opening_, &QSlider::sliderReleased, this, [this] {send("gripper_target", opening_->value());});
  // Keyboard and groove clicks also update the preview, without flooding ROS.
  connect(opening_, &QSlider::valueChanged, this, [this](int value) {
    if (!opening_->isSliderDown()) {send("gripper_target", value);}
  });
  auto preset = new QHBoxLayout();
  close_ = new QPushButton("闭合目标 0%", grip);
  open_ = new QPushButton("张开目标 100%", grip);
  preset->addWidget(close_); preset->addWidget(open_); grip_layout->addLayout(preset);
  connect(close_, &QPushButton::clicked, this, [this] {send("gripper_target", 0);});
  connect(open_, &QPushButton::clicked, this, [this] {send("gripper_target", 100);});
  feedback_ = new QLabel("夹爪反馈：—", grip); feedback_->setWordWrap(true);
  grip_layout->addWidget(feedback_);
  auto grip_row = new QHBoxLayout();
  grip_plan_ = new QPushButton("① 夹爪规划", grip);
  grip_execute_ = new QPushButton("② 执行夹爪", grip);
  grip_row->addWidget(grip_plan_); grip_row->addWidget(grip_execute_); grip_layout->addLayout(grip_row);
  connect(grip_plan_, &QPushButton::clicked, this, [this] {send("gripper_plan", settings());});
  connect(grip_execute_, &QPushButton::clicked, this, [this] {send("execute_gripper");});
  layout->addWidget(grip);
  status_ = new QTextBrowser(this);
  status_->setFixedHeight(140);  // Long joint errors must not hide Stop/Release.
  layout->addWidget(status_);
  stop_ = new QPushButton("停止轨迹 · 保持当前位置", this);
  stop_->setStyleSheet("background:#a52b37;color:white;font-weight:bold;padding:8px;");
  layout->addWidget(stop_);
  connect(stop_, &QPushButton::clicked, this, [this] {close_requested_ = false; send("stop");});
  release_ = new QPushButton("规划并执行回零 · J1～J6 = 0", this); layout->addWidget(release_);
  connect(release_, &QPushButton::clicked, this, [this] {send("home");});
  emergency_ = new QPushButton("已支撑 · 紧急失能（跳过回零）", this); layout->addWidget(emergency_);
  connect(emergency_, &QPushButton::clicked, this, [this] {send("emergency_release");});
  detached_close_ = new QPushButton("主控失联 · 仅关闭显示（不代表已回零）", this);
  detached_close_->setVisible(false);
  layout->addWidget(detached_close_);
  connect(detached_close_, &QPushButton::clicked, this, [this] {
    display_only_close_ = true;
    window()->close();
  });
  auto note = new QLabel("绿色：MoveIt IK 目标；动画：原生轨迹。\n原程序关节限值，速度／加速度默认 10%。\nFollowJointTrajectory → ros2_control，100 Hz。\n修改参数后需重新规划；拖动只更新模型。", this);
  note->setWordWrap(true); layout->addWidget(note); layout->addStretch();
  refresh(QJsonObject());
  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this] {
    if (std::chrono::steady_clock::now() - received_ > std::chrono::seconds(2)) {
      refresh(QJsonObject());
      detached_close_->setVisible(true);
      status_->setPlainText("主控状态已失联，无法确认回零或失能。\n可点击下方按钮关闭显示；这不会伪报回零完成。");
    }
  }); timer->start(250);
}

void PlanningPanel::onInitialize() {
  QTimer::singleShot(0, this, [this] {window()->installEventFilter(this);});
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {return;}
  auto node = abstraction->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::String>("ui/command", 10);
  subscription_ = node->create_subscription<std_msgs::msg::String>("ui/status", 1,
    [this](const std_msgs::msg::String::SharedPtr message) {
      auto document = QJsonDocument::fromJson(QByteArray::fromStdString(message->data));
      if (!document.isObject()) {return;}
      auto state = document.object();
      QMetaObject::invokeMethod(this, [this, state] {
        received_ = std::chrono::steady_clock::now(); refresh(state);
      }, Qt::QueuedConnection);
    });
}

bool PlanningPanel::eventFilter(QObject * watched, QEvent * event) {
  if (watched == window() && event->type() == QEvent::Close && !close_allowed_ && !display_only_close_) {
    // The backend may have crashed or been killed. Its absence cannot approve
    // homing, but it must not make an orphan visualization impossible to close.
    bool parsed = false;
    const int parent = qEnvironmentVariableIntValue("A6Y_SESSION_PID", &parsed);
    if (parsed && parent > 0 && ::kill(parent, 0) == -1 && errno == ESRCH) {
      return rviz_common::Panel::eventFilter(watched, event);
    }
    static_cast<QCloseEvent *>(event)->ignore();
    if (!close_requested_) {
      close_requested_ = true;
      send("shutdown");
      status_->setPlainText("已请求退出，等待主控确认接管状态。\n未接管直接关闭；已接管先回零。停止按钮可取消退出。");
    }
    return true;
  }
  return rviz_common::Panel::eventFilter(watched, event);
}

void PlanningPanel::send(const QString & command, const QJsonValue & value) {
  if (!publisher_) {return;}
  std_msgs::msg::String message;
  message.data = QJsonDocument(QJsonObject{{"command", command}, {"value", value}}).toJson(QJsonDocument::Compact).toStdString();
  publisher_->publish(message);
}

QJsonObject PlanningPanel::settings() const {
  return QJsonObject{{"velocity_scaling", speed_->value()/100.},
    {"acceleration_scaling", acceleration_->value()/100.}, {"position_only", position_only_->isChecked()}};
}

void PlanningPanel::refresh(const QJsonObject & s) {
  const bool ready = s["ready"].toBool();
  const bool idle = ready && !s["busy"].toBool() && !s["executing"].toBool() && !s["home_active"].toBool();
  // A lost final heartbeat must not revoke an already-approved close.
  close_allowed_ = close_allowed_ || s["close_allowed"].toBool();
  if (!s.isEmpty()) {detached_close_->setVisible(false);}
  if (s["close_failed"].toBool()) {close_requested_ = false;}
  const bool executable = idle && s["plan_ready"].toBool() && (s["mock"].toBool() || s["permitted"].toBool());
  if (s.contains("role")) {identity_->setText(s["role"].toString() + (s["mock"].toBool() ? " · 模拟" : " · 真机"));}
  QSignalBlocker permit_block(permit_), opening_block(opening_), speed_block(speed_),
    acceleration_block(acceleration_), mode_block(position_only_);
  if (!speed_->hasFocus() && s["velocity_scaling"].isDouble()) {speed_->setValue(qRound(100*s["velocity_scaling"].toDouble()));}
  if (!acceleration_->hasFocus() && s["acceleration_scaling"].isDouble()) {acceleration_->setValue(qRound(100*s["acceleration_scaling"].toDouble()));}
  if (s["position_only"].isBool()) {position_only_->setChecked(s["position_only"].toBool());}
  permit_->setChecked(s["permitted"].toBool()); permit_->setEnabled(ready && !s["mock"].toBool());
  for (auto button : {plan_, reset_, grip_plan_, open_, close_}) {button->setEnabled(idle);}
  speed_->setEnabled(idle); acceleration_->setEnabled(idle); position_only_->setEnabled(idle); opening_->setEnabled(idle);
  execute_->setEnabled(executable && s["plan_group"].toString() == "arm");
  grip_execute_->setEnabled(executable && s["plan_group"].toString() == "gripper");
  QString execution_hint;
  if (!ready) {
    execution_hint = "执行不可用：等待会话就绪。";
  } else if (!idle) {
    execution_hint = "执行不可用：当前操作尚未结束。";
  } else if (!s["mock"].toBool() && !s["permitted"].toBool()) {
    execution_hint = s["plan_ready"].toBool()
      ? "规划已通过；执行按钮等待授权：请勾选上方“允许真机执行”。"
      : "真机执行尚未授权：勾选上方许可后，规划、预览，再点击执行。";
  } else if (!s["plan_ready"].toBool()) {
    execution_hint = "执行不可用：请重新规划。修改目标或参数、轨迹超过 30 秒后需重新规划。";
  } else {
    execution_hint = QString("可执行%1：检查预览后点击执行；控制器将在点击后自动激活。")
      .arg(s["plan_group"].toString() == "gripper" ? "夹爪" : "机械臂");
  }
  execution_hint_->setText(execution_hint);
  execution_hint_->setStyleSheet(ready && !s["mock"].toBool() && !s["permitted"].toBool()
    ? "color:#a34700;font-weight:bold;" : "");
  execute_->setToolTip(s["plan_ready"].toBool() && s["plan_group"].toString() != "arm"
    ? "当前为夹爪规划，请点击“执行夹爪”。" : execution_hint);
  grip_execute_->setToolTip(s["plan_ready"].toBool() && s["plan_group"].toString() != "gripper"
    ? "当前为机械臂规划，请点击“执行 Execute”。" : execution_hint);
  stop_->setEnabled(publisher_ != nullptr);
  release_->setEnabled(ready && !s["home_active"].toBool() && (s["mock"].toBool() || s["permitted"].toBool()));
  emergency_->setEnabled(s["enabled"].toBool() || s["executing"].toBool() || s["home_active"].toBool());
  if (!opening_->isSliderDown() && s["gripper_percent"].isDouble()) {
    opening_->setValue(qRound(s["gripper_percent"].toDouble()));
    opening_label_->setText(QString("目标开度：%1%（电子限位内行程）").arg(opening_->value()));
  }
  feedback_->setText(QString("单指位置：反馈 %1 mm / 目标 %2 mm")
    .arg(s["gripper_current_mm"].isDouble() ? QString::number(s["gripper_current_mm"].toDouble(), 'f', 2) : "—")
    .arg(s["gripper_target_mm"].isDouble() ? QString::number(s["gripper_target_mm"].toDouble(), 'f', 2) : "—") +
    "\n" + s["gripper_limit_status"].toString());
  const auto status = s["status"].toString() + "\n" + s["preview_status"].toString() + "\n" +
    s["hardware_status"].toString() + "\n" + s["collision_status"].toString() + "\n" +
    s["wait_note"].toString() + "\n" + s["warning"].toString() + "\n" + s["error"].toString();
  if (status_->toPlainText() != status) {status_->setPlainText(status);}
}
}
PLUGINLIB_EXPORT_CLASS(a6y_rviz_panel::PlanningPanel, rviz_common::Panel)
