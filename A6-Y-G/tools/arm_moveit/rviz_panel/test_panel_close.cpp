#include "planning_panel.hpp"
#include <QApplication>
#include <QCloseEvent>
#include <QEventLoop>
#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace a6y_rviz_panel {
class PlanningPanelCloseTest {
public:
  static void check(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
  static void run() {
    QMainWindow window;
    auto panel = new PlanningPanel(&window);
    window.setCentralWidget(panel);
    window.installEventFilter(panel);
    QJsonObject planned{{"ready",true},{"plan_ready",true},{"plan_group","arm"},{"mock",false},{"permitted",false}};
    panel->refresh(planned);
    check(!panel->execute_->isEnabled(), "real execution must wait for permission");
    check(panel->execution_hint_->text().contains("等待授权"), "gray execute must explain missing permission");
    planned["permitted"] = true;
    panel->refresh(planned);
    check(panel->execute_->isEnabled(), "permission plus plan must enable execute even before controller activation");
    check(!panel->grip_execute_->isEnabled(), "arm plan must not enable gripper execution");
    planned["plan_group"] = "gripper";
    panel->refresh(planned);
    check(panel->grip_execute_->isEnabled() && !panel->execute_->isEnabled(), "gripper plan enables only gripper execution");
    planned["plan_ready"] = false;
    panel->refresh(planned);
    check(!panel->grip_execute_->isEnabled() && panel->execution_hint_->text().contains("重新规划"), "expired plan must explain replan");
    panel->refresh(QJsonObject());
    qputenv("A6Y_SESSION_PID", QByteArray::number(getpid()));
    QCloseEvent first;
    check(panel->eventFilter(&window, &first) && !first.isAccepted(), "live session must intercept close");
    check(panel->close_requested_, "close must be pending");
    panel->refresh(QJsonObject{{"close_failed", true}});
    check(!panel->close_requested_, "failed home must allow a later explicit retry");
    panel->refresh(QJsonObject{{"close_allowed", true}});
    panel->refresh(QJsonObject());
    QCloseEvent approved;
    check(!panel->eventFilter(&window, &approved), "missing heartbeat must not revoke approved close");
    panel->close_allowed_ = false;
    qputenv("A6Y_SESSION_PID", "2147483647");
    QCloseEvent orphan;
    check(!panel->eventFilter(&window, &orphan), "dead backend must not trap orphan display");
    qputenv("A6Y_SESSION_PID", QByteArray::number(getpid()));
    window.show();
    panel->received_ = std::chrono::steady_clock::now() - std::chrono::seconds(3);
    QEventLoop loop;
    QTimer::singleShot(350, &loop, &QEventLoop::quit);
    loop.exec();
    check(!panel->detached_close_->isHidden(), "stale UI must offer display-only close");
    panel->detached_close_->click();
    check(!window.isVisible(), "display-only close must work even if backend is unresponsive");
  }
};
}
int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  try {a6y_rviz_panel::PlanningPanelCloseTest::run();}
  catch (const std::exception & e) {std::cerr << e.what() << std::endl; return 1;}
  std::cout << "PASS: execution permission, inactive controller, group selection, invalid plan hint; live-close, failed-home, approved-close heartbeat loss, orphan-close, stale-display escape" << std::endl;
  return 0;
}
