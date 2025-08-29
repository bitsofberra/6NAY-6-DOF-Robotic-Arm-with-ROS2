#include <rqt_gui_cpp/plugin.h>
#include <QPushButton>
#include <QWidget>
#include <QHBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <panda_yolo_pickplace/srv/trigger_pick_place.hpp>

namespace panda_yolo_pickplace {
class GuiPanel : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:
  GuiPanel(): rqt_gui_cpp::Plugin(){ setObjectName("PandaPickPlaceGUI"); }
  void initPlugin(qt_gui_cpp::PluginContext& context) override {
    widget_ = new QWidget(); auto *layout = new QHBoxLayout(widget_);
    auto *btn_red = new QPushButton("Pick RED"); auto *btn_green = new QPushButton("Pick GREEN");
    layout->addWidget(btn_red); layout->addWidget(btn_green); widget_->setLayout(layout);
    context.addWidget(widget_);
    node_ = std::make_shared<rclcpp::Node>("pick_gui");
    client_ = node_->create_client<panda_yolo_pickplace::srv::TriggerPickPlace>("/pick_place/trigger");
    QObject::connect(btn_red, &QPushButton::clicked, [this](){ call("red"); });
    QObject::connect(btn_green, &QPushButton::clicked, [this](){ call("green"); });
  }
  void shutdownPlugin() override { widget_->deleteLater(); }
private:
  void call(const std::string &c){
    if(!client_->wait_for_service(std::chrono::seconds(1))) return;
    auto req = std::make_shared<panda_yolo_pickplace::srv::TriggerPickPlace::Request>();
    req->color = c; auto fut = client_->async_send_request(req);
  }
  QWidget *widget_ = nullptr; rclcpp::Node::SharedPtr node_;
  rclcpp::Client<panda_yolo_pickplace::srv::TriggerPickPlace>::SharedPtr client_;
};
} // ns
#include <pluginlib/class_list_macros.hpp>
#include "gui_panel.moc"
PLUGINLIB_EXPORT_CLASS(panda_yolo_pickplace::GuiPanel, rqt_gui_cpp::Plugin)