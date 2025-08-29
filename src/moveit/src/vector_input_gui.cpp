#include <QApplication>
#include <QWidget>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class VectorInputGui : public QWidget
{
  Q_OBJECT
public:
  VectorInputGui(QWidget* parent=nullptr)
  : QWidget(parent)
  {
    // ROS node & publisher
    node_ = std::make_shared<rclcpp::Node>("vector_input_gui");
    pub_  = node_->create_publisher<geometry_msgs::msg::Point>("/pick_place_cmd", 10);

    auto* v = new QVBoxLayout(this);
    auto* f = new QFormLayout();

    // Δx, Δy, Δz (metre cinsinden ofset)
    dx_ = new QDoubleSpinBox(); dx_->setRange(-1.5, 1.5); dx_->setDecimals(4); dx_->setSingleStep(0.01); dx_->setValue(-0.20);
    dy_ = new QDoubleSpinBox(); dy_->setRange(-1.5, 1.5); dy_->setDecimals(4); dy_->setSingleStep(0.01); dy_->setValue(-0.20);
    dz_ = new QDoubleSpinBox(); dz_->setRange(-0.3,  0.3); dz_->setDecimals(4); dz_->setSingleStep(0.005); dz_->setValue(0.00);

    f->addRow("Δx (m)", dx_);
    f->addRow("Δy (m)", dy_);
    f->addRow("Δz (m)", dz_);
    v->addLayout(f);

    auto* send = new QPushButton("Pick & Place");
    v->addWidget(send);

    status_ = new QLabel("Topic: /pick_place_cmd");
    v->addWidget(status_);

    connect(send, &QPushButton::clicked, this, [this](){
      geometry_msgs::msg::Point p;
      p.x = dx_->value(); p.y = dy_->value(); p.z = dz_->value();
      pub_->publish(p);
      status_->setText(QString("Gönderildi: Δx=%1, Δy=%2, Δz=%3")
                       .arg(p.x,0,'f',3).arg(p.y,0,'f',3).arg(p.z,0,'f',3));
    });

    // rclcpp spin_some için küçük timer (GUI donmasın)
    connect(&timer_, &QTimer::timeout, this, [this](){
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_);
      exec.spin_some();
    });
    timer_.start(20);
  }

private:
  QDoubleSpinBox *dx_{}, *dy_{}, *dz_{};
  QLabel* status_{};
  QTimer timer_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  VectorInputGui w;
  w.setWindowTitle("Pick & Place Command (Δx, Δy, Δz)");
  w.show();

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}

#include "vector_input_gui.moc"
