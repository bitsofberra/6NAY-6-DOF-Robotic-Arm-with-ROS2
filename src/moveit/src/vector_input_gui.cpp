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
    pub_  = node_->create_publisher<geometry_msgs::msg::Point>("/vector_distance_input", 10);

    auto* v = new QVBoxLayout(this);
    auto* f = new QFormLayout();

    x_ = new QDoubleSpinBox(); x_->setRange(-2.0,  2.0); x_->setDecimals(4); x_->setSingleStep(0.01); x_->setValue(0.50);
    y_ = new QDoubleSpinBox(); y_->setRange(-2.0,  2.0); y_->setDecimals(4); y_->setSingleStep(0.01); y_->setValue(0.10);
    z_ = new QDoubleSpinBox(); z_->setRange( 0.0,  2.0); z_->setDecimals(4); z_->setSingleStep(0.01); z_->setValue(0.54);

    f->addRow("x", x_);
    f->addRow("y", y_);
    f->addRow("z", z_);
    v->addLayout(f);

    auto* send = new QPushButton("Gönder");
    v->addWidget(send);

    status_ = new QLabel("Topic: /vector_distance_input");
    v->addWidget(status_);

    connect(send, &QPushButton::clicked, this, [this](){
      geometry_msgs::msg::Point p;
      p.x = x_->value(); p.y = y_->value(); p.z = z_->value();
      pub_->publish(p);
      status_->setText(QString("Yollandı: x=%1, y=%2, z=%3")
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
  QDoubleSpinBox *x_{}, *y_{}, *z_{};
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
  w.setWindowTitle("Vector Distance Input (GUI -> /vector_distance_input)");
  w.show();

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}

#include "vector_input_gui.moc"