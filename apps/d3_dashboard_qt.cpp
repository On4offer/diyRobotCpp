#include <QApplication>
#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QMainWindow window;
  auto* central = new QWidget;
  auto* root = new QVBoxLayout(central);
  auto* notice = new QLabel(QStringLiteral(
      "D3 C++ Qt Dashboard — 默认模拟模式；连接真机前先确认机械臂竖直且工作区无人。"));
  notice->setWordWrap(true);
  root->addWidget(notice);
  auto* armed = new QCheckBox(QStringLiteral("控制模式（允许写入）"));
  root->addWidget(armed);
  auto* grid = new QGridLayout;
  root->addLayout(grid);
  for (int i = 0; i < 6; ++i) {
    auto* name = new QLabel(QString("Servo %1").arg(i + 1));
    auto* status = new QLabel("pos=2048  12.0V  28C  OK");
    auto* slider = new QSlider(Qt::Horizontal);
    slider->setRange(800, 3400);
    slider->setValue(2048);
    slider->setEnabled(false);
    QObject::connect(armed, &QCheckBox::toggled, slider, &QSlider::setEnabled);
    grid->addWidget(name, i, 0);
    grid->addWidget(status, i, 1);
    grid->addWidget(slider, i, 2);
  }
  auto* estop = new QPushButton(QStringLiteral("急停：失能全部力矩"));
  estop->setStyleSheet("background:#b00020;color:white;font-weight:bold;padding:8px");
  root->addWidget(estop);
  QObject::connect(estop, &QPushButton::clicked, [&] {
    armed->setChecked(false);
    QMessageBox::warning(
        &window, QStringLiteral("急停"),
        QStringLiteral("模拟模式：已锁定全部写控件。真机后端会同时写 Torque_Enable=0。"));
  });
  window.setCentralWidget(central);
  window.setWindowTitle("diyRobotCpp D3 Dashboard");
  window.resize(900, 360);
  window.show();
  return app.exec();
}
