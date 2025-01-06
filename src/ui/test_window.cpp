/**
 * @file test_window.cpp
 * @brief 测试窗口类的实现
 */

#include "ui/test_window.h"
#include <QVBoxLayout>

TestWindow::TestWindow(QWidget* parent) : QMainWindow(parent) {
    // Create central widget
    QWidget* central = new QWidget(this);
    setCentralWidget(central);
    
    // Create layout
    QVBoxLayout* layout = new QVBoxLayout(central);
    
    // Create joystick
    joystick_ = new JoystickWidget(this);
    
    // Create label to display joystick output
    position_label_ = new QLabel("Position: (0.00, 0.00)", this);
    position_label_->setAlignment(Qt::AlignCenter);
    
    // Add to layout
    layout->addWidget(joystick_, 0, Qt::AlignCenter);
    layout->addWidget(position_label_);
    
    // Connect signals
    connect(joystick_, &JoystickWidget::positionChanged,
            this, &TestWindow::onPositionChanged);
            
    // Set window properties
    setWindowTitle("Joystick Test");
    resize(300, 400);
}

void TestWindow::onPositionChanged(double x, double y) {
    QString text = QString("Position: (%1, %2)")
        .arg(QString::number(x, 'f', 2))
        .arg(QString::number(y, 'f', 2));
    position_label_->setText(text);
} 