#include "selfdrive/ui/qt/widgets/controls.h"

#include <QHBoxLayout>
#include <QPainter>
#include <QStyleOption>

BaseControl::BaseControl(const QString &title, const QString &desc, QWidget *parent) : QFrame(parent) {
  main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->setSpacing(0);

  // Title and description
  title_label = new QLabel(title);
  title_label->setStyleSheet("font-size: 50px; font-weight: bold;");
  title_label->setMargin(30);
  title_label->setAlignment(Qt::AlignCenter);

  desc_label = new QLabel(desc);
  desc_label->setStyleSheet("font-size: 40px; color: #777");
  desc_label->setMargin(20);
  desc_label->setAlignment(Qt::AlignCenter);
  desc_label->setWordWrap(true);

  if (!title.isEmpty()) main_layout->addWidget(title_label);
  if (!desc.isEmpty()) main_layout->addWidget(desc_label);

  setStyleSheet(R"(
    BaseControl {
      border: 1px solid #333;
      border-radius: 10px;
      background-color: #292929;
    }
  )");
}

void BaseControl::setDescription(const QString &desc) {
  desc_label->setText(desc);
}

void BaseControl::addItem(QWidget *widget) {
  main_layout->addWidget(widget);
}

ButtonControl::ButtonControl(const QString &title, const QString &text, const QString &desc, QWidget *parent)
    : BaseControl(title, desc, parent) {
  btn = new QPushButton(text);
  btn->setStyleSheet(R"(
    QPushButton {
      padding: 40px;
      font-size: 50px;
      font-weight: bold;
      border-radius: 10px;
      background-color: #333;
      color: #fff;
    }
    QPushButton:pressed {
      background-color: #444;
    }
  )");

  addItem(btn);
  QObject::connect(btn, &QPushButton::clicked, this, &ButtonControl::clicked);
}