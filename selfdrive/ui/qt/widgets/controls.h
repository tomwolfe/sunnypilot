#pragma once

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

// Minimal controls for cabana compatibility
class BaseControl : public QFrame {
  Q_OBJECT

public:
  explicit BaseControl(const QString &title = "", const QString &desc = "", QWidget *parent = nullptr);
  void setDescription(const QString &desc);
  void addItem(QWidget *widget);

protected:
  QVBoxLayout *main_layout;
  QLabel *title_label;
  QLabel *desc_label;
};

class ButtonControl : public BaseControl {
  Q_OBJECT

public:
  ButtonControl(const QString &title, const QString &text, const QString &desc = "", QWidget *parent = nullptr);

Q_SIGNALS:
  void clicked();

private:
  QPushButton *btn;
};