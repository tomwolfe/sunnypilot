#pragma once

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QStandardPaths>

// Minimal util functions for cabana compatibility
inline QString getcwd() {
  return QDir::currentPath();
}

inline QString getAvailableSpace(const QString &path = QStandardPaths::writableLocation(QStandardPaths::HomeLocation)) {
  // Return a default string for compatibility
  return "Unknown";
}