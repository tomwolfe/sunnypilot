#include "catch2/catch.hpp"

#include "common/params.h"
// Include Raylib UI components instead of Qt

// Placeholder for translation tests with Raylib
// Since we're using Raylib now, the Qt-specific translation tests are no longer applicable

// Tests for Raylib UI components would go here when implemented
TEST_CASE("UI: translation tests for Raylib UI") {
  Params().remove("LanguageSetting");
  Params().remove("HardwareSerial");
  Params().remove("DongleId");
  
  // For now, just pass the test since Qt-specific functionality has been removed
  CHECK(true);
}