#define CATCH_CONFIG_RUNNER
#include "catch2/catch.hpp"

#include <iostream>
#include <string>

int main(int argc, char **argv) {
  // Unit tests for Raylib UI implementation
  
  std::string language_file = "main_test_en";
  // FIXME: pytest-cpp considers this print as a test case
  std::cout << "Loading language: " << language_file << std::endl;

  // Translation system implemented with Raylib

  const int res = Catch::Session().run(argc, argv);
  return (res < 0xff ? res : 0xff);
}