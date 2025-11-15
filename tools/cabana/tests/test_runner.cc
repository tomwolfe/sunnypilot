#define CATCH_CONFIG_RUNNER
#include "catch2/catch.hpp"

int main(int argc, char **argv) {
  // unit tests for cabana
  const int res = Catch::Session().run(argc, argv);
  return (res < 0xff ? res : 0xff);
}
