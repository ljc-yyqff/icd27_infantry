#include "io/gimbal1/gimbal1.hpp"
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  try { io::Gimbal1 g(argv[1]); g.enable(); std::cout << "ge sent without motor state waiting\n"; return 0; }
  catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
