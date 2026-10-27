// Operator confirmation before robot motions.
#pragma once

#include <iostream>
#include <string>

// Asks the operator to press Enter; anything else (or EOF) returns false.
inline bool confirm(const char* what) {
  std::cout << "\nNEXT: " << what << ". Keep the user stop at hand.\n"
            << "Press Enter to continue (anything else aborts)..." << std::endl;
  std::string line;
  if (!std::getline(std::cin, line) || !line.empty()) {
    std::cout << "Aborted.\n";
    return false;
  }
  return true;
}
