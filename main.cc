#include <iostream>
#include <vector>
#include <string>

#include "pcd_sequence_viewer.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <dir>\n";
    return -1;
  }
  const std::string dir(argv[1]);
  PcdSequenceViewer viewer(dir);
  viewer.Run();

  return 0;
}
