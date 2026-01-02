#include "recorder/scan_player.hh"
#include "sensors.pb.h"
#include <filesystem>
#include <getopt.h>
#include <iostream>

static void printUsage() {
  std::cerr << "Usage: scan_parser [-f file] [-t sim time] [-p publish]"
            << std::endl;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printUsage();
    exit(0);
  }
  bool simulate_time = false;
  bool publish_scan = false;
  std::string file;
  int opt;
  while ((opt = getopt(argc, argv, "tpf:")) != -1) {
    switch (opt) {
    case 't':
      std::cout << "Sim time" << std::endl;
      simulate_time = true;
      break;

    case 'p':
      std::cout << "Publish (TODO)" << std::endl;
      publish_scan = true;
      break;
    case 'f':
      file = optarg;
      break;

    default:
      printUsage();
      exit(0);
    }
  }

  if (!std::filesystem::exists(file)) {
    std::cerr << "File does not exist: " << file << std::endl;
    exit(-1);
  }

  msensor::ScanPlayer player(file);

  size_t nr_imu_entries = 0;
  size_t nr_scan_entries = 0;

  while (player.next()) {
    const auto entry = player.getLastEntry();

    switch (entry.entry_case()) {
    case sensors::RecordingEntry::kScan:
      nr_scan_entries++;
      break;

    case sensors::RecordingEntry::kImu:
      nr_imu_entries++;
      break;

    default:
      break;
    }
  }

  std::cout << std::format("Scans: {}, Imu: {}\n", nr_scan_entries,
                           nr_imu_entries);
}