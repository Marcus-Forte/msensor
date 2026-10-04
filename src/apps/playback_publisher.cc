#include "msensor/recorder/recording_driver.hh"
#include "msensor_server.hh"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <poll.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {

std::atomic_bool g_should_stop = false;

void request_stop(int) { g_should_stop.store(true); }

class TerminalInputMode {
public:
  TerminalInputMode() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_) != 0) {
      return;
    }

    auto raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }

  ~TerminalInputMode() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
      std::cout << std::endl;
    }
  }

  TerminalInputMode(const TerminalInputMode &) = delete;
  TerminalInputMode &operator=(const TerminalInputMode &) = delete;

private:
  termios original_{};
  bool active_ = false;
};

void print_usage(const char *program) {
  std::cout
      << "Usage: " << program
      << " -f <recording.pbscan> [-s <speed>] [-p <port>] [--autoplay]\n"
      << "  -f, --file <file>   Recording to replay (required)\n"
      << "  -s, --speed <x>     Playback speed (1.0 = real time, 0 = max) "
         "[default 1.0]\n"
      << "  -p, --port <port>   gRPC listen port [default 50051]\n"
      << "  -a, --autoplay      Start playback immediately (useful without "
         "stdin)\n"
      << "  -h, --help          Show this help message\n"
      << "Playback starts paused unless --autoplay is specified.\n"
      << "Controls: Space pause/resume, r rewind, Right speed up, "
         "Left slow down, Ctrl-C quit.\n";
}

} // namespace

int main(int argc, char **argv) {
  std::string file;
  double speed = 1.0;
  int port = 50051;
  bool autoplay = false;

  static const option long_options[] = {
      {"file", required_argument, nullptr, 'f'},
      {"speed", required_argument, nullptr, 's'},
      {"port", required_argument, nullptr, 'p'},
      {"autoplay", no_argument, nullptr, 'a'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, 0, nullptr, 0}};

  int opt;
  while ((opt = getopt_long(argc, argv, "f:s:p:ah", long_options, nullptr)) !=
         -1) {
    switch (opt) {
    case 'f':
      file = optarg;
      break;
    case 's':
      speed = std::stod(optarg);
      break;
    case 'p':
      port = std::stoi(optarg);
      break;
    case 'a':
      autoplay = true;
      break;
    case 'h':
      print_usage(argv[0]);
      return 0;
    default:
      print_usage(argv[0]);
      return 1;
    }
  }

  if (file.empty()) {
    std::cerr << "Error: a recording file is required.\n";
    print_usage(argv[0]);
    return 1;
  }
  if (!std::filesystem::exists(file)) {
    std::cerr << "Recording does not exist: " << file << "\n";
    return 1;
  }
  if (speed < 0.0) {
    std::cerr << "Speed must be non-negative.\n";
    return 1;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  auto driver =
      std::make_shared<msensor::RecordingSensorDriver>(file, speed, !autoplay);
  driver->init();

  SensorsServer server(nullptr, nullptr, driver, driver,
                       "0.0.0.0:" + std::to_string(port));
  server.start();

  TerminalInputMode terminal_mode;
  driver->startSampling();

  std::cout << "Loaded " << file << " at speed " << speed << " (0 = max); "
            << (autoplay ? "playing" : "paused")
            << " at start. Space: play/pause, r: rewind, "
               "Right/Left: speed up/down, Ctrl-C: quit."
            << std::endl;

  const auto report_speed = [&driver]() {
    const double current = driver->speed();
    if (current == 0.0) {
      std::cout << "Playback speed: max." << std::endl;
    } else {
      std::cout << "Playback speed: " << current << "x." << std::endl;
    }
  };
  bool stdin_open = true;
  bool finished_reported = false;
  int escape_state = 0;
  while (!g_should_stop.load()) {
    if (!driver->isFinished()) {
      finished_reported = false;
    }
    if (driver->isFinished() && !finished_reported) {
      finished_reported = true;
      std::cout << "Recording finished; SLAM remains connected. Press r to "
                   "rewind or Ctrl-C to exit."
                << std::endl;
    }

    if (!stdin_open) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int result = poll(&input, 1, 50);
    if (result < 0) {
      continue;
    }
    if (result == 0) {
      continue;
    }

    if (input.revents & POLLIN) {
      char keys[32];
      const ssize_t count = read(STDIN_FILENO, keys, sizeof(keys));
      for (ssize_t i = 0; i < count; ++i) {
        const char key = keys[i];
        if (escape_state == 0) {
          if (key == '\x1b') {
            escape_state = 1;
          } else if (key == ' ') {
            if (driver->isFinished()) {
              std::cout << "Recording finished; press r to rewind."
                        << std::endl;
            } else {
              const bool pause = !driver->isPaused();
              driver->setPaused(pause);
              std::cout << (pause ? "Paused." : "Playing.") << std::endl;
            }
          } else if (key == 'r' || key == 'R') {
            const bool was_finished = driver->isFinished();
            driver->setPaused(true);
            driver->requestReset();
            finished_reported = was_finished;
            std::cout << "Rewound to start and paused. Press Space to resume."
                      << std::endl;
          }
        } else if (escape_state == 1) {
          if (key == '[') {
            escape_state = 2;
          } else if (key == 'O') {
            escape_state = 3;
          } else {
            escape_state = 0;
          }
        } else {
          const bool arrow_final = key >= '@' && key <= '~';
          if (arrow_final) {
            if (key == 'C') {
              const double previous = driver->speed();
              driver->increaseSpeed();
              if (driver->speed() == previous) {
                std::cout << "Playback is already at maximum speed."
                          << std::endl;
              } else {
                report_speed();
              }
            } else if (key == 'D') {
              driver->decreaseSpeed();
              report_speed();
            }
            escape_state = 0;
          }
        }
      }
    }
    if (input.revents & (POLLHUP | POLLERR | POLLNVAL)) {
      stdin_open = false;
    }
  }

  driver->stopSampling();
  server.stop();
  std::cout << "Playback server stopped." << std::endl;
  return 0;
}
