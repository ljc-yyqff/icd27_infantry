#include "io/gimbal1/gimbal1.hpp"
#include "gimbal1_console.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
void print_help()
{
  std::cout <<
    "Commands:\n"
    "  enable | e                 send ge\\r\\n\n"
    "  disable | d                send gd\\r\\n\n"
    "  yaw | y <angle> <speed>    send relative yaw command yr\n"
    "  pitch | p <angle> <speed>  send relative pitch command pr\n"
    "  status                     show axis state, age and raw status replies\n"
    "  help | h                   show this help\n"
    "  quit | q                   disable the gimbal and exit\n"
    "Angles are increments in degrees; speed is in degrees/second.\n";
}

void require_no_extra_input(std::istringstream & input)
{
  std::string extra;
  if (input >> extra) throw std::invalid_argument("unexpected argument: " + extra);
}

void read_motion_arguments(std::istringstream & input, double & angle, double & speed)
{
  if (!(input >> angle >> speed)) {
    throw std::invalid_argument("expected: <relative-angle-deg> <max-speed-deg-s>");
  }
  require_no_extra_input(input);
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/gimbal1.yaml";

  try {
    io::Gimbal1 gimbal(config_path);
    std::cout << "gimbal1 opened " << gimbal.port() << " at " << gimbal.baudrate()
              << " baud (8N1). It starts DISABLED.\n";
    std::cout << "Standalone mode: CAN and the legacy binary gimbal protocol are not initialized.\n";
    print_help();

    std::string line;
    while (std::cout << "gimbal1> " << std::flush && gimbal1_read_command(gimbal, line)) {
      std::istringstream input(line);
      std::string command;
      if (!(input >> command)) continue;

      try {
        if (command == "enable" || command == "e") {
          require_no_extra_input(input);
          gimbal.enable();
          std::cout << "ge sent; motor execution belongs to A-board\n";
        } else if (command == "disable" || command == "d") {
          require_no_extra_input(input);
          gimbal.disable();
          std::cout << "sent: gd\\r\\n (disabled)\n";
        } else if (command == "yaw" || command == "y") {
          double angle = 0.0;
          double speed = 0.0;
          read_motion_arguments(input, angle, speed);
          gimbal.move_relative_yaw(angle, speed);
          std::cout << "sent: yr " << angle << ' ' << speed << "\\r\\n\n";
        } else if (command == "pitch" || command == "p") {
          double angle = 0.0;
          double speed = 0.0;
          read_motion_arguments(input, angle, speed);
          gimbal.move_relative_pitch(angle, speed);
          std::cout << "sent: pr " << angle << ' ' << speed << "\\r\\n\n";
        } else if (command == "status") {
          require_no_extra_input(input);
          io::Gimbal1Feedback feedback;
          gimbal.poll_feedback(feedback);
          std::cout << gimbal.status_text() << '\n';
        } else if (command == "help" || command == "h") {
          require_no_extra_input(input);
          print_help();
        } else if (command == "quit" || command == "q") {
          require_no_extra_input(input);
          gimbal.disable();
          std::cout << "sent: gd\\r\\n; exiting safely.\n";
          return 0;
        } else {
          std::cerr << "Unknown command. Type 'help'.\n";
        }
      } catch (const std::exception & e) {
        std::cerr << "Command rejected: " << e.what() << '\n';
      }
    }

    std::cout << "\nInput closed; closing serial only (no automatic gd).\n";
  } catch (const std::exception & e) {
    std::cerr << "gimbal1 startup failed: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
