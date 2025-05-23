#include <chrono>
#include <vector>

#include "bldc_haptics.hpp"
#include "cli.hpp"

#include "motorgo-mini.hpp"
#include "tinys3_test_stand.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BLDC Test Stand", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI
#pragma message("Using MotorGo Mini hardware configuration")
  logger.info("Using MotorGo Mini hardware configuration");
  // we don't want to init both motors, so we'll pass in auto_init=false
  auto &motorgo_mini = espp::MotorGoMini::get();
  auto motor1_config = motorgo_mini.default_motor1_config;
  motorgo_mini.init_motor_channel_1(motor1_config);
  auto motor = motorgo_mini.motor1();
  using BldcHaptics = espp::BldcHaptics<espp::MotorGoMini::BldcMotor>;
#elif CONFIG_EXAMPLE_HARDWARE_TEST_STAND
#pragma message("Using TinyS3 Test Stand hardware configuration")
  logger.info("Using TinyS3 Test Stand hardware configuration");
  auto &test_stand = espp::TinyS3TestStand::get();
  auto motor = test_stand.motor();
  using BldcHaptics = espp::BldcHaptics<espp::TinyS3TestStand::BldcMotor>;
#else
#error "No hardware configuration selected"
#endif

  auto haptic_motor = BldcHaptics(BldcHaptics::Config{.motor = motor,
                                                      .kp_factor = 2,
                                                      .kd_factor_min = 0.01,
                                                      .kd_factor_max = 0.04,
                                                      .log_level = espp::Logger::Verbosity::WARN});

  // set the default detent config to be unbounded no detents so that if
  haptic_motor.update_detent_config(espp::detail::UNBOUNDED_NO_DETENTS);

  auto root_menu = std::make_unique<cli::Menu>("haptics", "Haptic configuration menu");
  root_menu->Insert(
      "start",
      [&](std::ostream &out) {
        if (!haptic_motor.is_running()) {
          out << "Starting motor!\n";
          haptic_motor.start();
        } else {
          out << "Motor already running!\n";
        }
      },
      "Start the motor");
  root_menu->Insert(
      "stop",
      [&](std::ostream &out) {
        if (haptic_motor.is_running()) {
          out << "Stopping motor!\n";
          haptic_motor.stop();
        } else {
          out << "Motor already stopped!\n";
        }
      },
      "Stop the motor");
  root_menu->Insert(
      "position",
      [&](std::ostream &out) {
        out << "Current position: " << haptic_motor.get_position() << "\n";
      },
      "Print the current position of the haptic motor");
  root_menu->Insert(
      "shaft_angle",
      [&](std::ostream &out) {
        out << "Current shaft angle: " << motor->get_shaft_angle() << " radians\n";
      },
      "Print the current position of the haptic motor");
  root_menu->Insert(
      "electrical_angle",
      [&](std::ostream &out) {
        out << "Current electrical angle: " << motor->get_electrical_angle() << " radians\n";
      },
      "Print the current position of the haptic motor");
  root_menu->Insert(
      "unbounded_no_detents",
      [&](std::ostream &out) {
        out << "Setting to unbounded no detents!\n";
        haptic_motor.update_detent_config(espp::detail::UNBOUNDED_NO_DETENTS);
      },
      "Set the haptic config to unbounded no detents");
  root_menu->Insert(
      "bounded_no_detents",
      [&](std::ostream &out) {
        out << "Setting to bounded no detents!\n";
        haptic_motor.update_detent_config(espp::detail::BOUNDED_NO_DETENTS);
      },
      "Set the haptic config to bounded no detents");
  root_menu->Insert(
      "multi_rev_no_detents",
      [&](std::ostream &out) {
        out << "Setting to multi rev no detents!\n";
        haptic_motor.update_detent_config(espp::detail::MULTI_REV_NO_DETENTS);
      },
      "Set the haptic config to multi rev no detents");
  root_menu->Insert(
      "on_off_strong_detents",
      [&](std::ostream &out) {
        out << "Setting to on off strong detents!\n";
        haptic_motor.update_detent_config(espp::detail::ON_OFF_STRONG_DETENTS);
      },
      "Set the haptic config to on off strong detents");
  root_menu->Insert(
      "coarse_values_strong_detents",
      [&](std::ostream &out) {
        out << "Setting to coarse values strong detents!\n";
        haptic_motor.update_detent_config(espp::detail::COARSE_VALUES_STRONG_DETENTS);
      },
      "Set the haptic config to coarse values strong detents");
  root_menu->Insert(
      "fine_values_no_detents",
      [&](std::ostream &out) {
        out << "Setting to fine values no detents!\n";
        haptic_motor.update_detent_config(espp::detail::FINE_VALUES_NO_DETENTS);
      },
      "Set the haptic config to fine values no detents");
  root_menu->Insert(
      "fine_values_with_detents",
      [&](std::ostream &out) {
        out << "Setting to fine values with detents!\n";
        haptic_motor.update_detent_config(espp::detail::FINE_VALUES_WITH_DETENTS);
      },
      "Set the haptic config to fine values with detents");
  root_menu->Insert(
      "magnetic_detents",
      [&](std::ostream &out) {
        out << "Setting to magnetic detents!\n";
        haptic_motor.update_detent_config(espp::detail::MAGNETIC_DETENTS);
      },
      "Set the haptic config to magnetic detents");
  root_menu->Insert(
      "return_to_center_with_detents",
      [&](std::ostream &out) {
        out << "Setting to return to center with detents!\n";
        haptic_motor.update_detent_config(espp::detail::RETURN_TO_CENTER_WITH_DETENTS);
      },
      "Set the haptic config to return to center with detents");
  root_menu->Insert(
      "click",
      // cppcheck-suppress constParameterReference
      [&](std::ostream &out, float strength) {
        strength = std::clamp(strength, 0.0f, 10.0f);
        espp::detail::HapticConfig config{
            .strength = strength,
            .frequency = 0.0f, // TODO: unused
            .duration = 1ms,   // TODO: unused
        };
        haptic_motor.play_haptic(config);
      },
      "Play a haptic click / buzz with the given strength (suggested range 1.0 - 5.0)");
  root_menu->Insert(
      "color",
      [](std::ostream &out) {
        out << "Colors ON\n";
        cli::SetColor();
      },
      "Enable colors in the cli");
  root_menu->Insert(
      "nocolor",
      [](std::ostream &out) {
        out << "Colors OFF\n";
        cli::SetNoColor();
      },
      "Disable colors in the cli");

  cli::Cli cli(std::move(root_menu));
  // turn the colors on by default :)
  cli::SetColor();
  // add an exit action to stop the motor when the cli exits
  cli.ExitAction([&](auto &out) {
    out << "Exiting menu and disabling haptics!\n";
    haptic_motor.stop();
    out << "Goodbye and thanks for all the fish.\n";
  });

  espp::Cli input(cli);
  input.SetInputHistorySize(10);
  input.Start();

  // if we've gotten here the cli has finished its session
}
