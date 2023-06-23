#include <chrono>
#include <vector>

#include <driver/i2c.h>

#include "bldc_driver.hpp"
#include "bldc_haptics.hpp"
#include "bldc_motor.hpp"
#include "butterworth_filter.hpp"
#include "cli.hpp"
#include "lowpass_filter.hpp"
#include "mt6701.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// pins for the bldc motor test stand with the TinyS3
static constexpr auto I2C_NUM = (I2C_NUM_1);
static constexpr auto I2C_SCL_IO = (GPIO_NUM_9);
static constexpr auto I2C_SDA_IO = (GPIO_NUM_8);
static constexpr int I2C_FREQ_HZ = (400 * 1000);
static constexpr int I2C_TIMEOUT_MS = (10);

extern "C" void app_main(void) {
  espp::Cli::configure_stdin_stdout();

  espp::Logger logger({.tag = "BLDC Test Stand", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  // make the I2C that we'll use to communicate with the mt6701 (magnetic encoder)
  i2c_config_t i2c_cfg;
  logger.info("initializing i2c driver...");
  memset(&i2c_cfg, 0, sizeof(i2c_cfg));
  i2c_cfg.sda_io_num = I2C_SDA_IO;
  i2c_cfg.scl_io_num = I2C_SCL_IO;
  i2c_cfg.mode = I2C_MODE_MASTER;
  i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
  auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
  if (err != ESP_OK)
    logger.error("config i2c failed");
  err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK)
    logger.error("install i2c driver failed");
  // make some lambda functions we'll use to read/write to the mt6701
  auto i2c_write = [](uint8_t dev_addr, uint8_t *data, size_t len) {
    i2c_master_write_to_device(I2C_NUM, dev_addr, data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };

  auto i2c_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_master_write_read_device(I2C_NUM, dev_addr, &reg_addr, 1, data, len,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };

  // make the velocity filter
  static constexpr float core_update_period = 0.001f; // seconds
  static constexpr float filter_cutoff_hz = 4.0f;
  espp::ButterworthFilter<2, espp::BiquadFilterDf2> bwfilter({
      .normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * 0.01 // core_update_period
  });
  espp::LowpassFilter lpfilter(
      {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * 0.01, // core_update_period,
       .q_factor = 1.0f});
  auto filter_fn = [&bwfilter, &lpfilter](float raw) -> float {
    // return bwfilter.update(raw);
    // return lpfilter.update(raw);

    // NOTE: right now there seems to be something wrong with the filter
    //       configuration, so we don't filter at all. Either 1) the filtering
    //       is not actually removing the noise we want, 2) it is adding too
    //       much delay for the PID to compensate for, or 3) there is a bug in
    //       the update function which doesn't take previous state into
    //       account?
    return raw;
  };

  // now make the mt6701 which decodes the data
  std::shared_ptr<espp::Mt6701> mt6701 = std::make_shared<espp::Mt6701>(
      espp::Mt6701::Config{.write = i2c_write,
                           .read = i2c_read,
                           .velocity_filter = filter_fn,
                           .update_period = std::chrono::duration<float>(core_update_period),
                           .log_level = espp::Logger::Verbosity::WARN});

  // now make the bldc driver
  std::shared_ptr<espp::BldcDriver> driver = std::make_shared<espp::BldcDriver>(
      espp::BldcDriver::Config{// this pinout is configured for the TinyS3 connected to the
                               // TMC6300-BOB in the BLDC Motor Test Stand
                               .gpio_a_h = 1,
                               .gpio_a_l = 2,
                               .gpio_b_h = 3,
                               .gpio_b_l = 4,
                               .gpio_c_h = 5,
                               .gpio_c_l = 21,
                               .gpio_enable = 34, // connected to the VIO/~Stdby pin of TMC6300-BOB
                               .gpio_fault = 36,  // connected to the nFAULT pin of TMC6300-BOB
                               .power_supply_voltage = 5.0f,
                               .limit_voltage = 5.0f,
                               .log_level = espp::Logger::Verbosity::WARN});

  // now make the bldc motor
  using BldcMotor = espp::BldcMotor<espp::BldcDriver, espp::Mt6701>;
  auto motor = BldcMotor(BldcMotor::Config{
      // measured by setting it into ANGLE_OPENLOOP and then counting how many
      // spots you feel when rotating it.
      .num_pole_pairs = 7,
      .phase_resistance =
          5.0f, // tested by running velocity_openloop and seeing if the veloicty is ~correct
      .kv_rating =
          320, // tested by running velocity_openloop and seeing if the velocity is ~correct
      .current_limit = 1.0f,             // Amps
      .zero_electric_offset = 2.3914752, // gotten from previously running without providing this
      // and it will be logged.
      .sensor_direction = espp::detail::SensorDirection::COUNTER_CLOCKWISE,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = driver,
      .sensor = mt6701,
      .velocity_pid_config =
          {
              .kp = 0.010f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 7.000f,
              .ki = 0.300f,
              .kd = 0.010f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .log_level = espp::Logger::Verbosity::WARN});

  using BldcHaptics = espp::BldcHaptics<BldcMotor>;

  auto haptic_motor = BldcHaptics({.motor = motor,
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
        out << "Current shaft angle: " << motor.get_shaft_angle() << " radians\n";
      },
      "Print the current position of the haptic motor");
  root_menu->Insert(
      "electrical_angle",
      [&](std::ostream &out) {
        out << "Current electrical angle: " << motor.get_electrical_angle() << " radians\n";
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
