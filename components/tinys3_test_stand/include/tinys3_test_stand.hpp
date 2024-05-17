#pragma once

#pragma once

#include <string>
#include <vector>

#include "base_component.hpp"
#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "i2c.hpp"
#include "mt6701.hpp"
#include "simple_lowpass_filter.hpp"

namespace espp {
/// This class acts as a board support component for the TinyS3 BLDC Test Stand.
/// It provides a high-level interface to the system's functionality.
///
/// High level overview of the system
/// - ESP32s3 module, using TinyS3
/// - TMC6300-BOB motor driver on a breakout board
/// - One MT6701 magnetic encoder connected via I2C
class TinyS3TestStand : public BaseComponent {
public:
  using Encoder = espp::Mt6701<>;
  using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;

  /// Constructor
  /// \param verbosity The verbosity level for the logger of the MotorGo-Mini
  ///        and its components
  explicit TinyS3TestStand(espp::Logger::Verbosity verbosity = espp::Logger::Verbosity::WARN)
      : BaseComponent("TinyS3 Test Stand", verbosity) {
    init();
  }

  /// Get a reference to the encoder
  /// \return A reference to the encoder
  Encoder &encoder() { return encoder_; }

  /// Get a reference to the motor driver
  /// \return A reference to the motor driver
  espp::BldcDriver &motor_driver() { return motor_driver_; }

  /// Get a reference to the motor
  /// \return A reference to the motor
  BldcMotor &motor() { return motor_; }

protected:
  static constexpr auto I2C_PORT = I2C_NUM_0;
  static constexpr auto I2C_SDA_PIN = GPIO_NUM_8;
  static constexpr auto I2C_SCL_PIN = GPIO_NUM_9;

  static constexpr uint64_t core_update_period_us = 1000; // 1 ms

  static constexpr auto MOTOR_A_H = GPIO_NUM_1;
  static constexpr auto MOTOR_A_L = GPIO_NUM_2;
  static constexpr auto MOTOR_B_H = GPIO_NUM_3;
  static constexpr auto MOTOR_B_L = GPIO_NUM_4;
  static constexpr auto MOTOR_C_H = GPIO_NUM_5;
  static constexpr auto MOTOR_C_L = GPIO_NUM_21;
  static constexpr auto MOTOR_ENABLE = GPIO_NUM_34;
  static constexpr auto MOTOR_FAULT = GPIO_NUM_36;

  void init() {
    init_encoder();
    init_motor();
  }

  void init_encoder() {
    bool run_task = true;
    std::error_code ec;
    encoder_.initialize(run_task, ec);
    if (ec) {
      logger_.error("Could not initialize encoder: {}", ec.message());
    }
  }

  void init_motor() { motor_.initialize(); }

  /// I2C bus for external communication
  I2c i2c_{{
      .port = I2C_PORT,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .clk_speed = 1 * 1000 * 1000, // MT6701 supports 1 MHz I2C
  }};

  // Encoder
  Encoder encoder_{
      {.write = std::bind(&espp::I2c::write, &i2c_, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
       .read = std::bind(&espp::I2c::read, &i2c_, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
       .update_period = std::chrono::duration<float>(core_update_period_us / 1e6f),
       .auto_init = false, // we have to initialize the SPI first before we can use the encoder
       .log_level = get_log_level()}};

  // Driver
  espp::BldcDriver motor_driver_{{.gpio_a_h = MOTOR_A_H,
                                  .gpio_a_l = MOTOR_A_L,
                                  .gpio_b_h = MOTOR_B_H,
                                  .gpio_b_l = MOTOR_B_L,
                                  .gpio_c_h = MOTOR_C_H,
                                  .gpio_c_l = MOTOR_C_L,
                                  .gpio_enable = MOTOR_ENABLE,
                                  .gpio_fault = MOTOR_FAULT,
                                  .power_supply_voltage = 5.0f,
                                  .limit_voltage = 5.0f,
                                  .log_level = get_log_level()}};

  // Filters
  espp::SimpleLowpassFilter motor_velocity_filter_{{.time_constant = 0.005f}};
  espp::SimpleLowpassFilter motor_angle_filter_{{.time_constant = 0.001f}};

  // Motor
  BldcMotor motor_{{
      .num_pole_pairs = 7,
      .phase_resistance = 5.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      // create shared_ptr from raw pointer to ensure shared_ptr doesn't delete the object
      .driver =
          std::shared_ptr<espp::BldcDriver>(std::shared_ptr<espp::BldcDriver>{}, &motor_driver_),
      // create shared_ptr from raw pointer to ensure shared_ptr doesn't delete the object
      .sensor = std::shared_ptr<Encoder>(std::shared_ptr<Encoder>{}, &encoder_),
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
      .velocity_filter = [this](auto v) { return motor_velocity_filter_(v); },
      .angle_filter = [this](auto v) { return motor_angle_filter_(v); },
      .auto_init = false, // we have to initialize the SPI first before we can use the encoder
      .log_level = get_log_level(),
  }};
};
} // namespace espp
