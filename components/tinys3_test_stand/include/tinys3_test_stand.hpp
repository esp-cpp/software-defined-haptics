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
  /// Alias for the encoder type
  using Encoder = espp::Mt6701<>;

  /// Alias for the BLDC motor type
  using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;

  /// Alias for the velocity filter type
  using VelocityFilter = espp::SimpleLowpassFilter;

  /// Alias for the angle filter type
  using AngleFilter = espp::SimpleLowpassFilter;

  /// @brief Access the singleton instance of the TinyS3TestStand class
  /// @return Reference to the singleton instance of the TinyS3TestStand class
  static TinyS3TestStand &get() {
    static TinyS3TestStand instance;
    return instance;
  }

  TinyS3TestStand(const TinyS3TestStand &) = delete;
  TinyS3TestStand &operator=(const TinyS3TestStand &) = delete;
  TinyS3TestStand(TinyS3TestStand &&) = delete;
  TinyS3TestStand &operator=(TinyS3TestStand &&) = delete;

  /////////////////////////////////////////////////////////////////////////////
  // Motors
  /////////////////////////////////////////////////////////////////////////////

  /// Driver Configuration for the MotorGo-Mini Motor Driver(s)
  struct DriverConfig {
    float power_supply_voltage; ///< The power supply voltage in volts
    float limit_voltage;        ///< The limit voltage in volts
  };

  /// Default configuration for the TinyS3 Test Stand's BLDC motor
  const BldcMotor::Config default_motor_config{
      .num_pole_pairs = 7,
      .phase_resistance = 4.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = motor_driver_, // NOTE: user cannot override this
      .sensor = encoder_,      // NOTE: user cannot override this
      .velocity_pid_config =
          {
              .kp = 0.020f,
              .ki = 0.700f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 5.000f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .velocity_filter = [this](float v) { return motor_velocity_filter_(v); },
      .angle_filter = [this](float a) { return motor_angle_filter_(a); },
  };

  /// Initialize the TinyS3's components for its motor
  /// \details This function initializes the encoder, driver, and motor. This
  ///          consists of initializing encoder, motor_driver, and motor.
  /// \param motor_config The motor configuration
  /// \param driver_config The driver configuration
  void init_motor(const BldcMotor::Config &motor_config,
                  const DriverConfig &driver_config = {.power_supply_voltage = 5.0f,
                                                       .limit_voltage = 5.0f});

  /// Get a reference to the motor driver
  /// \return A shared pointer to the motor driver
  std::shared_ptr<espp::BldcDriver> motor_driver();

  /// Get a reference to the motor
  /// \return A shared pointer to the motor
  std::shared_ptr<BldcMotor> motor();

  /// Get a reference to the motor velocity filter
  /// \return A reference to the motor velocity filter
  VelocityFilter &motor_velocity_filter();

  /// Get a reference to the motor angle filter
  /// \return A reference to the motor angle filter
  AngleFilter &motor_angle_filter();

  /////////////////////////////////////////////////////////////////////////////
  // Encoders
  /////////////////////////////////////////////////////////////////////////////

  /// Get a reference to the encoder
  /// \return A shared pointer to the encoder
  std::shared_ptr<Encoder> encoder();

  /// Reset the encoder accumulator
  /// \details This function resets the encoder accumulator to 0.
  ///          This will reset the encoder's position to be within the range
  ///          of 0 to 2*pi.
  void reset_encoder_accumulator();

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

  explicit TinyS3TestStand();

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
  Encoder::Config encoder_config_{
      .write = std::bind(&espp::I2c::write, &i2c_, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &i2c_, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3),
      .update_period = std::chrono::duration<float>(core_update_period_us / 1e6f),
      .log_level = get_log_level()};

  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<Encoder> encoder_{(Encoder *)(nullptr)};

  // Driver
  espp::BldcDriver::Config motor_driver_config_{
      .gpio_a_h = MOTOR_A_H,
      .gpio_a_l = MOTOR_A_L,
      .gpio_b_h = MOTOR_B_H,
      .gpio_b_l = MOTOR_B_L,
      .gpio_c_h = MOTOR_C_H,
      .gpio_c_l = MOTOR_C_L,
      .gpio_enable = MOTOR_ENABLE,
      .gpio_fault = MOTOR_FAULT,
      .power_supply_voltage = 5.0f, // NOTE: can be replaced by user
      .limit_voltage = 5.0f,        // NOTE: can be replaced by user
      .log_level = get_log_level()};
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<espp::BldcDriver> motor_driver_{(espp::BldcDriver *)(nullptr)};

  // Filters
  VelocityFilter motor_velocity_filter_{{.time_constant = 0.005f}};
  AngleFilter motor_angle_filter_{{.time_constant = 0.001f}};

  // Motor
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<BldcMotor> motor_{(BldcMotor *)(nullptr)};
};
} // namespace espp
