#include "tinys3_test_stand.hpp"

using namespace espp;

TinyS3TestStand::TinyS3TestStand()
    : BaseComponent("TinyS3 Test Stand") {}

void TinyS3TestStand::init_motor(const TinyS3TestStand::BldcMotor::Config &motor_config,
                                 const TinyS3TestStand::DriverConfig &driver_config) {
  bool run_task = true;
  std::error_code ec;
  // make the encoder
  encoder_ = std::make_shared<Encoder>(encoder_config_);
  // initialize the encoder
  encoder_->initialize(run_task, ec);
  if (ec) {
    logger_.error("Could not initialize encoder: {}", ec.message());
    return;
  }

  // copy the config data for the driver
  motor_driver_config_.power_supply_voltage = driver_config.power_supply_voltage;
  motor_driver_config_.limit_voltage = driver_config.limit_voltage;
  // make the driver
  motor_driver_ = std::make_shared<BldcDriver>(motor_driver_config_);

  // now copy the relevant configs into the motor config
  auto motor_config_copy = motor_config;
  motor_config_copy.driver = motor_driver_;
  motor_config_copy.sensor = encoder_;
  // now make the motor
  motor_ = std::make_shared<BldcMotor>(motor_config_copy);
  motor_->initialize();
}

std::shared_ptr<TinyS3TestStand::Encoder> TinyS3TestStand::encoder() { return encoder_; }

void TinyS3TestStand::reset_encoder_accumulator() { encoder_->reset_accumulator(); }

std::shared_ptr<espp::BldcDriver> TinyS3TestStand::motor_driver() { return motor_driver_; }

std::shared_ptr<TinyS3TestStand::BldcMotor> TinyS3TestStand::motor() { return motor_; }

TinyS3TestStand::VelocityFilter &TinyS3TestStand::motor_velocity_filter() {
  return motor_velocity_filter_;
}

TinyS3TestStand::AngleFilter &TinyS3TestStand::motor_angle_filter() { return motor_angle_filter_; }
