#pragma once

#include "module.hpp"
#include "peripherals/i2c.hpp"
#include "utility/math/bit.hpp"
#include "utility/enum.hpp"
#include "utility/math/units.hpp"
#include "utility/log.hpp"

namespace sjsu
{
  struct pca9685_settings
  {
    units::frequency::hertz_t frequency = 50_Hz;
    units::angle::degree_t min_angle = 0_deg;
    units::angle::degree_t max_angle = 180_deg;
    std::chrono::microseconds min_pulse = 520us;
    std::chrono::microseconds max_pulse = 2400us;
  };

  /// Driver for the PCA9685 I2C to PWM controller
  class PCA9685 : public Module<pca9685_settings>
  {
  public:
    /// Map of all of the used device addresses in this driver.
    enum class RegisterMap : uint8_t
    {
      kModeRegister1 = 0x00,
      kAllOutputAddress = 0x05,
      kOutputAddress0 = 0x06,
      kPreScaler = 0xFE
    };

    explicit constexpr PCA9685(I2c &i2c, uint8_t address = 0x40)
        : i2c_(i2c), kAddress(address)
    {
      ModuleInitialize();
    };

    void ModuleInitialize() override
    {
      i2c_.Initialize();
      SetFrequency(50_Hz);
      EnableDevice(true);
    }

    void SetFrequency(units::frequency::hertz_t output_frequency)
    {
      uint8_t scale_value = static_cast<uint8_t>(round((25_MHz / (4096 * output_frequency)).to<float>()) - 1);
      i2c_.Write(kAddress, {static_cast<uint8_t>(Value(RegisterMap::kPreScaler)), scale_value});
    }

    void SetPulseWidth(uint8_t output_number, units::time::second_t pulse_length)
    {
      uint16_t off_time = (pulse_length * settings.frequency * 4096).to<uint16_t>();
      setPWM(output_number, 0, off_time);
    }

    void SetAngle(uint8_t output_number, units::angle::degree_t angle)
    {
      const int min_pulse_width = static_cast<int>(settings.min_pulse.count());
      const int max_pulse_width = static_cast<int>(settings.max_pulse.count());
      const int pulse_width_difference = max_pulse_width - min_pulse_width;
      auto angle_to_pwn = std::chrono::microseconds((angle * pulse_width_difference / settings.max_angle) + min_pulse_width);
      SetPulseWidth(output_number, angle_to_pwn);
    }

  private:
    void EnableDevice(bool is_active = true)
    {
      uint8_t mode_register_data;
      constexpr auto kSleepMask = bit::MaskFromRange(4);

      i2c_.WriteThenRead(kAddress, {Value(RegisterMap::kModeRegister1)}, &mode_register_data, 1);
      mode_register_data = bit::Insert(mode_register_data, !is_active, kSleepMask);
      i2c_.Write(kAddress, {Value(RegisterMap::kModeRegister1), mode_register_data});
    }

    void setPWM(uint8_t output_number, uint16_t on_time, uint16_t off_time)
    {
      uint8_t on_time_high = static_cast<uint8_t>(on_time >> 8);
      uint8_t on_time_low = static_cast<uint8_t>(on_time & 0xFF);
      uint8_t off_time_high = static_cast<uint8_t>(off_time >> 8);
      uint8_t off_time_low = static_cast<uint8_t>(off_time & 0xFF);

      i2c_.Write(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0)), on_time_low});
      i2c_.Write(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 1), on_time_high});
      i2c_.Write(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 2), off_time_low});
      i2c_.Write(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 3), off_time_high});
    }

    I2c &i2c_;
    const uint8_t kAddress;
  };
} // namespace sjsu