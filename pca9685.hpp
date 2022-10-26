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
    std::chrono::microseconds min_pulse = 500us;
    std::chrono::microseconds max_pulse = 2400us;
  };

  /// Driver for the PCA9685 I2C to PWM controller
  class Pca9685 : public Module<pca9685_settings>
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

    explicit constexpr Pca9685(I2c &i2c, uint8_t address = 0x40)
        : i2c_(i2c), kAddress(address){};

    void ModuleInitialize() override
    {
      i2c_.Initialize();
      SetFrequency(50_Hz);
      EnableDevice(true);
    }

    void ModulePowerDown() override
    {
      EnableDevice(false);
    }

    void SetFrequency(units::frequency::hertz_t output_frequency)
    {
      uint8_t scale_value = static_cast<uint8_t>(round((25_MHz / (4096 * output_frequency)).to<float>()) - 1);
      i2c_.Write(kAddress, {static_cast<uint8_t>(Value(RegisterMap::kPreScaler)), scale_value});
    }

    // The delay can be used to stagger the outputs, to minimize current spikes.
    void SetDutyCycle(uint8_t output_number, float duty_cycle, float delay = 0.0f)
    {
      if (delay + duty_cycle > 1)
      {
        throw Exception(std::errc::invalid_argument, "The sum of the delay and duty cycle cannot be greater than 100%");
      }
      uint16_t on_time = 0;
      uint16_t off_time = static_cast<uint16_t>((4096.f * duty_cycle + 0.5f) + on_time);
      setPWM(output_number, on_time, off_time);
    }

    void SetPulseWidth(uint8_t output_number, units::time::second_t pulse_length)
    {
      uint16_t off_time = (pulse_length * settings.frequency * 4096).to<uint16_t>();
      setPWM(output_number, 0, off_time);
    }

    void SetAngle(uint8_t output_number, units::angle::degree_t angle)
    {
      const int pulseWidthRange = static_cast<int>((settings.max_pulse - settings.min_pulse).count());
      const int minPulseWidth = static_cast<int>(settings.min_pulse.count());
      auto angleToPwm = std::chrono::microseconds((angle * pulseWidthRange / settings.max_angle) + minPulseWidth);
      SetPulseWidth(output_number, angleToPwm);
    }

    void fullOn(uint8_t output_number)
    {
      if (output_number > 15)
      {
        throw Exception(std::errc::invalid_argument, "Invalid output number - pin must be between 0-15");
      }

      constexpr auto kOperateBit = bit::MaskFromRange(4);
      uint8_t onData;
      uint8_t off_data;

      i2c_.WriteThenRead(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 1)}, &onData, 1);
      i2c_.WriteThenRead(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 3)}, &off_data, 1);

      onData = bit::Insert(onData, true, kOperateBit);
      off_data = bit::Insert(off_data, false, kOperateBit);

      i2c_.Write(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 3), off_data});
      i2c_.Write(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 1), onData});
    }

    void fullOff(uint8_t output_number)
    {
      if (output_number > 15)
      {
        throw Exception(std::errc::invalid_argument, "Invalid output number - pin must be between 0-15");
      }
      constexpr auto kOperateBit = bit::MaskFromRange(4);

      uint8_t off_data;
      i2c_.WriteThenRead(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 3)}, &off_data, 1);
      off_data = bit::Insert(off_data, true, kOperateBit); // Enable full off
      i2c_.Write(kAddress, {static_cast<uint8_t>(4 * output_number + Value(RegisterMap::kOutputAddress0) + 3), off_data});
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