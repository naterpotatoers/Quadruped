#include "utility/log.hpp"
#include "utility/time/time.hpp"
#include "peripherals/lpc40xx/pwm.hpp"
#include "peripherals/lpc40xx/i2c.hpp"
#include "peripherals/lpc40xx/uart.hpp"
#include "devices/actuators/servo/servo.hpp"

#include "../pca9685.hpp"
#include "../serial.hpp"

static const uint8_t kFrontLeftKnee = 8;
static const uint8_t kFrontRightKnee = 9;
static const uint8_t kBackLeftKnee = 10;
static const uint8_t kBackRightKnee = 11;

static const uint8_t kFrontLeftHip = 12;
static const uint8_t kFrontRightHip = 13;
static const uint8_t kBackLeftHip = 14;
static const uint8_t kBackRightHip = 15;

int main()
{
  sjsu::PCA9685 pca9685(sjsu::lpc40xx::GetI2c<2>());
  sjsu::common::Serial serial(sjsu::lpc40xx::GetUart<0>());
  units::angle::degree_t hip_angle = 0_deg;
  units::angle::degree_t knee_angle = 0_deg;

  sjsu::LogInfo("Setting all servos to zero...");
  for (int8_t i = 0; i <= 15; i++)
  {
    pca9685.SetAngle(i, 0_deg);
  }

  sjsu::LogInfo("Starting main application in 3 seconds...");
  sjsu::Delay(3s);

  while (1)
  {
    std::string commands = serial.GetCommands();
    if (commands != "")
    {
      sjsu::LogInfo("Received commands: %s", commands.c_str());
      int first_angle = std::stoi(commands.substr(0, commands.find(",")));
      int second_angle = std::stoi(commands.substr(commands.find(",") + 1));
      hip_angle = units::angle::degree_t(static_cast<float>(first_angle));
      knee_angle = units::angle::degree_t(static_cast<float>(second_angle));
    }
    sjsu::LogInfo("Moving to angle: %d", hip_angle.to<int>());
    pca9685.SetAngle(kFrontLeftHip, hip_angle);
    pca9685.SetAngle(kFrontRightHip, hip_angle);
    pca9685.SetAngle(kBackLeftHip, hip_angle);
    pca9685.SetAngle(kBackRightHip, hip_angle);
    sjsu::Delay(1s);
  }
  return 0;
}
