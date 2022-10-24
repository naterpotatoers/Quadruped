#include "devices/actuators/servo/servo.hpp"
#include "peripherals/lpc40xx/pwm.hpp"
#include "utility/log.hpp"
#include "utility/time/time.hpp"
#include "peripherals/lpc40xx/i2c.hpp"
#include "../pca9685.hpp"

int main()
{
  sjsu::LogInfo("Servo application starting...");
  sjsu::lpc40xx::I2c &i2c = sjsu::lpc40xx::GetI2c<2>();
  sjsu::Pca9685 pca9685(i2c, 0x40);
  sjsu::LogInfo("Initializing PCA...");
  pca9685.ModuleInitialize();

  while (1)
  {
    sjsu::LogInfo("Moving to first angle: %d", 0);
    pca9685.SetAngle(15, 0_deg);
    pca9685.SetAngle(12, 0_deg);
    pca9685.SetAngle(11, 0_deg);
    sjsu::Delay(2s);
    sjsu::LogInfo("Moving to second angle: %d", 180);
    pca9685.SetAngle(15, 180_deg);
    pca9685.SetAngle(12, 180_deg);
    pca9685.SetAngle(11, 180_deg);
    sjsu::Delay(2s);
  }
  return 0;
}
