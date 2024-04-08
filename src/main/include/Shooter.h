#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <utilities/ICSparkMax.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc2/command/Commands.h>
#include <frc/StateSpaceUtil.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::CommandPtr Shoot();

 private:
  ICSparkMax _motor{1, 40_A};

  frc::LinearSystem<1, 1, 1> _flywheelSystem =
      frc::LinearSystemId::FlywheelSystem(frc::DCMotor::NEO(), 0.01_kg_sq_m, 1);
  frc::KalmanFilter<1, 1, 1> _observer{
      _flywheelSystem,
      {3},     // model std dev
      {1},     // encoder std dev
      20_ms    // timesteps
  };
  units::turns_per_second_t _estimatedVelocity = 0_rpm;

  frc::sim::FlywheelSim _flywheelSim{_flywheelSystem, frc::DCMotor::NEO(), 1, {10}};
};
