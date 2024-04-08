#include "Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
    frc::SmartDashboard::PutData("shooter", (wpi::Sendable*)&_motor);
    _motor.SetVoltage(10_V);
};

void Shooter::Periodic() {
    auto voltageApplied = _motor.GetSimVoltage();
    units::radians_per_second_t measuredVelocity = _motor.GetVelocity();
    _observer.Correct(frc::Vectord<1>{voltageApplied.value()}, frc::Vectord<1>{measuredVelocity.value()});
    _observer.Predict(frc::Vectord<1>{voltageApplied.value()}, 20_ms);
    _estimatedVelocity = _observer.Xhat(0) * 1_rad_per_s;
    frc::SmartDashboard::PutNumber("estimated velocity", _estimatedVelocity.value());
}

frc2::CommandPtr Shooter::Shoot() {
  return RunOnce([&] {
    _flywheelSim.SetState(_motor.GetVelocity() - 10_tps); // tell the physics sim to slow down
    _observer.SetXhat(0, _observer.Xhat(0)-62.8); // tell the state observer model that it's slowing down 10_tps (62.8 rad)
  });
}

void Shooter::SimulationPeriodic() {
    _flywheelSim.SetInputVoltage(_motor.GetSimVoltage());
    _flywheelSim.Update(20_ms);
    _motor.UpdateSimEncoder(0_tr, _flywheelSim.GetAngularVelocity());
}