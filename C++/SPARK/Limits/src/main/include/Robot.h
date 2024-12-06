// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include <rev/SparkMax.h>

using namespace rev::spark;

class Robot : public frc::TimedRobot {
private:
  /*
   * Initialize the SPARK MAX and get its limit switch and encoder objects for
   * later use.
   */
  SparkMax m_motor{1, SparkMax::MotorType::kBrushless};
  SparkLimitSwitch m_forwardLimitSwitch = m_motor.GetForwardLimitSwitch();
  SparkLimitSwitch m_reverseLimitSwitch = m_motor.GetReverseLimitSwitch();
  SparkRelativeEncoder m_encoder = m_motor.GetEncoder();

public:
  Robot();
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};
