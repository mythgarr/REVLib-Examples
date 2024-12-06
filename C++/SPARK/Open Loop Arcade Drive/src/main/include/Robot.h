// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include <rev/SparkMax.h>

using namespace rev::spark;

class Robot : public frc::TimedRobot {
private:
  // Initialize the SPARKs
  SparkMax m_leftLeader{1, SparkMax::MotorType::kBrushless};
  SparkMax m_leftFollower{2, SparkMax::MotorType::kBrushless};
  SparkMax m_rightLeader{3, SparkMax::MotorType::kBrushless};
  SparkMax m_rightFollower{4, SparkMax::MotorType::kBrushless};

  // Initialize joystick
  frc::XboxController joystick{0};

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
