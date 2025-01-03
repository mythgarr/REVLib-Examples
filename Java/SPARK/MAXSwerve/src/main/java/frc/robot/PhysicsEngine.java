// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.SwerveDriveSim;
import frc.robot.subsystems.DriveSubsystem;

public class PhysicsEngine {
  private static final double kDeltaTime = 1.0 / 50;
  private final TimedRobot m_robot;
  private final SwerveDriveSim m_drivetrainSim;
  private final Field2d m_simField;

  public PhysicsEngine(
      TimedRobot robot,
      DriveSubsystem driveSubsystem) {
    m_robot = robot;
    m_drivetrainSim = new SwerveDriveSim(driveSubsystem.getSimAccessor());
    m_simField = new Field2d();

    // Broadcast the Field with the same name used by robotpy PhysicsEngine
    SmartDashboard.putData("Field", m_simField);
  }

  public void updateSim() {
    if (m_robot.isEnabled()) {
      m_drivetrainSim.simulationPeriodic(kDeltaTime);
      m_simField.setRobotPose(m_drivetrainSim.getPose());
    }
  }
}
