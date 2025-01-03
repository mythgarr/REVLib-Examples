// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import java.util.Random;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MAXSwerveModule;

public class SwerveDriveSim {
  class SwerveModuleSim {
    private final SparkMaxSim m_driveSparkSim;
    private final SparkMaxSim m_turnSparkSim;
    private final SlewRateLimiter m_driveRateLimiter;
    private final PIDController m_turnController;

    private final Angle m_angularOffset;

    public SwerveModuleSim(MAXSwerveModule module) {
      var accessor = module.getSimAccessor();
      m_driveSparkSim = new SparkMaxSim(accessor.driving(), DCMotor.getNEO(1));
      m_turnSparkSim = new SparkMaxSim(accessor.turning(), DCMotor.getNEO(1));
      m_angularOffset = accessor.chassisOffset();

      // Calculates the velocity of the drive motor, simulating inertia and friction
      m_driveRateLimiter = new SlewRateLimiter(Constants.ModuleConstants.kdrivingMotorSimSlew);

      // Calculates the velocity of the turn motor
      m_turnController = new PIDController(
          Constants.ModuleConstants.kturningMotorSimSpeed,
          0,
          Constants.ModuleConstants.kturningMotorSimD);
      m_turnController.enableContinuousInput(-Math.PI, Math.PI);

      // Randomize starting rotation
      m_turnSparkSim.setPosition(new Random().nextDouble(-Math.PI, Math.PI));
    }

    public void simulationPeriodic(double vbus, double dt) {
      // m/s
      double targetVelocity = m_driveSparkSim.getSetpoint();
      double driveVelocity = m_driveRateLimiter.calculate(targetVelocity);
      m_driveSparkSim.iterate(
          driveVelocity,
          vbus,
          dt);

      // rad
      double targetAngle = m_turnSparkSim.getSetpoint();
      double curAngle = MathUtil.angleModulus(m_turnSparkSim.getAbsoluteEncoderSim().getPosition());
      double turnVelocity = m_turnController.calculate(curAngle, targetAngle);
      m_turnSparkSim.iterate(
          turnVelocity,
          vbus,
          dt);
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(
          Units.MetersPerSecond.of(m_driveSparkSim.getVelocity()),
          Rotation2d.fromRadians(m_turnSparkSim.getPosition() - m_angularOffset.in(Units.Radians)));
    }
  }

  private final ADIS16470_IMUSim m_gyroSim;
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveModuleSim[] m_modules;

  private final SwerveModuleState[] m_moduleStates;
  private final StructPublisher<Pose2d> m_posePublisher;

  private static final Translation2d kFieldSize = new Translation2d(16.49, 8.10);

  private Pose2d m_pose;

  public SwerveDriveSim(DriveSubsystem.SimAccessor accessor) {
    m_gyroSim = new ADIS16470_IMUSim(accessor.gyro());

    var modules = accessor.modules();
    m_kinematics = Constants.DriveConstants.kDriveKinematics;
    m_modules = new SwerveModuleSim[modules.length];
    m_moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      m_modules[i] = new SwerveModuleSim(modules[i]);
    }
    m_pose = new Pose2d();

    // Publish the simulated pose. This is the ACTUAL pose of the robot - the drive
    // system pose is an estimation
    // based on encoder and vision (if applicable) data.
    m_posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Sim/Pose", Pose2d.struct)
        .publish();
    m_posePublisher.set(m_pose);
  }

  public void simulationPeriodic(double dt) {
    double vbus = RobotController.getBatteryVoltage();
    for (int i = 0; i < m_modules.length; i++) {
      SwerveModuleSim module = m_modules[i];
      module.simulationPeriodic(vbus, dt);
      m_moduleStates[i] = module.getState();
    }

    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(m_moduleStates);
    Twist2d twist = chassisSpeeds.toTwist2d(dt);
    // Constrain the pose within the Field
    m_pose = clampPose(m_pose.exp(twist));

    // Publish the simulated pose
    m_posePublisher.set(m_pose);

    // Update the gyro heading
    m_gyroSim.setGyroAngleZ(m_pose.getRotation().getDegrees());
  }

  private Pose2d clampPose(Pose2d pose) {
    Translation2d translation = pose.getTranslation();
    translation = new Translation2d(
        MathUtil.clamp(translation.getX(), 0, kFieldSize.getX()),
        MathUtil.clamp(translation.getY(), 0, kFieldSize.getY()));
    return new Pose2d(translation, pose.getRotation());
  }

  /**
   * Returns the current simulated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_pose;
  }

  /**
   * Sets the current simulated pose of the robot.
   * @param pose The pose.
   */
  public void setPose(Pose2d pose) {
    m_pose = pose;
  }
}
