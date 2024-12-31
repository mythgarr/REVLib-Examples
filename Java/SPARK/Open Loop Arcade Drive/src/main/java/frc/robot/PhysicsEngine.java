package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhysicsEngine {
    private static final double kDeltaTime = 1.0 / 50;
    private final TimedRobot _robot;
    private final SparkMaxSim _leftSim;
    private final SparkMaxSim _rightSim;
    private final DifferentialDrivetrainSim _drivetrainSim;
    private final Field2d _simField;

    public PhysicsEngine(
        TimedRobot robot,
        SparkMax leftLeader,
        SparkMax rightLeader
    )
    {
        _robot = robot;
        _leftSim = new SparkMaxSim(leftLeader, DCMotor.getNEO(1));
        _rightSim = new SparkMaxSim(rightLeader, DCMotor.getNEO(1));
        _drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDoubleNEOPerSide,
            KitbotGearing.k10p71,
            KitbotWheelSize.kSixInch,
            null);
        _simField = new Field2d();
        SmartDashboard.putData("Field", _simField);
    }

    public void updateSim() {
        double vBus = RobotController.getBatteryVoltage();
        if (_robot.isEnabled()) {
          double leftOutput = _leftSim.getAppliedOutput();
          double rightOutput = _rightSim.getAppliedOutput();
          _drivetrainSim.setInputs(
              leftOutput * vBus,
              rightOutput * vBus);
          _drivetrainSim.update(kDeltaTime);
          _leftSim.iterate(
              _drivetrainSim.getLeftVelocityMetersPerSecond(),
              vBus,
              kDeltaTime);
          _rightSim.iterate(
              _drivetrainSim.getRightVelocityMetersPerSecond(),
              vBus,
              kDeltaTime);
        } else {
          _drivetrainSim.setInputs(0, 0);
          _drivetrainSim.update(vBus);
          _leftSim.iterate(
              0,
              vBus,
              kDeltaTime);
          _rightSim.iterate(
              0,
              vBus,
              kDeltaTime);
        }
        _simField.setRobotPose(_drivetrainSim.getPose());
    }

}
