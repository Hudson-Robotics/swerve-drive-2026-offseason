// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.BenchTestConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveTuning;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * Driver controls (Xbox, port 0):
 *
 * <ul>
 *   <li>Left stick: drive (field-relative: "up" is away from the driver)
 *   <li>Right stick X: rotate
 *   <li>Right bumper (hold): slow mode
 *   <li>Left bumper (hold): X-lock the wheels
 *   <li>Start: zero gyro. Point the robot AWAY from the driver first.
 *   <li>Back: toggle robot-relative (the fallback if the gyro is wrong)
 * </ul>
 */
public class RobotContainer {
  private static final String SPEED_CAP_KEY = "Drive/Speed Cap";

  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), DriveConstants.SWERVE_CONFIG_DIR));

  private boolean robotRelative = false;

  /**
   * Turns sticks into chassis speeds. Signs: pushing a stick up or left reads negative, but the
   * robot's +x is forward, +y is left and +rotation is counterclockwise, so all three are negated.
   *
   * <p>Alliance-relative control is OFF on purpose: at practice the Driver Station's alliance
   * setting is arbitrary, and "forward = away from the driver after zeroing" is what drivers expect.
   */
  private final SwerveInputStream driveInput =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> shapeAxis(-driverXbox.getLeftY()),
              () -> shapeAxis(-driverXbox.getLeftX()))
          .withControllerRotationAxis(() -> shapeAxis(-driverXbox.getRightX()))
          .robotRelative(() -> robotRelative)
          .allianceRelativeControl(false);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SmartDashboard.putNumber(SPEED_CAP_KEY, DriveConstants.DEFAULT_SPEED_CAP);
    configureBindings();
    configureDashboard();
  }

  /**
   * Deadband first, then scale by the speed cap (and slow mode). Doing it in this order keeps the
   * deadband the same size no matter how low the cap is.
   */
  private double shapeAxis(double axis) {
    double cap = MathUtil.clamp(SmartDashboard.getNumber(SPEED_CAP_KEY, DriveConstants.DEFAULT_SPEED_CAP), 0, 1);
    if (driverXbox.getHID().getRightBumperButton()) {
      cap *= DriveConstants.SLOW_MODE_SCALE;
    }
    return MathUtil.applyDeadband(axis, OperatorConstants.DEADBAND) * cap;
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(drivebase.driveCommand(driveInput));

    driverXbox.start().onTrue(drivebase.zeroGyroCommand());
    driverXbox.back().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative).ignoringDisable(true));
    driverXbox.leftBumper().whileTrue(drivebase.lockCommand());
  }

  /**
   * Buttons on the dashboard (Elastic: add them from the "SmartDashboard" tree). Click once to start
   * a command and click again to stop it. Disabling the robot also stops them.
   */
  private void configureDashboard() {
    SmartDashboard.putData("Drive/Zero Gyro", drivebase.zeroGyroCommand());

    // Bench tests: robot ON BLOCKS. Numbered in the order the bring-up checklist uses them.
    SmartDashboard.putData("Bench/1 Point Wheels Forward", drivebase.pointWheelsCommand(0));
    SmartDashboard.putData("Bench/2 Point Wheels Left (90)", drivebase.pointWheelsCommand(90));
    SmartDashboard.putData(
        "Bench/3 Drive Forward Slow",
        drivebase.driveRobotRelativeCommand(BenchTestConstants.SLOW_DRIVE_METERS_PER_SECOND, 0));
    SmartDashboard.putData(
        "Bench/4 Spin CCW Slow",
        drivebase.driveRobotRelativeCommand(0, BenchTestConstants.SLOW_SPIN_RADIANS_PER_SECOND));
    SmartDashboard.putData("Bench/5 Steer Step Test", drivebase.steerStepTestCommand());
    SmartDashboard.putData("Bench/6 Drive Step Test", drivebase.driveStepTestCommand());

    SwerveTuning tuning = drivebase.getTuning();
    SmartDashboard.putData("Tuning/Capture Offsets", tuning.captureOffsetsCommand());
    SmartDashboard.putData("Tuning/Apply PIDF", tuning.applyPidfCommand());
    SmartDashboard.putData("Tuning/Print PIDF JSON", tuning.printPidfJsonCommand());
  }

  /** Publishes driver-mode state for the dashboard. Called every loop from Robot. */
  public void updateDashboard() {
    SmartDashboard.putBoolean("Drive/Robot Relative", robotRelative);
    SmartDashboard.putBoolean("Drive/Slow Mode", driverXbox.getHID().getRightBumperButton());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
