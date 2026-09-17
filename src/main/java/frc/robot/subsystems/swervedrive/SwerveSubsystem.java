// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.BenchTestConstants;
import frc.robot.Constants.DriveConstants;
import java.io.File;
import java.util.Arrays;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The drivetrain. YAGSL builds the whole swerve drive from the JSON config folder; this class adds
 * driver commands, Test-mode bench commands, live tuning, and health alerts on top of it.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final SwerveHealth health;
  private final SwerveTuning tuning;

  // What the bench step tests are currently asking for, so dashboards can graph setpoint vs actual.
  private double benchSteerSetpointDegrees = 0;
  private double benchDriveSetpointMetersPerSecond = 0;

  public SwerveSubsystem(File configDirectory) {
    // HIGH publishes every module's setpoints and sensor readings under SmartDashboard/swerve/.
    // Turn it down to LOW once the robot is tuned, to save roboRIO CPU and network.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(configDirectory)
              .createSwerveDrive(DriveConstants.MAX_SPEED_METERS_PER_SECOND, new Pose2d());
    } catch (Exception e) {
      // A bad JSON file lands here. The message names the file and field that failed.
      throw new RuntimeException("YAGSL could not load " + configDirectory, e);
    }

    // Heading correction only helps when driving by target angle; we drive by rotation speed.
    swerveDrive.setHeadingCorrection(false);
    // Cosine compensation slows a wheel that is still turning toward its angle. It misbehaves in sim.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    // Corrects the curve a robot drifts into when translating and rotating at once.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    // Off: the absolute encoders seed the steer motors once at boot, which is enough to start.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    health = new SwerveHealth(swerveDrive);
    tuning = new SwerveTuning(swerveDrive);
  }

  @Override
  public void periodic() {
    health.update();
    SmartDashboard.putNumber("Drive/Heading (deg)", getPose().getRotation().getDegrees());
    publishBenchGraphs();
  }

  // ---------------------------------------------------------------------------------------------
  // Driving
  // ---------------------------------------------------------------------------------------------

  /**
   * Drives with speeds from a {@link swervelib.SwerveInputStream}. The stream hands back
   * field-relative speeds, including when its robot-relative toggle is on.
   */
  public Command driveCommand(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    return run(() -> swerveDrive.driveFieldOriented(fieldRelativeSpeeds.get()))
        .withName("Drive");
  }

  /** Makes the direction the robot faces right now "field forward". Works while disabled. */
  public Command zeroGyroCommand() {
    return runOnce(swerveDrive::zeroGyro).ignoringDisable(true).withName("Zero Gyro");
  }

  /** Points the wheels in an X so the robot is hard to push. Hold the button. */
  public Command lockCommand() {
    return run(swerveDrive::lockPose).withName("X-Lock");
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public SwerveTuning getTuning() {
    return tuning;
  }

  // ---------------------------------------------------------------------------------------------
  // Bench tests: run these with the robot ON BLOCKS. Each one isolates one kind of problem.
  // ---------------------------------------------------------------------------------------------

  /**
   * Steers every wheel to the same angle with the drive motors stopped. 0 = straight forward, 90 =
   * pointing left. A wheel that disagrees has a wrong offset or angle inversion.
   */
  public Command pointWheelsCommand(double degrees) {
    return run(() -> setAllWheelAngles(degrees))
        .finallyDo(this::stopModules)
        .withName("Point Wheels " + degrees);
  }

  /**
   * Drives robot-relative at a fixed slow speed. Forward: every wheel's front edge should roll the
   * same way. Spin: the module arrows in AdvantageScope should form a clean counterclockwise circle.
   */
  public Command driveRobotRelativeCommand(double forwardMetersPerSecond, double ccwRadiansPerSecond) {
    return run(() ->
            swerveDrive.drive(
                new Translation2d(forwardMetersPerSecond, 0), ccwRadiansPerSecond, false, false))
        .finallyDo(this::stopModules)
        .withName("Bench Drive");
  }

  /** Steps every wheel through 0, 90, 180, 270 degrees, over and over. Graph it to tune steer PID. */
  public Command steerStepTestCommand() {
    Command[] steps =
        Arrays.stream(BenchTestConstants.STEER_STEP_ANGLES_DEGREES)
            .mapToObj(
                angle ->
                    run(() -> {
                          benchSteerSetpointDegrees = angle;
                          setAllWheelAngles(angle);
                        })
                        .withTimeout(BenchTestConstants.STEP_SECONDS))
            .toArray(Command[]::new);
    return Commands.sequence(steps).repeatedly().finallyDo(this::stopModules).withName("Steer Step Test");
  }

  /** Drives forward at 0, 1, 2, then 0 m/s. Graph it to tune drive PIDF. ON BLOCKS: it goes fast. */
  public Command driveStepTestCommand() {
    Command[] steps =
        Arrays.stream(BenchTestConstants.DRIVE_STEP_SPEEDS_METERS_PER_SECOND)
            .mapToObj(
                speed ->
                    run(() -> {
                          benchDriveSetpointMetersPerSecond = speed;
                          swerveDrive.drive(new Translation2d(speed, 0), 0, false, false);
                        })
                        .withTimeout(BenchTestConstants.STEP_SECONDS))
            .toArray(Command[]::new);
    return Commands.sequence(steps).finallyDo(this::stopModules).withName("Drive Step Test");
  }

  /** SysId characterization of the drive motors (for kS/kV/kA later). ON BLOCKS or open floor. */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /** "frontleft" for the module loaded from frontleft.json (YAGSL keeps the ".json" in the name). */
  static String moduleName(SwerveModule module) {
    return module.configuration.name.replace(".json", "");
  }

  private void setAllWheelAngles(double degrees) {
    for (SwerveModule module : swerveDrive.getModules()) {
      module.getDriveMotor().set(0);
      module.setAngle(degrees);
    }
  }

  private void stopModules() {
    benchDriveSetpointMetersPerSecond = 0;
    for (SwerveModule module : swerveDrive.getModules()) {
      module.getDriveMotor().set(0);
    }
  }

  /**
   * Publishes setpoint and measured values in shapes that graph cleanly:
   *
   * <ul>
   *   <li>steer angles are unwrapped around the setpoint, so 359 vs 1 degree graphs as a 2 degree
   *       gap instead of a jump;
   *   <li>drive speed is the forward component, so a wheel that YAGSL flipped 180 degrees still
   *       graphs as positive.
   * </ul>
   */
  private void publishBenchGraphs() {
    SmartDashboard.putNumber("Bench/Steer Setpoint (deg)", benchSteerSetpointDegrees);
    SmartDashboard.putNumber("Bench/Drive Setpoint (mps)", benchDriveSetpointMetersPerSecond);
    for (SwerveModule module : swerveDrive.getModules()) {
      String prefix = "Bench/" + moduleName(module) + "/";
      double measuredDegrees = module.getAbsolutePosition();
      double unwrapped =
          benchSteerSetpointDegrees
              + MathUtil.inputModulus(measuredDegrees - benchSteerSetpointDegrees, -180, 180);
      SmartDashboard.putNumber(prefix + "Steer Angle (deg)", unwrapped);

      SwerveModuleState state = module.getState();
      SmartDashboard.putNumber(
          prefix + "Drive Forward Speed (mps)", state.speedMetersPerSecond * state.angle.getCos());
    }
  }
}
