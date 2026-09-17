// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;

/**
 * Live tuning from the dashboard, so tuning doesn't need a redeploy.
 *
 * <p>How to use it:
 *
 * <ol>
 *   <li>Edit the numbers under Tuning/Drive and Tuning/Steer.
 *   <li>Press "Tuning/Apply PIDF". It sends the gains to all 8 SPARK MAXes.
 *   <li>When it's good, press "Tuning/Print PIDF JSON", copy the block from the Driver Station
 *       console into src/main/deploy/swerve/zombie-ricky/modules/pidfproperties.json, and commit.
 * </ol>
 *
 * <p>Dashboard numbers reset to the JSON values on every reboot. The JSON file is the real source
 * of truth.
 */
public class SwerveTuning {
  private static final String DRIVE = "Tuning/Drive/";
  private static final String STEER = "Tuning/Steer/";

  private final SwerveDrive swerveDrive;

  SwerveTuning(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    SwerveModule first = swerveDrive.getModules()[0];
    publish(DRIVE, first.getDrivePIDF());
    publish(STEER, first.getAnglePIDF());
  }

  /**
   * Applies the dashboard gains to every module. Works while disabled, which is also the only time
   * the SPARK MAX saves them to its own memory (while enabled they last until it reboots).
   */
  public Command applyPidfCommand() {
    return Commands.runOnce(this::applyPidf).ignoringDisable(true).withName("Apply PIDF");
  }

  /** Prints the current dashboard gains as a ready-to-paste pidfproperties.json. */
  public Command printPidfJsonCommand() {
    return Commands.runOnce(this::printPidfJson).ignoringDisable(true).withName("Print PIDF JSON");
  }

  /**
   * Robot DISABLED, all four wheels straightened by hand (bevel gears on the LEFT): reads every
   * CANcoder and prints the absoluteEncoderOffset for each module file.
   */
  public Command captureOffsetsCommand() {
    return Commands.runOnce(this::captureOffsets).ignoringDisable(true).withName("Capture Offsets");
  }

  private void applyPidf() {
    for (SwerveModule module : swerveDrive.getModules()) {
      module.setDrivePIDF(read(DRIVE, module.getDrivePIDF()));
      module.setAnglePIDF(read(STEER, module.getAnglePIDF()));
      // setDrivePIDF/setAnglePIDF only update YAGSL's copy; burnFlash() actually sends it.
      module.getDriveMotor().burnFlash();
      module.getAngleMotor().burnFlash();
    }
    System.out.println("[Tuning] Applied PIDF to all modules.");
    printPidfJson();
  }

  private void printPidfJson() {
    SwerveModule first = swerveDrive.getModules()[0];
    PIDFConfig drive = read(DRIVE, first.getDrivePIDF());
    PIDFConfig steer = read(STEER, first.getAnglePIDF());
    System.out.println(
        "[Tuning] pidfproperties.json:\n{\n"
            + "  \"drive\": " + json(drive) + ",\n"
            + "  \"angle\": " + json(steer) + "\n}");
  }

  private void captureOffsets() {
    StringBuilder out = new StringBuilder("[Tuning] Offsets (wheels must be straight, bevels LEFT):\n");
    for (SwerveModule module : swerveDrive.getModules()) {
      String name = SwerveSubsystem.moduleName(module);
      if (module.getAbsoluteEncoder() == null) {
        out.append("  ").append(name).append(": no absolute encoder\n");
        continue;
      }
      // The raw CANcoder angle while the wheel points forward is the offset YAGSL subtracts.
      double raw = module.getAbsoluteEncoder().getAbsolutePosition();
      String value = String.format("%.2f", raw);
      if (module.getAbsoluteEncoder().readingError) {
        value += "  <-- BAD READING, do not use";
      }
      SmartDashboard.putString("Tuning/Offsets/" + name, value);
      out.append("  ").append(name).append(".json  \"absoluteEncoderOffset\": ").append(value).append('\n');
    }
    System.out.println(out);
  }

  private static void publish(String prefix, PIDFConfig config) {
    SmartDashboard.putNumber(prefix + "P", config.p);
    SmartDashboard.putNumber(prefix + "I", config.i);
    SmartDashboard.putNumber(prefix + "D", config.d);
    SmartDashboard.putNumber(prefix + "F", config.f);
    SmartDashboard.putNumber(prefix + "IZone", config.iz);
  }

  /** Reads the dashboard gains, keeping the output range from the current config. */
  private static PIDFConfig read(String prefix, PIDFConfig current) {
    PIDFConfig config =
        new PIDFConfig(
            SmartDashboard.getNumber(prefix + "P", current.p),
            SmartDashboard.getNumber(prefix + "I", current.i),
            SmartDashboard.getNumber(prefix + "D", current.d),
            SmartDashboard.getNumber(prefix + "F", current.f),
            SmartDashboard.getNumber(prefix + "IZone", current.iz));
    config.output = current.output;
    return config;
  }

  private static String json(PIDFConfig c) {
    return String.format(
        "{\"p\": %s, \"i\": %s, \"d\": %s, \"f\": %s, \"iz\": %s}", c.p, c.i, c.d, c.f, c.iz);
  }
}
