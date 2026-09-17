// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;

/**
 * Turns "the robot drives weird" into a named device. Every problem shows up as an Alert (Elastic
 * and AdvantageScope list them under "Alerts") that says which module and which CAN ID to look at.
 *
 * <p>YAGSL already raises its own alerts for CANcoder magnet health and bad readings; this class
 * adds SPARK MAX errors and faults, the navX, and one summary light: Health/All OK.
 */
class SwerveHealth {
  private static final String GROUP = "Swerve Health";

  /** One check: a condition that means trouble, and the alert shown while it is true. */
  private record Check(BooleanSupplier isBad, Alert alert) {}

  private final List<Check> checks = new ArrayList<>();

  SwerveHealth(SwerveDrive swerveDrive) {
    for (SwerveModule module : swerveDrive.getModules()) {
      String name = SwerveSubsystem.moduleName(module);
      addMotorChecks(name + " drive", module.getDriveMotor());
      addMotorChecks(name + " steer", module.getAngleMotor());
      add(
          module::getAbsoluteEncoderReadIssue,
          name + ": absolute encoder (CANcoder) reading failed. Check its CAN ID, wiring and magnet.",
          AlertType.kError);
    }

    if (swerveDrive.getGyro().getIMU() instanceof AHRS navx) {
      add(
          () -> !navx.isConnected(),
          "navX not connected. Field-relative driving will not work; use robot-relative (Back).",
          AlertType.kError);
      add(
          navx::isCalibrating,
          "navX calibrating. Keep the robot still until this goes away.",
          AlertType.kWarning);
    }
  }

  private void addMotorChecks(String label, SwerveMotor swerveMotor) {
    if (!(swerveMotor.getMotor() instanceof SparkBase spark)) {
      return;
    }
    String who = label + " SPARK MAX (CAN " + spark.getDeviceId() + ")";

    // Errors from talking to the device: missing, duplicate ID, or firmware that doesn't match REVLib.
    Alert errorAlert = new Alert(GROUP, who + ": communication error", AlertType.kError);
    checks.add(
        new Check(
            () -> {
              REVLibError error = spark.getLastError();
              boolean bad =
                  error == REVLibError.kCANDisconnected
                      || error == REVLibError.kTimeout
                      || error == REVLibError.kDuplicateCANId
                      || error == REVLibError.kInvalidCANId
                      || error == REVLibError.kCantFindFirmware
                      || error == REVLibError.kFirmwareTooOld
                      || error == REVLibError.kFirmwareTooNew;
              if (bad) {
                errorAlert.setText(who + ": " + error + ". Check CAN wiring, ID and firmware.");
              }
              return bad;
            },
            errorAlert));

    // Faults the SPARK MAX reports about itself (sensor cable, overheating, CAN, gate driver...).
    Alert faultAlert = new Alert(GROUP, who + ": active fault", AlertType.kError);
    checks.add(
        new Check(
            () -> {
              if (!spark.hasActiveFault()) {
                return false;
              }
              SparkBase.Faults f = spark.getFaults();
              faultAlert.setText(
                  who + ": fault" + (f.sensor ? " SENSOR(check motor sensor cable)" : "")
                      + (f.can ? " CAN" : "")
                      + (f.temperature ? " TEMPERATURE" : "")
                      + (f.motorType ? " MOTOR-TYPE" : "")
                      + (f.gateDriver ? " GATE-DRIVER" : "")
                      + (f.firmware ? " FIRMWARE" : "")
                      + (f.escEeprom ? " EEPROM" : "")
                      + (f.other ? " OTHER" : ""));
              return true;
            },
            faultAlert));

    // Warnings: stall is the dangerous one. A stalled NEO heats up and can kill its SPARK MAX.
    Alert warningAlert = new Alert(GROUP, who + ": warning", AlertType.kWarning);
    checks.add(
        new Check(
            () -> {
              if (!spark.hasActiveWarning()) {
                return false;
              }
              SparkBase.Warnings w = spark.getWarnings();
              warningAlert.setText(
                  who + ": warning" + (w.stall ? " STALL(motor not turning, stop and look)" : "")
                      + (w.brownout ? " BROWNOUT(check battery)" : "")
                      + (w.overcurrent ? " OVERCURRENT" : "")
                      + (w.sensor ? " SENSOR" : "")
                      + (w.hasReset ? " RESET" : ""));
              return true;
            },
            warningAlert));
  }

  private void add(BooleanSupplier isBad, String text, AlertType type) {
    checks.add(new Check(isBad, new Alert(GROUP, text, type)));
  }

  /** Re-runs every check. Called from the subsystem's periodic(). */
  void update() {
    boolean allOk = true;
    for (Check check : checks) {
      boolean bad = check.isBad().getAsBoolean();
      check.alert().set(bad);
      if (bad && check.alert().getType() == AlertType.kError) {
        allOk = false;
      }
    }
    SmartDashboard.putBoolean("Health/All OK", allOk);
  }
}
