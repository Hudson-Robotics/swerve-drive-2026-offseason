// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Robot-wide numbers. Hardware settings (CAN IDs, inversions, offsets, gear ratios, PID gains) are
 * NOT here: they live in the YAGSL JSON files under src/main/deploy/swerve/zombie-ricky/.
 */
public final class Constants {
  private Constants() {}

  public static final class DriveConstants {
    /** Folder under src/main/deploy/ holding this robot's YAGSL config. */
    public static final String SWERVE_CONFIG_DIR = "swerve/zombie-ricky";

    /**
     * Speed that full stick maps to. SDS MK4i L3 + NEO free speed is about 4.93 m/s (16.2 ft/s),
     * so 14.5 ft/s leaves some headroom for friction and battery sag.
     */
    public static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14.5);

    /**
     * Fraction of max speed the driver gets when the robot boots. Change it live on the dashboard
     * (Drive/Speed Cap) as bring-up goes well; it resets to this value on every reboot.
     */
    public static final double DEFAULT_SPEED_CAP = 0.25;

    /** Extra multiplier on top of the speed cap while the driver holds slow mode. */
    public static final double SLOW_MODE_SCALE = 0.4;

    /**
     * Seconds the motors stay in brake mode after disabling, so the robot stops, before switching
     * to coast so people can push it.
     */
    public static final double BRAKE_HOLD_AFTER_DISABLE_SECONDS = 10;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    /** Stick values smaller than this are treated as zero, so a resting stick doesn't creep. */
    public static final double DEADBAND = 0.1;
  }

  /** Numbers for the Test-mode bench commands. Run those with the robot on blocks. */
  public static final class BenchTestConstants {
    public static final double SLOW_DRIVE_METERS_PER_SECOND = 0.5;
    public static final double SLOW_SPIN_RADIANS_PER_SECOND = 1.0;

    /** How long each step of the steer and drive step tests holds its setpoint. */
    public static final double STEP_SECONDS = 2.0;

    public static final double[] STEER_STEP_ANGLES_DEGREES = {0, 90, 180, 270};
    public static final double[] DRIVE_STEP_SPEEDS_METERS_PER_SECOND = {0, 1.0, 2.0, 0};
  }
}
