// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final Timer m_disabledTimer = new Timer();

  public Robot() {
    // Record every NetworkTables value and the joysticks to a .wpilog file on the roboRIO (or a USB
    // stick if one is plugged in). Open it in AdvantageScope to replay a bad run after the fact.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateDashboard();
  }

  @Override
  public void disabledInit() {
    // Brake so the robot stops, then coast after a few seconds so it can be pushed around.
    m_robotContainer.setMotorBrake(true);
    m_disabledTimer.restart();
  }

  @Override
  public void disabledPeriodic() {
    if (m_disabledTimer.hasElapsed(DriveConstants.BRAKE_HOLD_AFTER_DISABLE_SECONDS)) {
      m_robotContainer.setMotorBrake(false);
      m_disabledTimer.stop();
      m_disabledTimer.reset();
    }
  }

  @Override
  public void disabledExit() {
    m_disabledTimer.stop();
    m_robotContainer.setMotorBrake(true);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
