// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // Auto chooser
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  // Auto option constants
  private static final String kAutoLeft   = "Left Side Auto";
  private static final String kAutoMiddle = "Middle Auto";
  private static final String kAutoRight  = "Right Side Auto";

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // Configure the auto chooser options
    m_autoChooser.setDefaultOption("Right Side Auto", kAutoRight);
    m_autoChooser.addOption("Left Side Auto", kAutoLeft);
    m_autoChooser.addOption("Middle Auto", kAutoMiddle);

    // Put the chooser on SmartDashboard
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    initializeGyro();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // Read the selected auto from SmartDashboard
    String autoSelected = m_autoChooser.getSelected();
    SmartDashboard.putString("Auto Running", autoSelected);

    switch (autoSelected) {
      case kAutoLeft:
        m_autonomousCommand = m_robotContainer.getLeftSideAuto();
        break;
      case kAutoRight:
        m_autonomousCommand = m_robotContainer.getRightSideAuto();
        break;
      case kAutoMiddle:
      default:
        m_autonomousCommand = m_robotContainer.getMiddleAuto();
        break;
    }

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  private void initializeGyro() {
    DriveSubsystem drive = m_robotContainer.getDriveSubsystem();

    new Thread(() -> {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      drive.resetGyroToZero();
    }).start();
  }
}