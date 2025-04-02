// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // autonomous option strings
  private static final String kDefaultAuto = "default";
  private static final String kAuto1 = "leave community";
  private static final String kAuto2 = "do nothing";
  private static final String kAuto3 = "leave, score, find, score";
  private static final String kAuto4 = "test limelight";
  private static final String kAuto5 = "test elevator and wheels";
  // autonomous stuff
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser <>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_autoSelected = m_chooser.getSelected();

    SmartDashboard.putData("Auto Choices", m_chooser);

    m_chooser.setDefaultOption("do nothing", kDefaultAuto);
    m_chooser.addOption("leave", kAuto1);
    m_chooser.addOption("leave and score", kAuto2);
    m_chooser.addOption("FORWARD TO REEF", kAuto3);
    m_chooser.addOption("test limelight", kAuto4);
    m_chooser.addOption("test elevator and wheels", kAuto5);
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

    m_autoSelected = m_chooser.getSelected();

    System.out.println("Auto Selected:" + m_autoSelected);

    switch(m_autoSelected) {
      case kDefaultAuto:
        m_autonomousCommand = m_robotContainer.leaveCommunityCommand();
        break;

      case kAuto1:
        m_autonomousCommand = m_robotContainer.leaveCommunityCommand();
        break;

      case kAuto2:
        m_autonomousCommand = m_robotContainer.leaveAndScoreCommand();
        break;

      case kAuto3:
        m_autonomousCommand = m_robotContainer.forwardToReef();
        break;

      case kAuto4:
        m_autonomousCommand = m_robotContainer.testLimelightCommand();
        break;
      
      case kAuto5:
        m_autonomousCommand = m_robotContainer.testFunctions();
        break;
    }
        
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.homeSetpoints();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}