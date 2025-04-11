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
  private static final String kDefaultAuto = "Leave";
  private static final String kAuto1 = "Do Nothing";
  private static final String kAuto2 = "Leave and Score (CENTER)";
  private static final String kAuto3 = "Leave and Score (LEFT)";
  private static final String kAuto4 = "Leave and Score (RIGHT)";
  private static final String kAuto5 = "Two Coral (LEFT)";
  private static final String kAuto6 = "Two Coral (RIGHT)";
  // autonomous stuff
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser <>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_autoSelected = m_chooser.getSelected();

    SmartDashboard.putData("Auto Choices", m_chooser);

    m_chooser.setDefaultOption("Leave", kDefaultAuto);
    m_chooser.addOption("Do Nothing", kAuto1);
    m_chooser.addOption("Leave and Score (CENTER)", kAuto2);
    m_chooser.addOption("Leave and Score (LEFT)", kAuto3);
    m_chooser.addOption("Leave and Score (RIGHT)", kAuto4);
    m_chooser.addOption("Two Coral (LEFT)", kAuto5);
    m_chooser.addOption("Two Coral (RIGHT)", kAuto6);
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

    m_robotContainer.homeSetpoints();
    m_autoSelected = m_chooser.getSelected();

    System.out.println("Auto Selected:" + m_autoSelected);

    switch(m_autoSelected) {
      case kDefaultAuto:
        m_autonomousCommand = m_robotContainer.leaveCommunityCommand();
        break;

      case kAuto1:
        m_autonomousCommand = m_robotContainer.doNothing();
        break;

      case kAuto2:
        m_autonomousCommand = m_robotContainer.leaveAndScoreCommandCenter();
        break;

      case kAuto3:
        m_autonomousCommand = m_robotContainer.leaveAndScoreCommandLeft();
        break;

      case kAuto4:
        m_autonomousCommand = m_robotContainer.leaveAndScoreCommandRight();
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