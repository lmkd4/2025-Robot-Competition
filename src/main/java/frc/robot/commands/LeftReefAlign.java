package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class LeftReefAlign extends Command {

    private final DriveSubsystem m_drive;
    public Vision m_lime;
    
    public double x;
    public double y;


    public LeftReefAlign(DriveSubsystem subsystem1, Vision subsystem2) {
        m_drive = subsystem1;
        m_lime = subsystem2;
    }

    @Override
    public void initialize() {
      
    }
  
    @Override
    public void execute() {
      m_lime.alignToTargetLeft();
    }
  
    @Override
    public void end(boolean interrupted) {
      m_drive.drive(0, 0, 0, false);
    }
  
    @Override
    public boolean isFinished() {
      return m_lime.getBotPose3d().getX() < .005 && m_lime.getBotPose3d().getZ() < .005;
    }
  }
