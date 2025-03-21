package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;

public class RightReefAlign extends Command {

    private final DriveSubsystem m_drive;
    public Vision m_lime;

    public double x;
    public double y;



    public RightReefAlign(DriveSubsystem subsystem1, Vision subsystem2) {
        m_drive = subsystem1;
        m_lime = subsystem2;
    }

    @Override
    public void initialize() {
      
    }
  
    @Override
    public void execute() {
      m_lime.alignToTargetRight();
    }
  
    @Override
    public void end(boolean interrupted) {
      m_drive.drive(0, 0, 0, false);
    }
  
    @Override
    public boolean isFinished() {
      return (Math.abs(y)-(0.48)) < .005 && (Math.abs(x) - 0.045 < .005);
    }
  }
