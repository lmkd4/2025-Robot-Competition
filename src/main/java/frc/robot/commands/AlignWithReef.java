package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Vision;

public class AlignWithReef extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final Vision m_limelight;

    public AlignWithReef(DriveSubsystem subsystem1, Vision subsystem2) {
        m_driveSubsystem = subsystem1;
        m_limelight = subsystem2;

        addRequirements(subsystem1, subsystem2);
    }

    @Override
    public void initialize() {

    }
  
    @Override
    public void execute() {
      m_driveSubsystem.drive(m_limelight.getX()*0.1, 0, 0, false);
    }
  
    @Override
    public void end(boolean interrupted) {
      m_driveSubsystem.drive(0, 0, 0, false);
    }
  
    @Override
    public boolean isFinished() {
      return m_limelight.getX() < .5;
    }
  }
