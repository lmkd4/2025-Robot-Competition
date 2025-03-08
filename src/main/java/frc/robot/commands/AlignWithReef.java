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

    }

    @Override
    public void initialize() {
      
      // No need to move the elevator here; logic is handled in execute()
    }
  
    // Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        drive.()
  
  
    }
  
    @Override
    public void end(boolean interrupted) {
      m_elevator.stop1();
    }
  
    @Override
    public boolean isFinished() {
      return Math.abs(m_elevator.getElevatorHeight() - targetDistance) < 10;
    }
  }
