package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Vision;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithReef extends Command {

    private final DriveSubsystem m_drive;
    private final Vision m_lime;

    public AlignWithReef(DriveSubsystem subsystem1, Vision subsystem2) {
        m_drive = subsystem1;
        m_lime = subsystem2;

        addRequirements(subsystem1, subsystem2);
    }

    @Override
    public void initialize() {
      
    }
  
    @Override
    public void execute() {
      m_drive.drive(m_lime.getTx()*0.1, 0, 0, false);
    }
  
    @Override
    public void end(boolean interrupted) {
      m_drive.drive(0, 0, 0, false);
    }
  
    @Override
    public boolean isFinished() {
      return Math.abs(m_lime.getTx()) < .5;
    }
  }
