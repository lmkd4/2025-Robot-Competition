package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.commands.ScoringCommand;

public class PivotCommand extends Command {
  
  private final ElevatorPivot m_elevatorPivot;
  private final double targetAngle;
  private final ScoringCommand m_scoringCommand;
  
    
  public PivotCommand(ElevatorPivot subsystem, ScoringCommand command, double targetAngle) {
    m_elevatorPivot = subsystem;
    m_scoringCommand = command;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    // No need to move the elevator here; logic is handled in execute()
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double currentAngle = m_elevatorPivot.getPivotAngle();
    
    if (currentAngle > targetAngle) {
      m_elevatorPivot.pivotOutCommand();
    } 
    
    else if (currentAngle < targetAngle) {
      m_elevatorPivot.pivotInCommand();
    } 
    
    else {
      m_elevatorPivot.stop();
    }
  }

  // Called when command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_elevatorPivot.stop();
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return Math.abs(m_elevatorPivot.getPivotAngle() - targetAngle) < 0.5; // Adjust tolerance as needed
  }
}