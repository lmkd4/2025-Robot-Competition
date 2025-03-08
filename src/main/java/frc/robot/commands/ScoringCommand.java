/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;

public class ScoringCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Elevator m_elevator;
  private final ElevatorPivot m_elevatorPivot;
  private int targetConfig;

  ElevatorCommand m_elevatorCommand = new ElevatorCommand(m_elevator, targetConfig);

  public ScoringCommand(Elevator subsystem1, ElevatorPivot subsystem2, int targetDistance) {
    m_elevator = subsystem1;
    m_elevatorPivot = subsystem2;
    
    this.targetConfig = targetDistance;
    addRequirements(subsystem1, subsystem2);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    switch(targetConfig) {

        case 180:
            m_elevatorPivot.findSetpoint(0.4).andThen(m_elevatorCommand);
          break;

        case 190:
            m_elevatorPivot.findSetpoint(0.5).andThen(m_elevatorCommand);
          break;

        
        default:

      }
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return m_elevatorCommand.isFinished();
  }
}*/