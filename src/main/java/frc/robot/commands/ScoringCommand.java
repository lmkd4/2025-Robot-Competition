package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.ElevatorPivot;

public class ScoringCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final Elevator m_elevator;
  private final DistanceSensor m_distanceSensor;
  private final double targetDistance;
  
  public ScoringCommand(Elevator subsystem1, DistanceSensor subsystem2, double targetDistance) {
    m_elevator = subsystem1;
    m_distanceSensor = subsystem2;
    this.targetDistance = targetDistance;
    
    addRequirements(subsystem1, subsystem2);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    // No need to move the elevator here; logic is handled in execute()
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double currentDistance = m_distanceSensor.getRealRange();

    if (currentDistance > targetDistance + 0.5) {
      m_elevator.moveDown();
    } 
    else if (currentDistance < targetDistance - 0.5) {
      m_elevator.moveUp();
    } 
    else {
      m_elevator.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_distanceSensor.getRealRange() - targetDistance) < 0.5;
  }
}