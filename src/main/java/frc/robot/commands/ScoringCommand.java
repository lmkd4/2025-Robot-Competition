package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.ElevatorPivot;

public class ScoringCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final Elevator m_elevator;
  private final ElevatorPivot m_elevatorPivot;

  public DistanceSensor m_distanceSensor;

  public double targetDistance;
  
  public ScoringCommand(Elevator subsystem, DistanceSensor subsystem1, ElevatorPivot subsystem2, double targetDistance) {
    m_elevator = subsystem;
    m_distanceSensor = subsystem1;
    m_elevatorPivot = subsystem2;
    
    addRequirements(subsystem);
    addRequirements(subsystem1);
    addRequirements(subsystem2);

    this.targetDistance = targetDistance;
  }

  // called when the command is initially scheduled
  @Override
  public void initialize() {
    m_distanceSensor.getRealRange();

    // move elevator
    if (m_distanceSensor.getRealRange() >= targetDistance) {
      m_elevator.moveDown();
    }
    else if (m_distanceSensor.getRealRange() <= targetDistance) {
      m_elevator.moveUp();
    }

    // integrate pivot movement here
  }

  // called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    if (m_distanceSensor.getRealRange() == targetDistance) {
      m_elevator.stop();
    }
    // integrate pivot movement here
  }

  // called when command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // returns true when the command should end
  @Override
  public boolean isFinished() {
    return m_distanceSensor.getRealRange() == targetDistance;
  }
}
