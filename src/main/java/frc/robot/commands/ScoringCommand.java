package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;

public class ScoringCommand extends Command {
  
  private final Elevator m_elevator;
  private final ElevatorPivot m_elevatorPivot;

  private final double targetDistance;
  private final double pivotSetpoint;

  public ScoringCommand(Elevator elevator, ElevatorPivot pivot, double height, double angle) {
    m_elevator = elevator;
    m_elevatorPivot = pivot;
    
    this.targetDistance = height;
    this.pivotSetpoint = angle;
  }

  @Override
  public void initialize() {
    // Reset the PID controllers to remove any previous state
    // Explicitly re-set the setpoints
    m_elevator.setElevatorSetpoint(targetDistance);
    m_elevatorPivot.setPivotSetpoint(pivotSetpoint);
  }

  @Override
  public void execute() {    
    m_elevator.controlElevator();  // Updates elevator control based on feedback
    m_elevatorPivot.controlPivot(); // Adjusts pivot position dynamically
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
    m_elevatorPivot.stop();
  }

  @Override
  public boolean isFinished() {
      double elevatorError = Math.abs(targetDistance - m_elevator.getElevatorHeight());
      double pivotError = Math.abs(Math.abs(m_elevatorPivot.getPivotAngle()) - pivotSetpoint);
      // Only finish if we're actually at the correct position
      return (elevatorError <= 10 && pivotError <= 0.005);
  }
}
