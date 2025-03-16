package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;

public class ElevatorCommand extends Command {
  
  private final Elevator m_elevator;

  private final double targetDistance;

  public ElevatorCommand(Elevator elevator, double height) {
    m_elevator = elevator;
    this.targetDistance = height;
  }

  @Override
  public void initialize() {
    m_elevator.setElevatorSetpoint(targetDistance);
  }

  @Override
  public void execute() { 
    // Updates elevator control based on feedback   
    m_elevator.controlElevator();  
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  @Override
  public boolean isFinished() {
      double elevatorError = Math.abs(targetDistance - m_elevator.getElevatorHeight());
      return (elevatorError <= 10);
  }
}
