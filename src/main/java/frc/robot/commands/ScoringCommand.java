package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    m_elevator.setElevatorSetpoint(targetDistance);
    m_elevatorPivot.setPivotSetpoint(pivotSetpoint);
  }

  @Override
  public void execute() {    

    SmartDashboard.putNumber("Elevator height: ", m_elevator.getElevatorHeight());

    double elevatorError = Math.abs(m_elevator.getElevatorHeight() - targetDistance);
    double pivotError = Math.abs(m_elevatorPivot.getPivotAngle()) - Math.abs(pivotSetpoint);

    System.out.println("Elevator Error: " + elevatorError + ", Pivot Error: " + pivotError);

    m_elevator.controlElevator(); 
    m_elevatorPivot.voidControlPivot();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Scoring Command has finished!");
    m_elevator.stop();
    m_elevatorPivot.stop();
  }

  @Override
  public boolean isFinished() {

    boolean elevatorError = Math.abs(targetDistance - m_elevator.getElevatorHeight()) <= 10;
    boolean posPivotError = Math.abs(m_elevatorPivot.getPivotAngle()) - Math.abs(pivotSetpoint) <= 0.25;
    boolean negPivotError = Math.abs(m_elevatorPivot.getPivotAngle()) - Math.abs(pivotSetpoint) >= -0.25;

    return (elevatorError && posPivotError && negPivotError);
  }
}
