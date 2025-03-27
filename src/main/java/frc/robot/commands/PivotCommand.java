package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorPivot;

public class PivotCommand extends Command {
  
  private final ElevatorPivot m_elevatorPivot;

  private final double targetAngle;

  public PivotCommand(ElevatorPivot pivot, double height) {
    m_elevatorPivot = pivot;
    this.targetAngle = height;
  }

  @Override
  public void initialize() {
    m_elevatorPivot.setPivotSetpoint(targetAngle);

  }

  @Override
  public void execute() { 
    System.out.println("Pivot command is running!");
    m_elevatorPivot.controlPivot();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Pivot command finished!");
    m_elevatorPivot.stop();
  }

  @Override
  public boolean isFinished() {
      double elevatorError = Math.abs(targetAngle - m_elevatorPivot.getPivotAngle());
      return (elevatorError <= 10);
  }
}