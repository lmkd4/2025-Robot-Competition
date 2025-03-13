package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

import frc.robot.Robot;

public class ElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Elevator m_elevator;
  private final int targetDistance;
  
  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;
 
  private PIDController elevatorController = new PIDController(kP, kI, kD);

  public ElevatorCommand(Elevator subsystem1, int height) {
    m_elevator = subsystem1;
    this.targetDistance = height;
    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    // No need to move the elevator here; logic is handled in execute()
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double currentDistance = m_elevator.getElevatorHeight();

    //elevator 

    if (currentDistance == Double.NaN) {
      System.out.println("we have a feedback problem");
      return;
    }

    if (currentDistance > targetDistance) {
      m_elevator.moveDown1();
    }

    else if (currentDistance < targetDistance) {
      m_elevator.moveUp1();
    }

    else {
      m_elevator.stop1();
    }

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
