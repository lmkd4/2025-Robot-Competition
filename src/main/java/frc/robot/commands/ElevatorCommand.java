package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.DistanceSensor;

public class ElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final Elevator m_elevator;

  public DistanceSensor m_distanceSensor;

  private final double targetDistance = 150;

  private final double kShelfDist = 0;
  private final double kLowDist = 0;
  private final double kMidDist = 0;
  private final double kHighDist = 0;

  public ElevatorCommand(Elevator subsystem, DistanceSensor subsystem1) {
    m_elevator = subsystem;
    m_distanceSensor = subsystem1;
    addRequirements(subsystem);
    addRequirements(subsystem1);
  }

  // called when the command is initially scheduled
  @Override
  public void initialize() {
    m_elevator.moveUp();
    m_distanceSensor.getRealRange();
  }

  // called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    if (m_distanceSensor.getRealRange() >= targetDistance) {
      m_elevator.stop();
    }
  }

  // lowest reef level setpoint
  public void shelfLevel() {
    if (m_distanceSensor.getRealRange() >= targetDistance) {
      m_elevator.stop();
    }
  }

  // low reef level setpoint
  public void lowLevel() {
    m_distanceSensor.getRealRange();
  }

  // called when command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // returns true when the command should end
  @Override
  public boolean isFinished() {
    return m_distanceSensor.getRealRange() >= targetDistance;
  }
}
