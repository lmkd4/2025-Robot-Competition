package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BowWheels;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;

public class ScoreInAuto extends Command {
  
 private final BowWheels m_bowWheels;
 private final Timer timer = new Timer();


  public ScoreInAuto(BowWheels wheels) {
    m_bowWheels = wheels;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() { 
    m_bowWheels.runWheelsInAuto();
  }

  @Override
  public void end(boolean interrupted) {
    m_bowWheels.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }
}
