package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BowWheels;

public class AutoWheels extends Command {
  
  private final BowWheels m_wheels;
  private Timer time;

  public AutoWheels(BowWheels wheels) {
    m_wheels = wheels;
  }

  @Override
  public void initialize() {
    time.reset();
  }

  @Override
  public void execute() { 
    System.out.println("Wheels are running!");
    // Updates elevator control based on feedback   
    m_wheels.autoOuttake();
  }

  @Override
  public void end(boolean interrupted) {
    m_wheels.stop();
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}
