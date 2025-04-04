package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BowWheels;

public class AutoWheels extends Command {
  
  private final BowWheels m_wheels;
  private Timer time = new Timer();

  public AutoWheels(BowWheels wheels) {
    m_wheels = wheels;

    addRequirements(m_wheels);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
    m_wheels.autoOuttake();
  }

  @Override
  public void execute() { 
    System.out.println("Wheels are running!");
    // Updates elevator control based on feedback   
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Wheels have STOPPED");
    m_wheels.stop();
  }

  @Override
  public boolean isFinished() {
      return time.hasElapsed(2);
  }
}
