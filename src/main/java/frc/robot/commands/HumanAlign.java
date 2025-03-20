package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class HumanAlign extends Command {

    private final DriveSubsystem m_drive;
    public Vision m_lime;
    
    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public double x;
    public double y;

    private String leftOrRight;

    private PIDController xTranslationController = new PIDController(kP, kI, kD);
    private PIDController yTranslationController = new PIDController(kP, kI, kD);


    public HumanAlign(DriveSubsystem subsystem1, Vision subsystem2) {
        m_drive = subsystem1;
        m_lime = subsystem2;
    }

    @Override
    public void initialize() {
      
    }
  
    @Override
    public void execute() {

      Pose3d pose = m_lime.getBotPose3d();

      x = -pose.getX();
      y = -m_lime.getArea();

      double xTranslation = xTranslationController.calculate(x, .1);
      double yTranslation = yTranslationController.calculate(y, -4);

      if (m_lime.hasValidTarget()) {
        m_drive.drive(xTranslation, yTranslation, 0.0, false);
      }

    }
  
    @Override
    public void end(boolean interrupted) {
      m_drive.doNothing();
    }
  
    @Override
    public boolean isFinished() {
      return (Math.abs(y-(-1.5)) < .05) && (Math.abs(x) < .05);
    }
  }
