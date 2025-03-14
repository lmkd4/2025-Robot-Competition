package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Vision;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithReef extends Command {

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


    public AlignWithReef(DriveSubsystem subsystem1, Vision subsystem2, String orientation) {
        m_drive = subsystem1;
        m_lime = subsystem2;

        this.leftOrRight = orientation;

        addRequirements(subsystem1, subsystem2);
    }

    @Override
    public void initialize() {
      
    }
  
    @Override
    public void execute() {

        switch (leftOrRight) {

        case "L":
          double xTranslation = xTranslationController.calculate(x, 1);
          double yTranslation = yTranslationController.calculate(y, -22);

          break;

        case "R":

          break;
      }

      Pose3d pose = m_lime.getBotPose3d();
            double t[] = { pose.getX(), pose.getY(), pose.getZ() };

            double rX[] = { pose.getRotation().toMatrix().get(0, 0),
                            pose.getRotation().toMatrix().get(1, 0),
                            pose.getRotation().toMatrix().get(2, 0)};

            double rY[] = { pose.getRotation().toMatrix().get(0, 1),
                            pose.getRotation().toMatrix().get(1, 1),
                            pose.getRotation().toMatrix().get(2, 1)};

            double rZ[] = { pose.getRotation().toMatrix().get(0, 2),
                            pose.getRotation().toMatrix().get(1, 2),
                            pose.getRotation().toMatrix().get(2, 2)};
            
            SmartDashboard.putNumberArray("translation", t);
            SmartDashboard.putNumberArray("rotation about x", rX);
            SmartDashboard.putNumberArray("rotation about y", rY);
            SmartDashboard.putNumberArray("rotation about z", rZ);

            SmartDashboard.putNumber("translate 1", t[0]);
            SmartDashboard.putNumber("translate 2", t[1]);
            SmartDashboard.putNumber("translate 3", t[2]);

            SmartDashboard.putNumber("rotation x 1", rX[0]);
            SmartDashboard.putNumber("rotation x 2", rX[1]);
            SmartDashboard.putNumber("rotation x 3", rX[2]);

            SmartDashboard.putNumber("rotation y 1", rY[0]);
            SmartDashboard.putNumber("rotation y 2", rY[1]);
            SmartDashboard.putNumber("rotation y 3", rY[2]);

            SmartDashboard.putNumber("rotation z 1", rZ[0]);
            SmartDashboard.putNumber("rotation z 2", rZ[1]);
            SmartDashboard.putNumber("rotation z 3", rZ[2]);

            SmartDashboard.putNumber("tx", m_lime.getTx());
            SmartDashboard.putNumber("ty", m_lime.getTy());
            SmartDashboard.putNumber("ta", m_lime.getArea());

        Pose3d pose3d = m_lime.getBotPose3d();


        x = -pose3d.getX();
        y = -m_lime.getArea();

        // right side
        double xTranslation = xTranslationController.calculate(x, .1);
        double yTranslation = yTranslationController.calculate(y, -4);

        // left side


        m_drive.drive(xTranslation, yTranslation, 0.0, false);
    }
  
    @Override
    public void end(boolean interrupted) {
      m_drive.drive(0, 0, 0, false);
    }
  
    @Override
    public boolean isFinished() {
      return (Math.abs(y-(-1.5)) < .05) && (Math.abs(x) < .05);
    }
  }
