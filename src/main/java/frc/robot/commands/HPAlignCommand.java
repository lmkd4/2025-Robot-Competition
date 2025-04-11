package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class HPAlignCommand extends Command {

    private final DriveSubsystem m_drive;
    private final Vision m_lime;
    
    public double xTarget;
    public double yTarget;
    public double rotTarget;

    private static final double kP = 0.75;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private PIDController xTranslationController = new PIDController(kP, kI, kD);
    private PIDController yTranslationController = new PIDController(kP, kI, kD);

    public HPAlignCommand(DriveSubsystem subsystem1, Vision subsystem2, double x, double y, double rot) {
        m_drive = subsystem1;
        m_lime = subsystem2;

        this.xTarget = x;
        this.yTarget = y;
        this.rotTarget = rot;
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
      // differentiate between two different limelights?
      m_lime.HPLightsOn();
      System.out.println("Command is running...");

      if (!m_lime.hasValidTargetReef()) {
        m_drive.drive(0, 0, 0, false);
        return;
      }

      Translation3d target_point_tag_frame = new Translation3d(xTarget, yTarget, rotTarget);

      Pose3d pose3d = m_lime.getBotPose3dHP();
      Translation3d target_point_robot_frame = target_point_tag_frame.rotateBy(pose3d.getRotation()).plus(pose3d.getTranslation());

      double xTranslation = xTranslationController.calculate(-target_point_robot_frame.getX(), 0.00);
      double yTranslation = yTranslationController.calculate(-target_point_robot_frame.getZ(), 0.00);
      double rot_cmd = pose3d.getRotation().getY() * 0.80;

      m_drive.drive(xTranslation, yTranslation, rot_cmd, false);
    }
  
    @Override
    public void end(boolean interrupted) {
      System.out.println("Command is finished!");
      m_drive.drive(0, 0, 0, false);
    }
  
    @Override
    public boolean isFinished() {
      return m_lime.aligned();
    }
  }
