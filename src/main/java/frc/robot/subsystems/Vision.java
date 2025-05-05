package frc.robot.subsystems;

import java.util.Set;

import org.dyn4j.geometry.Vector3;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private static final double kP = 0.75;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private Blinkin led;

    private PIDController xTranslationController = new PIDController(kP, kI, kD);
    private PIDController yTranslationController = new PIDController(kP, kI, kD);

    private final NetworkTable limelightTable;
    private final DriveSubsystem m_drive;

    
    public Vision(DriveSubsystem drive) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable("");
        this.m_drive = drive;
    }

    public double getTx() {
        return LimelightHelpers.getTX("");
    }

    public double getTxHP() {
        return LimelightHelpers.getTX("limelight-hp");
    }

    public double getTxReef() {
        return LimelightHelpers.getTX("limelight-reef");
    }

    public double getTy() {
        return LimelightHelpers.getTY("");
    }

    public Pose3d getBotPose3dReef() {
        return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-reef");
    }

    public Pose3d getBotPose3dHP() {
        return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-HP");
    }

    public double getArea() {
        return LimelightHelpers.getTA(""); 
    }

    public boolean hasValidTargetReef() {
        return LimelightHelpers.getTV("limelight-reef");
    }

    public void reefLightsOn() {
        LimelightHelpers.setLEDMode_ForceOn("limelight-reef");
    }

    public void HPLightsOn() {
        LimelightHelpers.setLEDMode_ForceOn("limelight-HP");
    }

    public Command alignToTargetRight() {
        return run(() -> {

            if (!hasValidTargetReef()) {
                m_drive.drive(0, 0, 0, false);
                return;
            }

        System.out.println("Test");
        Pose3d pose3d = getBotPose3dReef();

        Translation3d target_point_tag_frame = new Translation3d(0.36, 0, -0.45);
        
        Translation3d target_point_robot_frame = target_point_tag_frame.rotateBy(pose3d.getRotation()).plus(pose3d.getTranslation()) ;

        double x = pose3d.getX();
        double y = pose3d.getZ();

        double xTranslation = xTranslationController.calculate(-target_point_robot_frame.getX(), 0.00);
        double yTranslation = yTranslationController.calculate(-target_point_robot_frame.getZ(), 00.0);
        double rot_cmd = pose3d.getRotation().getY() * 0.30;

        m_drive.drive(xTranslation, yTranslation, rot_cmd, false);
      });
    }

    public void htc() {
        Pose3d pose3d = getBotPose3dReef();

        Translation3d targetInTagFrame = new Translation3d(0, 0, 0);

        Translation3d targetInRobotFrame = targetInTagFrame.rotateBy(pose3d.getRotation()).plus(pose3d.getTranslation());

        double xTrans = xTranslationController.calculate(-targetInTagFrame.getX(), 0);
        double yTrans = yTranslationController.calculate(-targetInTagFrame.getZ(), 0);

        // UNSCALED
        double theta = pose3d.getRotation().getY();
        
        // gradient runs until limelight "0" is accomplished
        led.setLED(xTrans);
        m_drive.drive(xTrans, yTrans, theta, false);
    }

    public Command alignToTargetLeft() {
        return run(() -> {

        System.out.println("Test");
        Pose3d pose3d = getBotPose3dReef();

        Translation3d target_point_tag_frame = new Translation3d(0.064, 0, -0.45);
        
        Translation3d target_point_robot_frame = target_point_tag_frame.rotateBy(pose3d.getRotation()).plus(pose3d.getTranslation()) ;

        double x = pose3d.getX();
        double y = pose3d.getZ();

        double xTranslation = xTranslationController.calculate(-target_point_robot_frame.getX(), 0.00);
        double yTranslation = yTranslationController.calculate(-target_point_robot_frame.getZ(), 00.0);
        double rot_cmd = pose3d.getRotation().getY() * 0.80;

        m_drive.drive(xTranslation, yTranslation, rot_cmd, false);
      });
    }

    public boolean aligned() {
        Pose3d pose3d = getBotPose3dReef();
        Translation3d target_point_tag_frame = new Translation3d(0.064, 0, -0.45);
        Translation3d target_point_robot_frame = target_point_tag_frame.rotateBy(pose3d.getRotation()).plus(pose3d.getTranslation());

        return Math.hypot(target_point_robot_frame.getX(), target_point_robot_frame.getZ()) < .025;
    }

    public Command rotateToTarget() {
        return run(() -> {

            System.out.println("Test");
            Pose3d pose3d = getBotPose3dReef();
    
            Translation3d target_point_tag_frame = new Translation3d(0, 0, -1.0);
            
            Translation3d target_point_robot_frame = target_point_tag_frame.rotateBy(pose3d.getRotation()).plus(pose3d.getTranslation()) ;

            double rot_cmd = pose3d.getRotation().getY() * 0.30;
    
            if (hasValidTargetReef()) {
                m_drive.drive(0, 0, rot_cmd, false);
            }
          });
    }


    public Command displayValues() {
        return run(() -> {
            Pose3d pose = getBotPose3dReef();

            Translation3d target_point_tag_frame = new Translation3d(0.0, 0.0, -1.0);
        
            Translation3d target_point_robot_frame = target_point_tag_frame.rotateBy(pose.getRotation()).plus(pose.getTranslation()) ;
            
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
    
            SmartDashboard.putNumber("tx", getTx());
            SmartDashboard.putNumber("ty", getTy());
            SmartDashboard.putNumber("ta", getArea());

            SmartDashboard.putNumber("tpx", target_point_robot_frame.getX());
            SmartDashboard.putNumber("tpy", target_point_robot_frame.getY());
            SmartDashboard.putNumber("tpz", target_point_robot_frame.getZ());
            SmartDashboard.putNumber("tpyaw", pose.getRotation().getY());
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("human player tx: ", getTxHP());
        SmartDashboard.putNumber("reef tx: ", getTxReef());
        
        LimelightHelpers.setLEDMode_ForceOff("limelight-reef");
        LimelightHelpers.setLEDMode_ForceOff("limelight-HP");
        
    }
}