package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private PIDController xTranslationController = new PIDController(kP, kI, kD);
    private PIDController yTranslationController = new PIDController(kP, kI, kD);

    private final NetworkTable limelightTable;
    private final DriveSubsystem m_drive;

    private String limelightName = ""; // Set to actual Limelight name
    
    public Vision(DriveSubsystem drive) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable("");
        this.m_drive = drive;
    }

    public double getTx() {
        return LimelightHelpers.getTX("");
    }

    public double getTy() {
        return LimelightHelpers.getTY("");
    }

    public Pose3d getBotPose3d() {
        return LimelightHelpers.getTargetPose3d_RobotSpace("");
    }

    public double getArea() {
        return LimelightHelpers.getTA("");
    }

    public boolean hasValidTarget() {
        return LimelightHelpers.getTV("");
    }
    
    public Command alignToTarget() {
        return run(() -> {
        Pose3d pose3d = getBotPose3d();

        double x = pose3d.getX();
        double y = pose3d.getZ();

        double xTranslation = xTranslationController.calculate(x, 0.05);
        double yTranslation = yTranslationController.calculate(y, -0.3);

        if (hasValidTarget()) {
            m_drive.drive(xTranslation, yTranslation, 0.0, false);
        }
     });
    }

    public Command displayValues() {
        return run(() -> {
            Pose3d pose = getBotPose3d();
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
        });
    }

    @Override
    public void periodic() {


    }
}