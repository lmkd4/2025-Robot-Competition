package frc.robot.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.limelight.LimelightHelpers;

public class Vision extends SubsystemBase {

    // PID Gains 
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private PIDController xTranslationController = new PIDController(kP, kI, kD);
    private PIDController yTranslationController = new PIDController(kP, kI, kD);

    private final NetworkTable limelightTable;
    private final DriveSubsystem m_drive;
    private String limelightName = "limelight"; // Set to actual Limelight name
    
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

    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    public void setCameraMode(int mode) {
        limelightTable.getEntry("camMode").setNumber(mode);
    }

    
    public void alignToTarget() {

            Pose3d pose3d = getBotPose3d();

            double x = pose3d.getX();
            double y = getArea();

            double xTranslation = xTranslationController.calculate(x, 0);
            double yTranslation = yTranslationController.calculate(y, -1.5);

            m_drive.drive(xTranslation, yTranslation, 0.0, false);
    }

    public Command followTarget() {
        return run(() -> {
            if (hasValidTarget()) {
                m_drive.drive(getTx() * 0.1, 0, 0, false);
            } else {
                m_drive.drive(0, 0, 0, false);
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
}