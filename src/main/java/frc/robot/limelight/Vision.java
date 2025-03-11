package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.limelight.LimelightHelpers;

public class Vision extends SubsystemBase {

    private final NetworkTable limelightTable;
    private final DriveSubsystem m_drive;
    private final String limelightName = "limelight"; // Set to actual Limelight name

    public Vision(DriveSubsystem drive) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        this.m_drive = drive;
    }

    public double getTx() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTy() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getArea() {
        return LimelightHelpers.getTA(limelightName);
    }

    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    public void setCameraMode(int mode) {
        limelightTable.getEntry("camMode").setNumber(mode);
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
            SmartDashboard.putNumber("tx", getTx());
            SmartDashboard.putNumber("ty", getTy());
            SmartDashboard.putNumber("ta", getArea());
        });
    }
}