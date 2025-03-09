
package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class Vision extends SubsystemBase {

    private final NetworkTable limelightTable;
    public DriveSubsystem m_drive;

    public Vision() {
        // Get the Limelight NetworkTable instance
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getX() {
        // tx - Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getY() {
        // ty - Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    public double getArea() {
        // ta - Target Area (0% of image to 100% of image)
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public boolean hasValidTarget() {
        // tv - Whether the limelight has any valid targets (0 or 1)
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public void setLEDMode(int mode) {
        // Set LED mode: 0 = pipeline default, 1 = off, 2 = blink, 3 = on
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    public Boolean conditional() {
      return false;
    }

    public Command displayValues() {
      return run(() -> {
        SmartDashboard.putNumber("tx", getX());
        SmartDashboard.putNumber("ty", getY());
        SmartDashboard.putNumber("ta", getArea());
      });
    }

    public void setCameraMode(int mode) {
        // Set camera mode: 0 = Vision processor, 1 = Driver Camera (no processing)
        limelightTable.getEntry("camMode").setNumber(mode);
    }
}