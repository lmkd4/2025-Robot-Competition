package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase {
  // sensor declaration
  private Rev2mDistanceSensor distanceSensor;

  public DistanceSensor() {
    // roboRio port
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    // configure
    distanceSensor.setAutomaticMode(true); // automatically updates distance
    distanceSensor.setDistanceUnits(Unit.kMillimeters); // set to millimeters
    distanceSensor.setRangeProfile(RangeProfile.kDefault); // set range profile
  }

  // method to get the current distance reading in millimeters
  public double getRealRange() {
    return distanceSensor.getRange(); // Use getRange() instead of GetRange()
  }

  // method to check if the sensor is operating normally
  public boolean isSensorValid() {
    return distanceSensor.isRangeValid();
  }

  @Override
  public void periodic() {
    // Called once per scheduler run, you can use it for debugging
  }
}
