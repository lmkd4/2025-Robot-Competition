
package frc.robot.limelight;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final NetworkTable m_limelightTable;
  private double tv, tx, ta, ty;
  private ArrayList<Double> m_targetList;
  private final int MAX_ENTRIES = 50;

  // limelight constructor
  public Vision() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_targetList = new ArrayList<Double>(MAX_ENTRIES);
  }

  @Override
  public void periodic() {
    // periodically run contents
    tv = m_limelightTable.getEntry("tv").getDouble(0);
    tx = m_limelightTable.getEntry("tx").getDouble(0);
    ta = m_limelightTable.getEntry("ta").getDouble(0);
    ty = m_limelightTable.getEntry("ty").getDouble(0);

    if (m_targetList.size() >= MAX_ENTRIES) {
      m_targetList.remove(0);
    }
    m_targetList.add(ta);
    }

    public double getTy() {
        return ty;
    }

    public double getTx() {
        return tx;
    }

    public double getTa() {
        double sum = 0;

        for (Double num : m_targetList) { 		      
        sum += num.doubleValue();
        }
        return sum/m_targetList.size();
    }

    public boolean isTargetValid() {
        return (tv == 1.0); 
    }
}