package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin extends SubsystemBase {


    Spark blinkin = new Spark(0);

    public Blinkin() {}

    // runs when limelight is searching
    public void limelight() {
        blinkin.set(0);
    }

    // L1
    public void elOne() {
        blinkin.set(0);
    }

    // L2
    public void elTwo() {
        blinkin.set(0);
    }

    // L3
    public void elThree() {
        blinkin.set(0);
    }

    // L4
    public void elFour() {
        blinkin.set(0);
    }

    // human player
    public void hp() {
        blinkin.set(0);
    }
    

    // runs when IR sensor is tripped
    public void recieve() {
        blinkin.set(0);
    }

    // runs when bow wheels intake
    public void wheels() {
        blinkin.set(0);
    }


    @Override
    public void periodic() {
        // default mode
        blinkin.set(0);
    }
}
