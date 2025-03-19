package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BowWheels extends SubsystemBase {

    private final SparkMax motor1;
    private final SparkMax motor2;

    private final DigitalInput ir_sensor = new DigitalInput(0);
  // IR sensor on DIO port 0
    private final double kWheelSpeed = 0.4;

    public BowWheels(int motor1Port, int motor2Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(motor2Port, MotorType.kBrushless);

        // set default command
    }

    // Command to intake (start moving the wheels in)
    public Command intake() {
        return run(() -> {
             if (!isIRSensorTriggered()) {
                motor1.set(kWheelSpeed);
                motor2.set(-kWheelSpeed);
            }
             else {
              motor1.set(kWheelSpeed);
                motor2.set(-kWheelSpeed);
             }
        });
    }

    public void runWheelsInAuto() {
        motor1.set(-kWheelSpeed);
        motor2.set(kWheelSpeed);
    }
    // Command to outtake (move the wheels out)
    public Command outtake() {
        return run(() -> {
            // Always run the outtake regardless of IR sensor status
            motor1.set(-kWheelSpeed);
            motor2.set(kWheelSpeed);
        });
    }

    public Command stop() {
        return run(() -> {
            // Always run the outtake regardless of IR sensor status
            motor1.set(0);
            motor2.set(0);
        });
    }
    
    public Command runUntilTripped() {
        return runEnd(() -> {
            motor1.set(-kWheelSpeed);
            motor2.set(kWheelSpeed);

            }, () -> { 
                
            if (ir_sensor.get() == false) {
                motor1.set(0);
                motor2.set(0);
            }
        });
    }

    // Reset or read the IR sensor state
    public void resetSwitch() {
        ir_sensor.get();
    }

    // Optional: method to check if the IR sensor is triggered
    public boolean isIRSensorTriggered() {
        return ir_sensor.get();
    }

    @Override
    public void periodic() {
        
    }
}
