package frc.robot.subsystems;

import org.ejml.data.DEigenpair;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;


import edu.wpi.first.wpilibj.I2C;

public class BowWheels extends SubsystemBase {

    private final SparkMax motor1;
    private final SparkMax motor2;

    private final DigitalInput ir_sensor = new DigitalInput(0);
    private static final double kWheelSpeed = 0.5;

    public BowWheels(int motor1Port, int motor2Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(motor2Port, MotorType.kBrushless);

        setDefaultCommand(new RunCommand(() -> {
            motor1.set(0.0);
            motor2.set(0.0);
        }, this));
    }

    public void resetSwitch() {
        ir_sensor.get();
    }

    public Command intake() {
        return run(() -> {
            motor1.set(kWheelSpeed);
            motor2.set(-kWheelSpeed);
        });
    }

    public Command outake() {
        return run(() -> {
            motor1.set(-kWheelSpeed);
            motor2.set(kWheelSpeed);
        });
    }
}