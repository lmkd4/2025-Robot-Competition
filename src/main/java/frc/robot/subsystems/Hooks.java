package frc.robot.subsystems;

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

public class Hooks extends SubsystemBase {

    private final SparkMax motor1;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    // constants
    private static final double kHookSpeed = 0.05;
    private static final double outLim = 0;
    private static final double inLim = 0;

    public Hooks(int motor1Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        setDefaultCommand(new RunCommand(() -> motor1.set(0.0), this));
        
        encoder = motor1.getEncoder();
        pidController = motor1.getClosedLoopController();
    }

    public Command hooksIn() {
        return new RunCommand(() -> {
            motor1.set(kHookSpeed);
        }, this);
    }

    public Command hooksOut() {
        return new RunCommand(() -> {
            motor1.set(-kHookSpeed);
        }, this);
    }

    public Command stop() {
        return new RunCommand(() -> {
            motor1.set(0);
        }, this);
    }
}