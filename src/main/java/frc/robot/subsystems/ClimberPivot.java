package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj.I2C;

public class ClimberPivot extends SubsystemBase {
    
    private final SparkFlex motor1;
    private final SparkFlex motor2;

    private double min = 0.6;

    private final SparkAbsoluteEncoder encoder;

    private static final double kPivotSpeed = 1;

    public ClimberPivot(int motor1Port, int motor2Port) {
        motor1 = new SparkFlex(motor1Port, MotorType.kBrushless);
        motor2 = new SparkFlex(motor2Port, MotorType.kBrushless);

        encoder = motor2.getAbsoluteEncoder();

        setDefaultCommand(new RunCommand(() -> {
            motor1.set(0.0);
            motor2.set(0.0);
            SmartDashboard.putNumber("pivot encoder", encoder.getPosition());
        }, this));
    }

    public double getClimberAngle() {
        return encoder.getPosition();
    }

    // this is actually out
    public Command pivotIn() {
        return new RunCommand(() -> {
            motor1.set(kPivotSpeed);
            motor2.set(-kPivotSpeed);
        });
    }

    // this is actually in 
    public Command pivotOut() {
        return new RunCommand(() -> {
            motor1.set(-kPivotSpeed);
            motor2.set(kPivotSpeed);
            /*
            if (getClimberAngle() >= min) {
                motor1.set(0);
                motor2.set(0);
            }*/
        });
    }

    public Command slowPivotIn() {
        return new RunCommand(() -> {
            motor1.set(-0.2);
            motor2.set(0.2);
        });
    }

    public Command stop() {
        return run(() -> {
            motor1.set(0);
            motor2.set(0);
        });
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Pivot Encoder: ", getClimberAngle());
    }
}