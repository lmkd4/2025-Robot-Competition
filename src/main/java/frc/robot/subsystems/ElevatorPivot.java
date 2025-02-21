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

public class ElevatorPivot extends SubsystemBase {

    private final SparkFlex motor1;

    // PID constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // constants
    private final double kPivotSpeed = 0.2;
    private final double blockMax = 0;
    private final double blockMin = 0;

    public ElevatorPivot(int motor1Port) {
        motor1 = new SparkFlex(motor1Port, MotorType.kBrushless);

        setDefaultCommand(new RunCommand(() -> {
            motor1.set(0.0);
        }, this));
    }

    public Command in() {
        return new RunCommand(() -> {
            motor1.set(kPivotSpeed);
        }, this);
    }

    public Command out() {
        return new RunCommand(() -> {
            motor1.set(-kPivotSpeed);
        }, this);
    }
}