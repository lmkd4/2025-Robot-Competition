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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;


import edu.wpi.first.wpilibj.I2C;

public class Elevator extends SubsystemBase {
    
    // Motors
    private final SparkMax motor1;
    private final SparkMax motor2;

    // Encoder and PID controller
    private final SparkClosedLoopController pidController;

    // Constants (modify these based on your elevator design)
    private static final double kElevatorSpeed = 0.15;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxHeight = 540.0; // Example max position
    private static final double kMinHeight = 75.0;

    private static final double lowLim = 0;
    private static final double highLim = 0;

    public Elevator(int motor1Port, int motor2Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(motor2Port, MotorType.kBrushless);

        // Encoder and PID controller from motor1
        pidController = motor1.getClosedLoopController();

        setDefaultCommand(new RunCommand(() -> {
            motor1.set(0.0);
            motor2.set(0.0);
        }, this));

        /*
        pidcontroller.setkP(kP);
        pidController.setkI(kI);
        pidController.setkD(kD);
        */
    }
    

    // Method to move the elevator to a target height
    public void setPosition(double position) {
        if (position > kMaxHeight) {
            position = kMaxHeight;
        } else if (position < kMinHeight) {
            position = kMinHeight;
        }
        pidController.setReference(position, SparkMax.ControlType.kPosition);
    }

    public void clampElevatorSetpoints() {
        // rewrite with distance sensor
    }

    public void stop() {
        motor1.set(0);
        motor2.set(0);
    }

    public Command moveUp() {
        return new RunCommand(() -> {
            motor1.set(kElevatorSpeed);
            motor2.set(-kElevatorSpeed);
        }, this);
    }

    public Command moveDown() {
        return new RunCommand(() -> {
            motor1.set(-kElevatorSpeed);
            motor2.set(kElevatorSpeed);
        }, this);
    }

    public Command shelfLevel() {
        return new InstantCommand();
    }
}