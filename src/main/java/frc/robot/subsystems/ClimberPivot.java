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
    
    // Motors
    private final SparkFlex motor1;
    private final SparkFlex motor2;

    // Encoder and PID controller
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    
    // Constants (modify these based on your elevator design)
    private static final double kPivotSpeed = 0.3;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ClimberPivot(int motor1Port, int motor2Port) {
        motor1 = new SparkFlex(motor1Port, MotorType.kBrushless);
        motor2 = new SparkFlex(motor2Port, MotorType.kBrushless);

        setDefaultCommand(new RunCommand(() -> {
            motor1.set(0.0);
            motor2.set(0.0);
        }, this));

        // Encoder and PID controller from motor1
        encoder = motor1.getEncoder();
        pidController = motor1.getClosedLoopController();

        SimpleMotorFeedforward m_pivotFeedforward = new SimpleMotorFeedforward(motor1Port, motor2Port);
        PIDController m_pivotFeedback = new PIDController(kP, kD, kI);
    }

    public Command pivotOut() {
        return new RunCommand(() -> {
            motor1.set(kPivotSpeed);
            motor2.set(-kPivotSpeed);
        }, this);
    }

    public Command pivotIn() {
        return new RunCommand(() -> {
            motor1.set(-kPivotSpeed);
            motor2.set(kPivotSpeed);
        }, this);
    }
}