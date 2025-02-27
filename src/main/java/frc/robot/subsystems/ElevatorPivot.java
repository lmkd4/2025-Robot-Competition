package frc.robot.subsystems;

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
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexExternalEncoderSim;

import edu.wpi.first.wpilibj.I2C;

public class ElevatorPivot extends SubsystemBase {

    private final SparkFlex motor1;
    public final SparkFlexExternalEncoderSim m_encoder;

    // PID constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // constants
    private final double kPivotSpeed = 0.2;

    private final PIDController m_feedback = new PIDController(kP, kI, kD);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kI, kD);

    public ElevatorPivot(int motor1Port) {
        motor1 = new SparkFlex(motor1Port, MotorType.kBrushless);
        m_encoder = new SparkFlexExternalEncoderSim(motor1);

        // what is error tolerance?
        m_feedback.setTolerance(0.1);

        setDefaultCommand(new RunCommand(() -> {
            motor1.set(0.0);
        }, this));
    }

    public Command stop() {
        return run( () -> {
            motor1.set(0);
        });
    }

    public Command pivotCommand(double setAngularVelocity) {
        return run(
                    () -> {
                      motor1.set(
                          m_feedforward.calculate(setAngularVelocity)
                              + m_feedback.calculate(
                                  m_encoder.getPosition(), setAngularVelocity));
                    });    
      }
    }