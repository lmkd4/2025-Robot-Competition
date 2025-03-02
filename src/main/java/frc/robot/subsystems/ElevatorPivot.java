package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkFlexExternalEncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorPivot extends SubsystemBase {

    private final SparkFlex motor1;
    private final SparkFlexExternalEncoderSim m_encoder;

    // PID Constants (Tuned for stability)
    private static final double kP = 0.5;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Pivot Speed & Target Position
    private static final double kAngularVelocity = 0.3;

    private final PIDController m_feedback = new PIDController(kP, kI, kD);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kI, kD);

    public ElevatorPivot(int motor1Port) {
        motor1 = new SparkFlex(motor1Port, MotorType.kBrushless);
        m_encoder = new SparkFlexExternalEncoderSim(motor1);

        m_feedback.setTolerance(0.1); 

        setDefaultCommand(new RunCommand(() -> {
            motor1.set(0.0);
        }, this));
    }

    public Command stop() {
        return runOnce(() -> {
            motor1.set(0);
        });
    }

    public Command pivotOutCommand() {
        return run(() -> {
            motor1.set(
                m_feedforward.calculate(-kAngularVelocity)
                    + m_feedback.calculate(
                        m_encoder.getPosition(), -kAngularVelocity));
          });    
    }
    
    public Command pivotInCommand() {
        return run(() -> {
                motor1.set(
                    m_feedforward.calculate(kAngularVelocity)
                        + m_feedback.calculate(
                            m_encoder.getPosition(), kAngularVelocity));   
        });
    }
}
