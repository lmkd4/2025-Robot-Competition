package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorPivot extends SubsystemBase {

    private final SparkFlex motor1;
    private final SparkAbsoluteEncoder encoder;

    private static final double kP = 0.05;  
    private static final double kI = 0; 
    private static final double kD = 0;  

    private static double pivotSetpoint = 0;

    private static final double kAngularVelocity = 0.3;

    private final PIDController feedback = new PIDController(kP, kI, kD);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.2);
    private final SlewRateLimiter pivotRateLimiter = new SlewRateLimiter(1);

    public ElevatorPivot(int motor1id) {
        motor1 = new SparkFlex(motor1id, MotorType.kBrushless);
        encoder = motor1.getAbsoluteEncoder();

        feedback.setTolerance(0.05);
    }

    public void setPivotSetpoint(double setpoint) {
        pivotSetpoint = pivotRateLimiter.calculate(setpoint);
    }

    // **Set home position based on current encoder value**
    public void homeSetpoints() {
        pivotSetpoint = encoder.getPosition();
        feedback.reset();
    }

    public double getPivotAngle() {
        return encoder.getPosition();
    }

    // **Manually apply PID control**
    public Command controlPivot() {
        return run(() -> {
            double pidOutput = feedback.calculate(getPivotAngle(), pivotSetpoint);
            double ffOutput = feedforward.calculate(pivotSetpoint);
            double finalOutput = pidOutput + ffOutput;

            motor1.set(finalOutput);
        });
    }

    public Command stop() {
        return runOnce(() -> motor1.set(0));
    }

    public Command pivotOutCommand() {
        return run(() -> {
            double output = feedforward.calculate(-kAngularVelocity)
                    + feedback.calculate(encoder.getPosition(), -kAngularVelocity);
            motor1.set(output);
        });
    }

    public Command pivotInCommand() {
        return run(() -> {
            double output = feedforward.calculate(kAngularVelocity)
                    + feedback.calculate(encoder.getPosition(), kAngularVelocity);
            motor1.set(output);
        });
    }

    public Command smartdashboard() {
        return run(() -> {
            SmartDashboard.putNumber("Pivot Encoder Position", encoder.getPosition());
            SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
            SmartDashboard.putNumber("Pivot Error", feedback.getPositionError());
        });
    }
}
