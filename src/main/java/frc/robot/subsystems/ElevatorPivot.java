package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorPivot extends SubsystemBase {

    private final SparkFlex motor1;
    private final SparkAbsoluteEncoder encoder;

    private static final double kP = 0.25;  
    private static final double kI = 0; 
    private static final double kD = 0;

    public static double pivotSetpoint = 0;

    private static final double kAngularVelocity = 0.3;

    public final PIDController feedback = new PIDController(kP, kI, kD);
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.04, 0);
    private final SlewRateLimiter pivotRateLimiter = new SlewRateLimiter(1);

    public ElevatorPivot(int motor1id) {
        motor1 = new SparkFlex(motor1id, MotorType.kBrushless);
        encoder = motor1.getAbsoluteEncoder();

        feedback.setTolerance(0.05);
    }

    public void setPivotSetpoint(double setpoint) {
        pivotSetpoint = pivotRateLimiter.calculate(setpoint);
    }

    public boolean withinThreshold() {
        return (Math.abs(getPivotAngle()) - Math.abs(pivotSetpoint)) <= 0.05 
        &&
        (Math.abs(getPivotAngle()) - Math.abs(pivotSetpoint) >= -0.05);
    }
    
    // **Set home position based on current encoder value**
    public void homeSetpoints() {

        pivotSetpoint = getPivotAngle();
        pivotSetpoint = clamp(2.43, -2.25, pivotSetpoint);
        feedback.reset();
    }

    public Command adjustSetpointUp() {
        return runOnce(() -> {
            if (pivotSetpoint < 2.43) {
                pivotSetpoint = getPivotAngle() + 0.25;
            }
            // add clamp
            pivotSetpoint = Math.min(pivotSetpoint, 2.43);
        });
    }


    /*
    public Command findSafety() {
        return runOnce(() -> {
            if (getPivotAngle() > 3.14) {
                pivotSetpoint = getPivotAngle() - 0.25;
            }

            else if (getPivotAngle() < -1.57) {
                pivotSetpoint = getPivotAngle() + 0.25;
            }
        });
    }
    */

    public Command adjustSetpointDown() {
        return runOnce(() -> {
            if (pivotSetpoint > -2.25){
                pivotSetpoint = getPivotAngle() - 0.25;
            }

            pivotSetpoint = Math.max(pivotSetpoint, -2.25);
        });
    }

    public double getPivotAngle() {
        return -(((2*3.14159)*encoder.getPosition()) - 2.77);
    }

    // try putting this code into a periodic method?
    public Command controlPivot() {
        return run(() -> { 
            
            /* Clamp TBT 
            if (getPivotAngle() < -1.57 && getPivotAngle() > 3.14) {
                findSafety();
            }
            */

            double pidOutput = feedback.calculate(getPivotAngle(), pivotSetpoint);
            double ffOutput = feedforward.calculate(getPivotAngle() - (0.209), 0);
            double finalOutput = ffOutput + pidOutput;

            SmartDashboard.putNumber("Pivot Encoder Position", getPivotAngle());
            SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
            SmartDashboard.putNumber("Pivot Error", feedback.getPositionError());

            motor1.set(-finalOutput);
        });
    }

    public void voidControlPivot() {
        double pidOutput = feedback.calculate(getPivotAngle(), pivotSetpoint);
        double ffOutput = feedforward.calculate(getPivotAngle() - (0.209), 0);
        double finalOutput = ffOutput + pidOutput;

        SmartDashboard.putNumber("Pivot Encoder Position", getPivotAngle());
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Pivot Error", feedback.getPositionError());

        motor1.set(-finalOutput);
    }

    public Command stop() {
        return run(() -> motor1.set(0));
    }

    public Command pivotOutCommand() {
        return new RunCommand(() -> motor1.set(0.2), this);
    }

    public Command pivotInCommand() {
        return new RunCommand(() -> motor1.set(-0.2), this);
    }
    

    public double clamp(double max, double min, double setpoint) {
        if (setpoint < min) {
            return min;
        }

        else if (setpoint > max) {
            return max;
        }

        return setpoint;
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle", getPivotAngle());
        double pidOutput = feedback.calculate(getPivotAngle(), pivotSetpoint);
        double ffOutput = feedforward.calculate(getPivotAngle() - (0.209), 0);
        double finalOutput = ffOutput + pidOutput;

        SmartDashboard.putNumber("Pivot Encoder Position", getPivotAngle());
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Pivot Error", feedback.getPositionError());

        motor1.set(-finalOutput);

    }
}
