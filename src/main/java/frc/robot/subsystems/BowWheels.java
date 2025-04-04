package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BowWheels extends SubsystemBase {

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final double kWheelSpeed = 0.4;

    private double speed = 0;

    public BowWheels(int motor1Port, int motor2Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(motor2Port, MotorType.kBrushless);
    }

    public void autoOuttake() {
        speed = -kWheelSpeed;
    }

    public void autoIntake() {
        speed = kWheelSpeed;
    }

    public Command outtake() {
        return run(() -> {
            speed = -kWheelSpeed;
        });
    }

    public Command stopTele() {
        return run(() -> {
            speed = 0;
        });
    }

    public Command intake() {
        return run(() -> {
            speed = kWheelSpeed;
        });
    }

    public void stop() {
            System.out.println("STOP");
            speed = 0;
    }
    
    @Override
    public void periodic() {
        motor1.set(speed);
        motor2.set(-speed);

        System.out.println(speed);
    }
}
