package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.Blinkin;

public class BowWheels extends SubsystemBase {

    private Blinkin led;

    private final DigitalInput limitSwitch;
    private final SparkMax motor1;
    private final SparkMax motor2;
    private final double kWheelSpeed = 0.4;

    private double speed = 0;

    public BowWheels(int motor1Port, int motor2Port, int motor3Port, int switchPort) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(motor2Port, MotorType.kBrushless);

        limitSwitch = new DigitalInput(switchPort);
    }

    public void autoOuttake() {
        speed = -kWheelSpeed;
        led.wheels();
    }

    public void autoIntake() {
        speed = kWheelSpeed;
    }

    public Command outtake() {
        return run(() -> {
            speed = -kWheelSpeed;
        });
    }

    public Command fastOuttake() {
        return run(() -> {
            speed = -.5;
        });
    }

    public Command stopTele() {
        return run(() -> {
            speed = 0;
        });
    }

    public Command intake() {
        return run(() -> {
            if (!limitSwitch.get()) {
                speed = 0;
                led.recieved();
            }
            else {
                speed = .6;
                led.recieve();
            }
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
    }
}
