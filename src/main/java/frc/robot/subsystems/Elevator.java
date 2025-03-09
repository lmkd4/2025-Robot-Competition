package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.simulation.MockLaserCan;
import java.lang.Double;

import edu.wpi.first.wpilibj.I2C;

public class Elevator extends SubsystemBase {
    
    // Motors
    private final SparkMax motor1;
    private final SparkMax motor2;

    // Encoder and PID controller
    private final PIDController feedback = new PIDController(kP, kI, kD);
    private final SimpleMotorFeedforward feedfoward = new SimpleMotorFeedforward(0.1, 0.2);
    private final SlewRateLimiter elevatorRateLimiter = new SlewRateLimiter(1);

    // Constants (modify these based on your elevator design)
    private static final double kElevatorSpeed = 0.55;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static double elevatorSetpoint = 0;
    
    private static final double kMaxHeight = 540.0; // Example max position
    private static final double kMinHeight = 75.0;

    private static final double lowLim = 0;
    private static final double highLim = 0;
    private LaserCan lc;

    public Elevator(int motor1Port, int motor2Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(motor2Port, MotorType.kBrushless);

        CanBridge.runTCP();

        lc = new LaserCan(0);

    
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        System.out.println("finished elevator config");

    }

    public double getElevatorHeight() {

        LaserCan.Measurement measurement = lc.getMeasurement();

        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putNumber("Target:", measurement.distance_mm);
            System.out.println("Target is " + measurement.distance_mm + " away");
            return measurement.distance_mm;
        }

        return Double.NaN;  
    }

    public Command getElevatorHeightCommand() {
        return run(() -> {
            getElevatorHeight();
            motor1.set(0);
            motor2.set(0);
        });
    }

    public boolean supplierCondition() {
        return false;
    }

    public void stop1() {
        motor1.set(0);
        motor2.set(0);
    }
    public Command stop() {
        return run(() -> {
            motor1.set(0);
            motor2.set(0);
        });
    }

    public Command moveUp() {
        return run(() -> {
            motor1.set(kElevatorSpeed);
            motor2.set(-kElevatorSpeed);
        });
    }

    public Command moveDown() {
        return run(() -> {
            motor1.set(-kElevatorSpeed);
            motor2.set(kElevatorSpeed);
        });
    }

    public boolean condition() {
        return false;
    }
    
    // necessary for command
    public void moveUp1() {
        motor1.set(kElevatorSpeed);
        motor2.set(-kElevatorSpeed);
    }

    public void moveDown1() {
        motor1.set(-kElevatorSpeed);
        motor2.set(kElevatorSpeed);
    }
}