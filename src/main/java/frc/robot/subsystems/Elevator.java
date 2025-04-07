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
import edu.wpi.first.units.Measure;
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
 
    private final SparkMax motor1;
    private final SparkMax motor2;

    public final PIDController elevatorController;

    private static final double kP = 0.01;
    private static final double kI = 0.0;
    private static final double kD = 0.0;


    private static double elevatorSetpoint = 0;
    private final double kElevatorSpeed = 0.2;

    public LaserCan lc;

    public Elevator(int motor1Port, int motor2Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(motor2Port, MotorType.kBrushless);

        elevatorController = new PIDController(kP, kI, kD);
        elevatorController.setTolerance(0.5);

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
        
        Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putNumber("Target:", measurement.distance_mm);
            return measurement.distance_mm;
        }
        return Double.NaN;  
    }

    public void setElevatorSetpoint(double setpoint) {
        elevatorSetpoint = setpoint;
        SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
    }

    public void controlElevator() {
        System.out.println("CONTROLLING ELEVATOR");
        double currentHeight = getElevatorHeight();
        
        if (!Double.isNaN(currentHeight)) {
            double error = elevatorSetpoint - currentHeight;
            
            if (Math.abs(error) < 10.0) {
                motor1.set(0);
                motor2.set(0);
                return;
            }
            
            double output = elevatorController.calculate(currentHeight, elevatorSetpoint);
           
            if (Math.abs(output) < 0.05) { 
                output = 0;
            }

            motor1.set(output);
            motor2.set(-output);
        }
    }    

    public Command moveToHeight(double height) {
        return run(() -> setElevatorSetpoint(height));
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

    public Command stop() {
        return run(() -> {
            motor1.set(0);
            motor2.set(0);
        });
    }

    public boolean isStable() {
        return Math.abs(elevatorController.getVelocityError()) < 1.0;
    }

    public boolean withinThreshold(double target) {
        return Math.abs(target - getElevatorHeight()) < 10;
    }
    
    public Command moveUp() {
        return run(() -> {
            motor1.set(kElevatorSpeed);
            motor2.set(-kElevatorSpeed );
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

    public Command control() {
        return run(() -> {
            double currentHeight = getElevatorHeight();
        
            if (!Double.isNaN(currentHeight)) {
                double error = elevatorSetpoint - currentHeight;
                
                if (Math.abs(error) < 10.0) {
                    motor1.set(0);
                    motor2.set(0);
                    return;
                }
                double output = elevatorController.calculate(currentHeight, elevatorSetpoint);
               
                if (Math.abs(output) < 0.05) { 
                    output = 0;
                }
    
                motor1.set(output);
                motor2.set(-output);
            }
        });
    }

    @Override
    public void periodic() {
        
    }
}