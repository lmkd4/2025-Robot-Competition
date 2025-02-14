package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;


import edu.wpi.first.wpilibj.I2C;

public class Hooks extends SubsystemBase {
    
    // Motors
    private final SparkMax motor1;

    // Encoder and PID controller
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    // Constants (modify these based on your elevator design)
    private static final double kElevatorSpeed = 0.5;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxHeight = 540.0; // Example max position
    private static final double kMinHeight = 75.0;

    private static final double lowLim = 0;
    private static final double highLim = 0;



    public Hooks(int motor1Port) {
        motor1 = new SparkMax(motor1Port, MotorType.kBrushless);

        // Encoder and PID controller from motor1
        encoder = motor1.getEncoder();
        pidController = motor1.getClosedLoopController();

        /*
        pidcontroller.setkP(kP);
        pidController.setkI(kI);
        pidController.setkD(kD);
        */
    }

    public void hooksIn() {
        motor1.set(0.1);
    }

    public void hooksOut() {
        motor1.set(-0.1);
    }

}