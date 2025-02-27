package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberPivot;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Hooks;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ScoringCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


// command-based is a "declarative" paragrim, so very little robot logic should be in handled in Robot.java
// strucutre of robot (including subsystems, commands, and button mappings) should be declared here

public class RobotContainer {
  // robot subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_elevator = new Elevator(14, 15); // CAN ID's
  private final DistanceSensor m_distanceSensor = new DistanceSensor();
  private final ElevatorPivot m_elevatorPivot = new ElevatorPivot(16);
  private final Hooks m_hooks = new Hooks(11);

  private final double kShelfDist = 130;
  private final double kLowReefDist = 180;
  private final double kMidReefDist = 200;
  private final double kHighReefDist = 250;

  // commands
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final ScoringCommand m_elevatorCommand = new ScoringCommand(m_elevator, m_distanceSensor, m_elevatorPivot, kShelfDist);

  // container; contains subsystems, OI devices, and commands
  public RobotContainer() {

    configureButtonBindings();
    m_elevator.clampElevatorSetpoints();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {

    // driving config
    new JoystickButton(m_driverController, Button.kR1.value)
      .whileTrue(new RunCommand(
          () -> m_robotDrive.setX(),
          m_robotDrive));

    // elevator preset commands
    new JoystickButton(m_operatorController, 1)
      .whileTrue(new ScoringCommand(m_elevator, m_distanceSensor, m_elevatorPivot, kShelfDist));
    new JoystickButton(m_operatorController, 2)
      .whileTrue(new ScoringCommand(m_elevator, m_distanceSensor, m_elevatorPivot, kLowReefDist));
    new JoystickButton(m_operatorController, 3)
      .whileTrue(new ScoringCommand(m_elevator, m_distanceSensor, m_elevatorPivot, kMidReefDist));
    new JoystickButton(m_operatorController, 4)
      .whileTrue(new ScoringCommand(m_elevator, m_distanceSensor, m_elevatorPivot, kHighReefDist));
    
    // elevator pivot control
    new JoystickButton(m_operatorController, 5).whileTrue(m_elevatorPivot.in()); // towards reef
    new JoystickButton(m_operatorController, 6).whileTrue(m_elevatorPivot.out()); // towards hp station

    // elevator manual control
    new JoystickButton(m_operatorController, 7).whileTrue(m_elevator.moveUp());
    new JoystickButton(m_operatorController, 8).whileTrue(m_elevator.moveDown());
  }

  // use to pass autonomous command to the main class
  public Command getAutonomousCommand() {

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    // all units in meters, s curve trajectory example
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}