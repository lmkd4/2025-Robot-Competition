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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BearingBlock;
import frc.robot.subsystems.ClimberPivot;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hooks;
import frc.robot.commands.ElevatorCommand;

import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.commands.ElevatorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


// command-based is a "declarative" paragrim, so very little robot logic should be in handled in Robot.java
// strucutre of robot (including subsystems, commands, and button mappings) should be declared here

public class RobotContainer {
  // robot subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_elevator = new Elevator(30, 31); // CAN ID's
  private final DistanceSensor m_distanceSensor = new DistanceSensor();
  private final ClimberPivot m_climberPivot = new ClimberPivot(12, 13);
  private final BearingBlock m_bearingBlock = new BearingBlock(14, 15);
  private final Hooks m_hooks = new Hooks(11);

  // commands
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final ElevatorCommand m_elevatorCommand = new ElevatorCommand(m_elevator, m_distanceSensor);

  // container; contains subsystems, OI devices, and commands
  public RobotContainer() {
    configureButtonBindings();

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

    new JoystickButton(m_driverController, Button.kR1.value)
      .whileTrue(new RunCommand(
          () -> m_robotDrive.setX(),
          m_robotDrive));

    /*
    new JoystickButton(m_operatorController, 1)
      .whileTrue(new StartEndCommand(
        () -> m_hooks.hooksIn(),
        () -> m_hooks.hooksOut(),
        m_hooks));
    
    new JoystickButton(m_operatorController, 2)
      .whileTrue(new StartEndCommand(
        () -> m_hooks.hooksOut(),
        () -> m_hooks.hooksIn(),
        m_hooks));
    // hooks out
    */
    
    new JoystickButton(m_operatorController, 1).whileTrue(m_hooks.hooksIn());
    new JoystickButton(m_operatorController, 2).whileTrue(m_hooks.hooksOut());
    /*
    climber pivot controls
    new JoystickButton(m_operatorController, 3)
      .whileTrue(new StartEndCommand(
        () -> m_climberPivot.pivotOut(),
        () -> m_climberPivot.pivotIn(),
        m_climberPivot));
    */

    new JoystickButton(m_operatorController, 3).whileTrue(m_climberPivot.pivotIn());
    new JoystickButton(m_operatorController, 4).whileTrue(m_climberPivot.pivotOut());

    new JoystickButton(m_operatorController, 5).whileTrue(m_bearingBlock.blockUp());
    new JoystickButton(m_operatorController, 6).whileTrue(m_bearingBlock.blockDown());
    

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