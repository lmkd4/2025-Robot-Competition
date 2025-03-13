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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberPivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.BowWheels;
import frc.robot.commands.AlignWithReef;
import frc.robot.commands.ElevatorCommand;
import frc.robot.limelight.Vision;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// command-based is a "declarative" paragrim, so very little robot logic should be in handled in Robot.java
// strucutre of robot (including subsystems, commands, and button mappings) should be declared here

public class RobotContainer {
  // robot subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_elevator = new Elevator(14, 15); // CAN ID's
  private final ElevatorPivot m_elevatorPivot = new ElevatorPivot(16);
  private final ClimberPivot m_climberPivot = new ClimberPivot(13, 12);
  private final BowWheels m_bowWheels = new BowWheels(17, 18);

  private final Vision reefLime = new Vision(m_robotDrive);

  // change elevator height here!
  private final ElevatorCommand m_elevatorCommandL1 = new ElevatorCommand(m_elevator, 40);  
  private final ElevatorCommand m_elevatorCommandL2 = new ElevatorCommand(m_elevator, 110);
  private final ElevatorCommand m_elevatorCommandL3 = new ElevatorCommand(m_elevator, 40);
  private final ElevatorCommand m_elevatorCommandL4 = new ElevatorCommand(m_elevator, 650);
  private final ElevatorCommand m_elevatorCommandL5 = new ElevatorCommand(m_elevator, 295);

  private final AlignWithReef alignWithReef = new AlignWithReef(m_robotDrive, reefLime);

  // align with reef command
  //private final AlignWithReef m_alignWithReef = new AlignWithReef(m_robotDrive, reefLight);

  // joystick initialization
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final CommandJoystick m_flightStick = new CommandJoystick(1);

  // container; contains subsystems, OI devices, and commands
  public RobotContainer() {

    configureButtonBindings();

    // set default commands
    m_elevatorPivot.homeSetpoints();
    m_elevatorPivot.setDefaultCommand(m_elevatorPivot.controlPivot());
    m_elevator.setDefaultCommand(m_elevator.getElevatorHeightCommand());

    reefLime.setDefaultCommand(reefLime.displayValues());
    // update limelight values on smart dashboard
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

    // L1 Command
    m_flightStick.button(7).onTrue(Commands.parallel(m_elevatorCommandL1, m_elevatorPivot.findSetpoint(-1.39)));

    // L2 Command
    m_flightStick.button(8).onTrue(Commands.parallel(m_elevatorCommandL2, m_elevatorPivot.findSetpoint(-1.39)));

    // L3 Command
    m_flightStick.button(9).onTrue(Commands.parallel(m_elevatorCommandL3, m_elevatorPivot.findSetpoint(1.74)));

    // L4 Command
    m_flightStick.button(10).onTrue(Commands.parallel(m_elevatorCommandL4, m_elevatorPivot.findSetpoint(1.74)));

    // L5 Command
    m_flightStick.button(11).onTrue(m_elevatorCommandL5.alongWith(m_elevatorPivot.findSetpoint(-1.29)));

    // manual pivot control
    m_flightStick.button(1).whileTrue(m_elevatorPivot.adjustSetpointUp());
    m_flightStick.button(2).whileTrue(m_elevatorPivot.adjustSetpointDown());

    // manual elevator control
    m_flightStick.button(3).whileTrue(m_elevator.moveUp());
    m_flightStick.button(4).whileTrue(m_elevator.moveDown());

    // manual elevator pivot control
    m_flightStick.button(5).whileTrue(m_bowWheels.intake());
    m_flightStick.button(6).whileTrue(m_bowWheels.outtake());

    new JoystickButton(m_driverController, 5).whileTrue(m_climberPivot.slowPivotIn());

    new JoystickButton(m_driverController, 3).whileTrue(alignWithReef);

    // manual climber pivot control
    new JoystickButton(m_driverController, 1).whileTrue(m_climberPivot.pivotIn());
    new JoystickButton(m_driverController, 2).whileTrue(m_climberPivot.pivotOut());
  }


  // use to pass autonomous command to the main class
  public Command getAutonomousCommand() {

    // Leave Community Trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    // all units in meters, s curve trajectory example
    Trajectory traj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-0.5, 0.02), new Translation2d(-1, -0.02)),
        new Pose2d(-1.5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        traj,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    m_robotDrive.resetOdometry(traj.getInitialPose());

    return swerveControllerCommand.andThen(
        () -> m_robotDrive.drive(0, 0, 0, false));
  }
}