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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.Set;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberPivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.BowWheels;
import frc.robot.commands.AlignWithReef;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FindHuman;
import frc.robot.commands.WheelsInAuto;
import frc.robot.commands.ScoringCommand;
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

  /*
  public final Vision reefLime = new Vision(m_robotDrive, "reefLime");
  public final Vision humanLime = new Vision(m_robotDrive, "humanLime");
  /*
   * 
   */
  public final Vision lime = new Vision(m_robotDrive);

  // change elevator height here!
  private final ScoringCommand kScoringCommandL1 = new ScoringCommand(m_elevator, m_elevatorPivot, 40, -1.39);
  private final ScoringCommand kScoringCommandL2 = new ScoringCommand(m_elevator, m_elevatorPivot, 110, -1.39);
  private final ScoringCommand kScoringCommandL3 = new ScoringCommand(m_elevator, m_elevatorPivot, 40, 1.74);
  private final ScoringCommand kScoringCommandL4 = new ScoringCommand(m_elevator, m_elevatorPivot, 650, 1.74);
  private final ScoringCommand kScoringCommandL5 = new ScoringCommand(m_elevator, m_elevatorPivot, 295, -1.29);

  // backup commands if ScoringCommand becomes unreliable
  /*
  private final ElevatorCommand kElevatorCommandL1 = new ElevatorCommand(m_elevator, 40);
  private final ElevatorCommand kElevatorCommandL2 = new ElevatorCommand(m_elevator, 110);
  private final ElevatorCommand kElevatorCommandL3 = new ElevatorCommand(m_elevator, 40);
  private final ElevatorCommand kElevatorCommandL4 = new ElevatorCommand(m_elevator, 650);
  private final ElevatorCommand kElevatorCommandL5 = new ElevatorCommand(m_elevator, 295);
  */

  // align with reef command
  private final AlignWithReef alignWithReef = new AlignWithReef(m_robotDrive, lime, "L");

  // joystick initialization
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final CommandJoystick m_flightStick = new CommandJoystick(1);

  public RobotContainer() {

    configureButtonBindings();

    m_elevatorPivot.homeSetpoints();
    m_elevatorPivot.setDefaultCommand(m_elevatorPivot.controlPivot());
    m_elevator.setDefaultCommand(m_elevator.getElevatorHeightCommand());
    lime.setDefaultCommand(lime.displayValues());

    m_bowWheels.setDefaultCommand(m_bowWheels.stop());

    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> {
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == Alliance.Red) {
                m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true);
                
                    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                        m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true);
                }
        }
    }, m_robotDrive));
}

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
      .whileTrue(new RunCommand(
          () -> m_robotDrive.setX(),
          m_robotDrive));

    m_flightStick.button(7).onTrue(kScoringCommandL1.andThen(m_elevator.stop()));
    m_flightStick.button(8).onTrue(kScoringCommandL2.andThen(m_elevator.stop()));
    m_flightStick.button(9).onTrue(kScoringCommandL3.andThen(m_elevator.stop()));
    m_flightStick.button(10).onTrue(kScoringCommandL4.andThen(m_elevator.stop()));
    m_flightStick.button(11).onTrue(kScoringCommandL5.andThen(m_elevator.stop()));

    /*
    m_flightStick.button(7).onTrue(kElevatorCommandL1.andThen(m_elevatorPivot.findSetpoint(0)));
    m_flightStick.button(8).onTrue(kElevatorCommandL2.andThen(m_elevatorPivot.findSetpoint(0)));
    m_flightStick.button(9).onTrue(kElevatorCommandL3.andThen(m_elevatorPivot.findSetpoint(0)));
    m_flightStick.button(10).onTrue(kElevatorCommandL4.andThen(m_elevatorPivot.findSetpoint(0)));
    m_flightStick.button(11).onTrue(kElevatorCommandL5.andThen(m_elevatorPivot.findSetpoint(0)));
    */

    // manual pivot control via setpoints
    m_flightStick.button(1).whileTrue(m_elevatorPivot.adjustSetpointUp());
    m_flightStick.button(2).whileTrue(m_elevatorPivot.adjustSetpointDown());

    // manual elevator control
    m_flightStick.button(3).whileTrue(m_elevator.moveUp());
    m_flightStick.button(4).whileTrue(m_elevator.moveDown());

    // manual bow wheel control
    m_flightStick.button(5).whileTrue(m_bowWheels.intake());
    m_flightStick.button(6).whileTrue(m_bowWheels.outtake());

    // controlled pivot in
    new JoystickButton(m_driverController, 5).whileTrue(m_climberPivot.slowPivotIn());

    // limelight? prob need to remove this

    new JoystickButton(m_driverController, 3).onTrue(alignWithReef);


    // manual climber pivot control
    new JoystickButton(m_driverController, 1).whileTrue(m_climberPivot.pivotIn());
    new JoystickButton(m_driverController, 2).whileTrue(m_climberPivot.pivotOut());
  }

  // use to pass autonomous command to the main class
  public Command leaveAndScoreCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory traj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-0.02, 0.02), new Translation2d(-0.025, 0.025)),
        new Pose2d(-0.03, 0.03, new Rotation2d(-0.78)),
        config);

    // all units in meters, drive forward ONLY
    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.03, 0.03, new Rotation2d(-0.78)),
        List.of(new Translation2d(-0.5, 0.02), new Translation2d(-1, -0.02)),
        new Pose2d(-.5, 0, new Rotation2d(-0.78)),
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

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
        traj1,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
    
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    m_robotDrive.resetOdometry(traj.getInitialPose());

    ScoringCommand autoScoringCommand = new ScoringCommand(m_elevator, m_elevatorPivot, 40, -1.39);
    WheelsInAuto bowWheelsCommand = new WheelsInAuto(m_bowWheels);
    
    return swerveControllerCommand.andThen(swerveControllerCommand1.andThen((
    autoScoringCommand.withTimeout(3.0).andThen(
        m_elevator.stop().withTimeout(.1).andThen(bowWheelsCommand.withTimeout(3).andThen(() -> m_robotDrive.drive(0, 0, 0, false)))))));
  }

  public Command doNothing() {
    return m_robotDrive.doNothing();
  }

  public Command leaveCommunityCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

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

    return swerveControllerCommand.andThen((() -> m_robotDrive.drive(0, 0, 0, false)));
  }

  private Command bargeToReef() {
    try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("bargeToReef");
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none(); // return a default command
    }
  }

  private Command reefToHP() {
    try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("reefToHP");
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none(); // return a default command
    }
  }

  private Command HPtoReef() {
    try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("HPtoReef");
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none(); // return a default command
    }
  }

  public Command findAndScore() {
    ScoringCommand runElevatorL5 = new ScoringCommand(m_elevator, m_elevatorPivot, 650, 1.74);
    WheelsInAuto scoreCoral = new WheelsInAuto(m_bowWheels);
    WheelsInAuto recieveCoral = new WheelsInAuto(m_bowWheels);
 
    FindHuman searchForHP = new FindHuman(m_robotDrive, lime);
    AlignWithReef searchForReef = new AlignWithReef(m_robotDrive, lime, "L");

    // run in parrallel until first command interrupts 
    ParallelRaceGroup findReef = new ParallelRaceGroup(
        bargeToReef(), 
        searchForReef
    );

    ParallelRaceGroup findHP = new ParallelRaceGroup(
        reefToHP(),
        searchForHP
    );

    ParallelRaceGroup findFinalPose = new ParallelRaceGroup(
        HPtoReef(),
        searchForHP
    );
    // find, the scoring command, then bow wheels for 2 seconds?, then HP station, then 
    return findReef.andThen(runElevatorL5.andThen(scoreCoral).andThen(findHP.andThen(recieveCoral).andThen(findFinalPose)));
    }

    public Command testPath() {

        WheelsInAuto runWheels = new WheelsInAuto(m_bowWheels);

        return Commands.defer(() -> {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile("run forward");
                return AutoBuilder.followPath(path);
            } catch (Exception e) {
                DriverStation.reportError("big oops: " + e.getMessage(), e.getStackTrace());
                return Commands.none(); // return a default command
            }
        }, Set.of(m_robotDrive)).andThen(kScoringCommandL1).andThen(runWheels); // Empty requirement set
    }
}
    