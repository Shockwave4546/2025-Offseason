// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.WheelDiameterCalibrationCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;
import java.util.function.Supplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // Add toggle to switch between field-relative and robot-relative driving
    private boolean robotOriented = false;

    // Add multiplier to adjust max speed of robot
    private double speedMultiplier = 1.0;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private boolean openLoop = false; // Default to closed-loop mode

    private final WheelDiameterCalibrationCommand wheelDiameterCalibrationCommand = new WheelDiameterCalibrationCommand(m_robotDrive);

    private final CoralSubsystem m_coralSubsystem = new CoralSubsystem(40);

      //angle snapping commnads and logic
    private double getNextSnapAngleCW(double currentAngleDeg) {
        currentAngleDeg = Math.IEEEremainder(currentAngleDeg, 360);
        if (currentAngleDeg < 0) currentAngleDeg += 360;

        for (double angle : DriveConstants.kSnapAngles) {
            if (Math.abs(currentAngleDeg - angle) <= DriveConstants.kSnapDeadzoneDeg) continue;
            if (angle > currentAngleDeg + 1e-3) return angle;
        }
    return DriveConstants.kSnapAngles[0]; // wrap around
    }

    private double getNextSnapAngleCCW(double currentAngleDeg) {
        currentAngleDeg = Math.IEEEremainder(currentAngleDeg, 360);
        if (currentAngleDeg < 0) currentAngleDeg += 360;

        for (int i = DriveConstants.kSnapAngles.length - 1; i >= 0; i--) {
            double angle = DriveConstants.kSnapAngles[i];
            if (Math.abs(currentAngleDeg - angle) <= DriveConstants.kSnapDeadzoneDeg) continue;
            if (angle < currentAngleDeg - 1e-3) return angle;
        }
    return DriveConstants.kSnapAngles[DriveConstants.kSnapAngles.length - 1]; // wrap around
    }

    private Command snapToAngleCommand(Supplier<Double> targetAngleSupplier) {
        ProfiledPIDController thetaController =
            new ProfiledPIDController(5.0, 0, 0,
                new TrapezoidProfile.Constraints(720, 720));
        thetaController.enableContinuousInput(-180, 180);

        return new RunCommand(() -> {
            double currentAngle = m_robotDrive.getHeading()+180;
            double target = Math.toDegrees(targetAngleSupplier.get());
            double rotationSpeed = thetaController.calculate(currentAngle, target);

        // Normal drive for translation, rotation controlled by PID
        m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            rotationSpeed,
            !robotOriented,
            openLoop
        );
    }, m_robotDrive).until(() -> Math.abs(thetaController.getPositionError()) < 2.0);
    }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Add shuffleboard speed multiplier
    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
                // Retrieve the speed multiplier from SmartDashboard
                double multiplier = SmartDashboard.getNumber("Speed Multiplier", 1.0);

                // Pass the multiplier to the drive method
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * multiplier,
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * multiplier,
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * multiplier,
                    !robotOriented,
                    openLoop);
            },
            m_robotDrive));
        
    // Configure autonomous commands
    configureAutonomousCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //When right bumper pessed set wheels to X mode
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //when Left bumper pressed zero the navx heading once
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    //While A button is Held Activate robot-relative
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> robotOriented = true));

    //While A button is not Held deactivate robot-relative
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onFalse(new InstantCommand(() -> robotOriented = false));

    // // Snap robot clockwise to next allowed angle
    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    //     .onTrue(snapToAngleCommand(() ->
    //     getNextSnapAngleCW(m_robotDrive.getHeading()+180)));

    // // Snap robot counter-clockwise to next allowed angle
    // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    //     .onTrue(snapToAngleCommand(() ->
    //     getNextSnapAngleCCW(m_robotDrive.getHeading()+180)));


    // When LT and RT are mostly pressed, run the Coral subsystem at 50% speed
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.1 && m_driverController.getRightTriggerAxis() < 0.7)
        .whileTrue(new RunCommand(
            () -> m_coralSubsystem.setRollerSpeed(0.18),
            m_coralSubsystem))
        .onFalse(new InstantCommand(
            () -> m_coralSubsystem.stopRoller(),
            m_coralSubsystem));

    // When LT and RT are mostly pressed, run the Coral subsystem at 50% speed
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.71)
        .whileTrue(new RunCommand(
            () -> m_coralSubsystem.setRollerSpeed(0.5),
            m_coralSubsystem))
        .onFalse(new InstantCommand(
            () -> m_coralSubsystem.stopRoller(),
            m_coralSubsystem));

    // When LT and RT are mostly pressed, run the Coral subsystem at 50% speed
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.1 && m_driverController.getLeftTriggerAxis() < 0.7)
        .whileTrue(new RunCommand(
            () -> m_coralSubsystem.setRollerSpeed(-0.18),
            m_coralSubsystem))
        .onFalse(new InstantCommand(
            () -> m_coralSubsystem.stopRoller(),
            m_coralSubsystem));

    // When LT and RT are mostly pressed, run the Coral subsystem at 50% speed
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.71)
        .whileTrue(new RunCommand(
            () -> m_coralSubsystem.setRollerSpeed(-0.5),
            m_coralSubsystem))
        .onFalse(new InstantCommand(
            () -> m_coralSubsystem.stopRoller(),
            m_coralSubsystem));
    
  }

  private void configureAutonomousCommands() {
    Trajectory straightPath = createStraightLineTrajectory();
    Trajectory sCurvePath = createSCurveTrajectory();

    Command straightAuto = createSwerveAutoCommand(straightPath)
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    Command sCurveAuto = createSwerveAutoCommand(sCurvePath)
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

    Command resetStraight = new InstantCommand(() -> m_robotDrive.resetOdometry(straightPath.getInitialPose()))
        .andThen(straightAuto);
    Command resetSCurve = new InstantCommand(() -> m_robotDrive.resetOdometry(sCurvePath.getInitialPose()))
        .andThen(sCurveAuto);

    autoChooser.setDefaultOption("Straight Path", resetStraight);
    autoChooser.addOption("S-Curve Path", resetSCurve);
    autoChooser.addOption("Wheel Diameter Calibration", wheelDiameterCalibrationCommand);

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Creates a swerve auto command to follow a given trajectory.
   *
   * @param trajectory The trajectory to follow.
   * @return A command that follows the trajectory using the swerve drive system.
   */
  private Command createSwerveAutoCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(1.0, 0.0, 0.0),
        new PIDController(1.0, 0.0, 0.0),
        new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 2.0)),
        m_robotDrive::setModuleStates,
        m_robotDrive
    );
  }

  /**
   * Creates a straight-line trajectory for autonomous.
   *
   * @return A trajectory that moves straight forward.
   */
  private Trajectory createStraightLineTrajectory() {
      return TrajectoryGenerator.generateTrajectory(
          List.of(
              new Pose2d(0, 0, new Rotation2d(0)), // Start position
              new Pose2d(1, 0, new Rotation2d(0))  // End position (3 meters forward)
          ),
          new TrajectoryConfig(1, 1.0) // Max speed and acceleration
      );
  }

  /**
   * Creates an S-curve trajectory for autonomous.
   *
   * @return A trajectory that follows an S-curve.
   */
  private Trajectory createSCurveTrajectory() {
      return TrajectoryGenerator.generateTrajectory(
          List.of(
              new Pose2d(0, 0, new Rotation2d(0)), // Start position
              new Pose2d(1.5, 1.5, new Rotation2d(Math.PI / 4)), // Midpoint (diagonal)
              new Pose2d(3, 0, new Rotation2d(0))  // End position (straight ahead)
          ),
          new TrajectoryConfig(2.0, 2.0) // Max speed and acceleration
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
