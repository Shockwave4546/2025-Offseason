package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HorizontalArmSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.Constants.DriveConstants;
import java.util.List;

/**
 * Autonomous command for arm engagement sequence
 * 
 * Sequence:
 * 1. Drive forward 90 inches
 * 2. Stop and engage arm
 * 3. Drive backward 30 inches (timed)
 * 4. Pause 1 second
 * 5. Drive forward 30 inches (timed)
 * 6. Run coral intake
 */
public class ArmEngagementAuto extends SequentialCommandGroup {
    
    // Distance constants (in meters)
    private static final double FORWARD_DIST = 2.286;   // 90 inches
    private static final double SHORT_DIST_TIME = 0.8;  // Time to drive 30 inches - need to tune this!!! Bad bad bad way but not enough time to deal with pathplanner
    private static final double DRIVE_SPEED = 0.4;      // Speed for timed driving (tune this)
    
    public ArmEngagementAuto(
            DriveSubsystem driveSubsystem,
            HorizontalArmSubsystem armSubsystem,
            CoralSubsystem coralSubsystem) {
        
        // Create forward trajectory (90 inches)
        Trajectory forwardPath = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(FORWARD_DIST, 0, new Rotation2d(0))
            ),
            new TrajectoryConfig(1.0, 1.0)
        );
        
        // Create swerve controller command for forward path
        Command followForwardPath = createSwerveCommand(forwardPath, driveSubsystem);
        
        // Build the sequence
        addCommands(
            // Reset odometry to start position
            new InstantCommand(() -> driveSubsystem.resetOdometry(forwardPath.getInitialPose())),
            // Drive forward 90 inches using the old AUTO mode command
            followForwardPath,
            new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, false, false)),
            Commands.waitSeconds(0.5),  // Pause for 0.5s before engaging the ARM...should try to get rid of it and see.
            // Engage arm
            armSubsystem.moveToEngagedCommand(),
            Commands.waitSeconds(0.5),  // Pause for 0.5 before Going BACKWARD
            // Drive backward 30 inches (timed) - NEED TO TUNE THIS!!!
            Commands.run(
                () -> driveSubsystem.drive(-DRIVE_SPEED, 0, 0, false, false),
                driveSubsystem
            ).withTimeout(SHORT_DIST_TIME),
            new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, false, false)),
            armSubsystem.moveToRestCommand(),   // Put the ARM back to REST position
            Commands.waitSeconds(1.0),
            // Drive forward 30 inches (timed) - NEED TO TUNE THIS!!!  Hopefully the back and forth time is the SAME!!!
            Commands.run(
                () -> driveSubsystem.drive(DRIVE_SPEED, 0, 0, false, false),
                driveSubsystem
            ).withTimeout(SHORT_DIST_TIME),
            new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, false, false)),
            // Run coral intake
            new CoralAutoOn(coralSubsystem),
            new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, false, false))
        );
    }
    
    /**
     * Creates a swerve controller command to follow a trajectory
     */
    private Command createSwerveCommand(Trajectory trajectory, DriveSubsystem driveSubsystem) {
        return new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 2.0)),
            driveSubsystem::setModuleStates,
            driveSubsystem
        );
    }
}