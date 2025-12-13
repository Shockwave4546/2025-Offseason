// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class WheelDiameterCalibrationCommand extends Command {
    private static final double driveRadius = Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);


    private final DriveSubsystem drive;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelDiameter = 0.0;

    public WheelDiameterCalibrationCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = Math.toRadians(drive.getHeading());
        accumGyroYawRads = 0.0; 

        drive.setWheelRadiusCailbration();
        startWheelPositions = drive.getSwerveModulePositions();

        omegaLimiter.reset(0);
    }

    private static final int numRotations = 2;  // spin 2 full rotations

    @Override
    public boolean isFinished() {
        return accumGyroYawRads >= numRotations * 2 * Math.PI;
    }


    @Override
    public void execute() {
        // Spin in place at fixed angular speed (e.g. 0.2)
        drive.drive(0.0, 0.0, 0.2, true, true);

        // Update accumulated yaw radians
        double currentYaw = Math.toRadians(drive.getHeading());
        accumGyroYawRads += MathUtil.angleModulus(currentYaw - lastGyroYawRads);
        lastGyroYawRads = currentYaw;

        // Calculate average wheel travel distance since start
        double[] wheelPositions = drive.getSwerveModulePositions();
        double averageWheelDistance = 0.0;
        for (int i = 0; i < 4; i++) {
            averageWheelDistance += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }
        averageWheelDistance /= 4.0;

        // Expected distance traveled for N rotations
        double expectedDistance = numRotations * 2 * Math.PI * driveRadius;

        // Calculate correction factor for wheel diameter
        double correctionFactor = expectedDistance / averageWheelDistance;

        // Calculate corrected wheel diameter
        currentEffectiveWheelDiameter = ModuleConstants.kWheelDiameterMeters * correctionFactor;

        // Publish values to SmartDashboard
        SmartDashboard.putNumber("accumGyroYawRads", accumGyroYawRads);
        SmartDashboard.putNumber("averageWheelDistance", averageWheelDistance);
        SmartDashboard.putNumber("expectedDistance", expectedDistance);
        SmartDashboard.putNumber("correctionFactor", correctionFactor);
        SmartDashboard.putNumber("correctedWheelDiameter", currentEffectiveWheelDiameter);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted && accumGyroYawRads >= numRotations * 2 * Math.PI) {
            System.out.println("Effective Wheel Diameter: "
                    + Units.metersToInches(currentEffectiveWheelDiameter)
                    + " inches");
        } else {
            System.out.println("Not enough data for characterization");
        }
    }
}