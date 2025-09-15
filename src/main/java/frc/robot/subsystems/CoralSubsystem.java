package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class CoralSubsystem extends SubsystemBase {
    // Motor controller for the coral rollers
    private final SparkMax m_coralRoller;

    // Constructor
    public CoralSubsystem(int kCoralCANId) {
        // Initialize the motor controller
        m_coralRoller = new SparkMax(kCoralCANId, MotorType.kBrushless);

        // Configure the motor
    }

    /**
     * Sets the speed of the coral rollers.
     *
     * @param speed The speed to set, from -1.0 to 1.0.
     */
    public void setRollerSpeed(double speed) {
        m_coralRoller.set(speed);
    }

    /**
     * Stops the coral rollers.
     */
    public void stopRoller() {
        m_coralRoller.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}