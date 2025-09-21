package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// Get the constants from the constants file where the On/Off is defined
import frc.robot.Constants.ServoConstants;
// Get the ServoSubsystem so it can be referenced to
import frc.robot.subsystems.ServoSubsystem;

public class ServoFullOff extends Command {
    private final ServoSubsystem m_ServoSubsystem;

    public ServoFullOff(ServoSubsystem servoSubsystem) {
        m_ServoSubsystem = servoSubsystem;
        addRequirements(m_ServoSubsystem);  // referencing this specifc servo in the subsystem
    }

    @Override
    public void initialize() {
        m_ServoSubsystem.setServoPosition(ServoConstants.kFullOff);
    }

    @Override
    public boolean isFinished() {
        return true; // This is an instant command and we are NOT waiting for any sensor reading/checking but assumed the servo will just work.  might not always be the case but here it is
    }
}
