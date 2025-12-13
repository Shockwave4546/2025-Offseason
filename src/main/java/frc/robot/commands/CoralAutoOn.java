package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class CoralAutoOn extends Command {
    private final CoralSubsystem m_coralSubsystem;
    private final Timer m_timer = new Timer();

    public CoralAutoOn(CoralSubsystem coralSubsystem) {
        m_coralSubsystem = coralSubsystem;
        addRequirements(m_coralSubsystem);  // referencing this specifc coralSubsystem in the subsystem
    }

    @Override
    public void initialize() {
        // set speed from 0.5 to 0 to not sends it out
        m_coralSubsystem.setRollerSpeed(0.5);
        m_timer.reset(); // Reset the timer to zero
        m_timer.start(); // Start counting
        System.out.println("Starting Coral command...");
    }

    @Override
    public void execute() {
        System.out.println("Timer is at: " + m_timer.get());
    }

    @Override
    public boolean isFinished() {
        // return true; // This is an instant command and we are NOT waiting for any sensor reading/checking but assumed the servo will just work.  might not always be the case but here it is
        if (m_timer.get() > 3.0) {
            System.out.println("Ending this crazy");
            m_coralSubsystem.setRollerSpeed(0.0);
            return true;    // End it
        } else {
            return false;
        }
    }
}
