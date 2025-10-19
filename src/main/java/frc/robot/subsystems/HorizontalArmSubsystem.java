package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.DigitalInput;  // UNCOMMENT when limit switch is added
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Horizontal Arm Subsystem with Motion Profiling
 * 
 * ARM SPECIFICATIONS:
 * - Total length: 23.25" (22.25" from pivot + 1" before pivot)
 * - Gear ratio: 20:1 (5:1 + 4:1)
 * - Pivot location: 2.5" above robot top, right side - Changing to 1.5" - need to check 1st
 * - Hook: 7" fixed perpendicular piece at end
 * - Material: Lightweight wood/plastic
 * 
 * POSITIONS:
 * - REST: 0° (horizontal, pointing back, on bracket base with 'rubber bumper support')
 * - ENGAGED: 170° (above horizontal, pointing forward/up) - guess...will tune to find out end resulting angle
 * - Total rotation: 170° (smooth motion with no "smack") - see above Engaged angle
 * 
 * CONTROL STRATEGY:
 * - Trapezoidal motion profiling for smooth acceleration/deceleration
 * - PID position control
 * - Gravity feedforward (kG) for holding against gravity
 * - SOFTWARE INTERLOCK: Control disabled until encoder is zeroed
 */
public class HorizontalArmSubsystem extends SubsystemBase {
    
    // Hardware
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    
    // ===== LIMIT SWITCH (FUTURE HARDWARE) =====
    // UNCOMMENT these lines when limit switch is installed:
    // private final DigitalInput restLimitSwitch = new DigitalInput(0); // TODO: Set DIO port
    // private static final boolean LIMIT_SWITCH_INSTALLED = true;
    private static final boolean LIMIT_SWITCH_INSTALLED = false; // Change to true when installed
    
    // Motion Profile
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;
    
    // Feedforward
    private final ArmFeedforward feedforward;
    
    // ===== SAFETY INTERLOCK =====
    // Prevents arm from moving until encoder is properly zeroed
    private boolean controlEnabled = false;
    
    // Constants - ARM GEOMETRY
    private static final double GEAR_RATIO = 25.0;
    private static final double ARM_LENGTH_INCHES = 22.25; // From pivot to end - probably useless but have in here as 'doc'
    
    // Constants - POSITION TARGETS (in degrees)
    public static final double REST_ANGLE = 0.0;      // Horizontal, pointing back
    public static final double ENGAGED_ANGLE = 130.0;    // 50° above horizontal eyeball and tune 
    
    // Constants - MOTION PROFILE LIMITS
    // Start conservative, tune based on testing
    private static final double MAX_VELOCITY_DEG_PER_SEC = 120.0;  // Tune this - wild guess
    private static final double MAX_ACCELERATION_DEG_PER_SEC_SQ = 240.0;  // Tune this - wild guess
    
    // Constants - CONTROL GAINS (tune these using the LIVE dashboard)
    private double kP = 0.02;    // Start small, increase until responsive
    private double kI = 0.0;     // Usually not needed
    private double kD = 0.0;     // Add if oscillation occurs
    private double kG = 0.05;    // Gravity compensation - tune this!
    
    // Constants - HARDWARE
    private static final int ARM_MOTOR_CAN_ID = 30; // Last available CAN SparkMax
    private static final double POSITION_TOLERANCE = 2.0; // degrees - need to play with from tuning
    
    // State tracking
    private boolean isCalibrationMode = false;
    
    public HorizontalArmSubsystem() {
        // Initialize motor
        armMotor = new SparkMax(ARM_MOTOR_CAN_ID, MotorType.kBrushless);    // Assuming we not going with a 770 and using NEO or NEO-550
        encoder = armMotor.getEncoder();
        pidController = armMotor.getClosedLoopController();
        
        // Configure motor - using the 2025 REV's Motor Configuration 
        configureMotor();
        
        // Initialize motion profile
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                MAX_VELOCITY_DEG_PER_SEC,
                MAX_ACCELERATION_DEG_PER_SEC_SQ
            )
        );
        
        // Initialize feedforward - need to tune there...starting realllllly small, might be too small
        feedforward = new ArmFeedforward(0.0, kG, 0.0, 0.0);
        
        // Start at rest position
        setpoint = new TrapezoidProfile.State(REST_ANGLE, 0.0);
        goal = new TrapezoidProfile.State(REST_ANGLE, 0.0);
        
        // Dashboard values
        initializeDashboard();
        
        // SAFETY: Start with control DISABLED
        controlEnabled = false;
        SmartDashboard.putString("Arm/Status", "NOT ZEROED - Position at REST and press BACK button");
        updateTopLevelStatus();
    }
    
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Motor configuration
        config.inverted(true);  // Direction confirmed in testing
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);    // Brake Mode, dont need it to squeeze too much on Algea
        config.smartCurrentLimit(30);  // most likely can go even way less than this...right now assuming using NEO, otherwise look up NEO550/others stalk
        
        // Encoder configuration (NEO is 42 counts per revolution)
        // Position = motor rotations * gear ratio = arm degrees
        // 1 motor rotation = 360 / GEAR_RATIO degrees of arm movement
        double positionConversionFactor = 360.0 / GEAR_RATIO;  // degrees per motor rotation
        double velocityConversionFactor = positionConversionFactor / 60.0;  // degrees per second
        
        config.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        
        // PID configuration
        config.closedLoop
            .pid(kP, kI, kD)
            .outputRange(-1.0, 1.0);
        
        // Apply configuration
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    // So using the periodic this will calc and update the pidController in the MaxSpark...need to check to make sure it's working...or not
    @Override
    public void periodic() {
        // ===== LIMIT SWITCH AUTO-ZERO (FUTURE FEATURE) =====
        // UNCOMMENT when limit switch is installed:
        /*
        if (LIMIT_SWITCH_INSTALLED && isAtRestSwitch() && !controlEnabled) {
            // Auto-zero when arm contacts limit switch
            encoder.setPosition(REST_ANGLE);
            controlEnabled = true;
            SmartDashboard.putString("Arm/Status", "Ready: Auto-zeroed at limit switch");
            updateTopLevelStatus();
        }
        */
        
        // ===== SAFETY INTERLOCK CHECK =====
        if (!controlEnabled) {
            // Don't run control until encoder is zeroed
            armMotor.stopMotor();
            updateTelemetry();
            SmartDashboard.putString("Arm/Status", "NOT ZEROED - Position at REST and press BACK button");
            SmartDashboard.putBoolean("Arm/Control Enabled", false);
            return; // Exit periodic - don't run motion control
        }
        
        // Control is enabled - normal operation
        SmartDashboard.putBoolean("Arm/Control Enabled", true);
        SmartDashboard.putString("Arm/Status", "Ready: Control Enabled");
        
        // Update PID gains from dashboard if in calibration mode
        if (isCalibrationMode) {
            updateGainsFromDashboard();
        }
        
        // Calculate next setpoint using motion profile
        setpoint = profile.calculate(0.02, setpoint, goal);  // 20ms loop time
        
        // Calculate feedforward based on current angle
        // Arm angle measured from horizontal (0° = horizontal) = REST; 90 = Vertical; 170 = ENGAGED
        // Need to convert to angle from vertical for cosine calculation
        double armAngleFromHorizontal = setpoint.position;
        double gravityFF = calculateGravityFeedforward(armAngleFromHorizontal);
        
        // Send position + feedforward to motor controller
        pidController.setReference(
            setpoint.position,
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            gravityFF
        );
        
        // Update telemetry
        updateTelemetry();
    }
    
    /**
     * Calculate gravity feedforward based on arm angle
     * Compensates for gravitational torque on the arm
     * 
     * Angle system: 0° = horizontal (rest), 90° = vertical up, 170° = engaged
     * 
     * Gravity effect:
     * - At 0° (horizontal): cos(0°) = 1.0 (maximum gravity torque)
     * - At 90° (vertical): cos(90°) = 0 (no gravity torque)
     * - At 170°: cos(170°) = -0.98 (gravity helps pull arm forward)
     */
    private double calculateGravityFeedforward(double angleDegrees) {
        double angleRadians = Math.toRadians(angleDegrees);
        return kG * Math.cos(angleRadians);
    }
    
    // ===== LIMIT SWITCH METHODS (FUTURE HARDWARE) =====
    // UNCOMMENT when limit switch is installed:
    /*
    private boolean isAtRestSwitch() {
        return !restLimitSwitch.get(); // Assumes normally-open switch
    }
    
    public boolean hasLimitSwitch() {
        return LIMIT_SWITCH_INSTALLED;
    }
    */
    
    // ===== POSITION COMMANDS =====
    
    public void goToRest() {
        goal = new TrapezoidProfile.State(REST_ANGLE, 0.0);
        SmartDashboard.putString("Arm/Target", "REST");
    }
    
    public void goToEngaged() {
        goal = new TrapezoidProfile.State(ENGAGED_ANGLE, 0.0);
        SmartDashboard.putString("Arm/Target", "ENGAGED");
    }
    
    public boolean atGoal() {
        return Math.abs(setpoint.position - goal.position) < POSITION_TOLERANCE &&
               Math.abs(setpoint.velocity) < 5.0;  // Near zero velocity
    }
    
    // ===== CALIBRATION & TUNING =====
    public void enableCalibrationMode() {
        isCalibrationMode = true;
        SmartDashboard.putBoolean("Arm/Calibration Mode", true);
    }
    
    public void disableCalibrationMode() {
        isCalibrationMode = false;
        SmartDashboard.putBoolean("Arm/Calibration Mode", false);
    }
    
    /**
     * Reset encoder to REST position (0°)
     * CRITICAL: Only call this when arm is physically at REST position!
     * This also ENABLES control after zeroing
     */
    public void resetEncoder() {
        encoder.setPosition(REST_ANGLE);
        controlEnabled = true; // Enable control after zeroing
        SmartDashboard.putString("Arm/Status", "✅ Encoder zeroed - Control ENABLED");
        SmartDashboard.putBoolean("Arm/Control Enabled", true);
        updateTopLevelStatus();
    }
    
    /**
     * Disable control (safety feature)
     * Call this to prevent arm from moving
     */
    public void disableControl() {
        controlEnabled = false;
        armMotor.stopMotor();
        SmartDashboard.putString("Arm/Status", "🛑 Control DISABLED");
        SmartDashboard.putBoolean("Arm/Control Enabled", false);
        updateTopLevelStatus();
    }
    
    /**
     * Check if control is enabled
     */
    public boolean isControlEnabled() {
        return controlEnabled;
    }
    
    // Read kP, kI, kD, kG from the LIVE Dashboard and use to update in live...using this to help tune....then edit the default values but can always tune again from that
    private void updateGainsFromDashboard() {
        double newKP = SmartDashboard.getNumber("Arm/Tune/kP", kP);
        double newKI = SmartDashboard.getNumber("Arm/Tune/kI", kI);
        double newKD = SmartDashboard.getNumber("Arm/Tune/kD", kD);
        double newKG = SmartDashboard.getNumber("Arm/Tune/kG", kG);
        
        // Separate the kP, kI, and kD from kG and only update as each 'group'
        if (newKP != kP || newKI != kI || newKD != kD) {
            kP = newKP;
            kI = newKI;
            kD = newKD;
            
            // Update PID gains using the newer REV API
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(kP, kI, kD);
            armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        // And if new kG, then update using the LIVE Dashboard value
        if (newKG != kG) {
            kG = newKG;
        }
    }
    
    // Put the kG, kP, kI, kD, etc up on the Dashboard
    private void initializeDashboard() {
        SmartDashboard.putBoolean("Arm/Calibration Mode", false);
        SmartDashboard.putBoolean("Arm/Control Enabled", false);
        SmartDashboard.putNumber("Arm/Tune/kP", kP);
        SmartDashboard.putNumber("Arm/Tune/kI", kI);
        SmartDashboard.putNumber("Arm/Tune/kD", kD);
        SmartDashboard.putNumber("Arm/Tune/kG", kG);
        SmartDashboard.putString("Arm/Target", "REST");
        SmartDashboard.putString("Arm/Status", "WARNING: NOT ZEROED");
    }

    // Updating the full telemetry on the ARM - probably on a new TAB for initially and then go from there...
    private void updateTelemetry() {
        SmartDashboard.putNumber("Arm/Current Angle", encoder.getPosition());
        SmartDashboard.putNumber("Arm/Current Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Arm/Setpoint Angle", setpoint.position);
        SmartDashboard.putNumber("Arm/Setpoint Velocity", setpoint.velocity);
        SmartDashboard.putNumber("Arm/Goal Angle", goal.position);
        SmartDashboard.putNumber("Arm/Position Error", goal.position - encoder.getPosition());
        SmartDashboard.putBoolean("Arm/At Goal", atGoal());
        SmartDashboard.putNumber("Arm/Motor Current", armMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Gravity FF", calculateGravityFeedforward(setpoint.position));
    }
    
    private void updateTopLevelStatus() {
        if (controlEnabled) {
            SmartDashboard.putString("ARM STATUS", "READY - Encoder Zeroed");
        } else {
            SmartDashboard.putString("ARM STATUS", "WARNING - NOT ZEROED - PRESS BACK at REST");
        }
    };

    public void stop() {
        armMotor.stopMotor();
    }

    // ===== Added Manual Control for Test Position/Direction =====
    public void setPercent(double percent) {
        armMotor.set(percent);
    }
    public Command manualUpCommand() {
        return run(() -> setPercent(0.12))
        .finallyDo(() -> stop())
        .withName("ManualUp");
    }
    public Command manualDownCommand() {
        return run(() -> setPercent(-0.12))
        .finallyDo(() -> stop())
        .withName("ManualDown");
    }
    
    // ===== COMMAND FACTORIES - think of these as 'methods' that can be called directly as COMMAND from up top, this makes this subsystem almost completely independent
    /**
     * Command to move arm to REST position (0°)
     * Returns when arm reaches target
     */
    public Command moveToRestCommand() {
        return runOnce(() -> goToRest())
            .andThen(run(() -> {}).until(this::atGoal))
            .withName("ArmToRest");
    }
    
    /**
     * Command to move arm to ENGAGED position (170°)
     * Returns when arm reaches target
     */
    public Command moveToEngagedCommand() {
        return runOnce(() -> goToEngaged())
            .andThen(run(() -> {}).until(this::atGoal))
            .withName("ArmToEngaged");
    }
    
    /**
     * Instant command to reset encoder
     * CRITICAL: Only use when arm is physically at REST position!
     * This also enables control after zeroing
     */
    public Command resetEncoderCommand() {
        return runOnce(this::resetEncoder)
            .withName("ResetArmEncoder");
    }
    
    /**
     * Command to disable control (safety)
     */
    public Command disableControlCommand() {
        return runOnce(this::disableControl)
            .withName("DisableArmControl");
    }
    
    /**
     * Command to enable calibration mode
     */
    public Command enableCalibrationCommand() {
        return runOnce(this::enableCalibrationMode)
            .withName("EnableCalibration");
    }
    
    /**
     * Command to disable calibration mode
     */
    public Command disableCalibrationCommand() {
        return runOnce(this::disableCalibrationMode)
            .withName("DisableCalibration");
    }
}