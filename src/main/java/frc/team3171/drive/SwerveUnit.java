package frc.team3171.drive;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.AnalogEncoder;

// REV Imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.controllers.ThreadedPIDController;
import static frc.team3171.HelperFunctions.Map;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class SwerveUnit implements RobotProperties {

    // Motor Controllers
    private final SparkFlex driveMotor;
    private final SparkMax slewMotor;

    // Absolute Encoder
    private final AnalogEncoder absoluteEncoder;

    // Relative Encoder
    private final RelativeEncoder relativeDriveEncoder, relativeSlewEncoder;

    // PID Controller
    private final ThreadedPIDController slewPIDController;
    private final ConcurrentLinkedQueue<String> slewPIDData;

    // Global Variables
    private double startingAngle;

    /**
     * Constructor
     *
     * @param driveInverted
     *            The config settings for the swerve unit.
     */
    public SwerveUnit(final SwerveUnitConfig swerveUnitConfig) {
        // Init the drive motor
        driveMotor = new SparkFlex(swerveUnitConfig.getDriveMotorID(), MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(swerveUnitConfig.isDriveMotorInverted());
        // Persist parameters to retain configuration in the event of a power cycle
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        relativeDriveEncoder = driveMotor.getEncoder();

        // Init the slew motor
        slewMotor = new SparkMax(swerveUnitConfig.getSlewMotorID(), MotorType.kBrushless);
        SparkMaxConfig slewConfig = new SparkMaxConfig();
        slewConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(swerveUnitConfig.isSlewMotorInverted());
        // Persist parameters to retain configuration in the event of a power cycle
        slewMotor.configure(slewConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        relativeSlewEncoder = slewMotor.getEncoder();

        // Init the absolute position encoder used for the slew angle
        absoluteEncoder = new AnalogEncoder(swerveUnitConfig.getAbsoluteEncoderID());
        absoluteEncoder.setInverted(swerveUnitConfig.isAbsoluteEncoderInverted());

        // Init the queue for pid data, if enabled
        slewPIDData = swerveUnitConfig.isLogPIDData() ? new ConcurrentLinkedQueue<String>() : null;

        // Init the gyro PID controller
        slewPIDController = new ThreadedPIDController(this::getSlewAngle, SLEW_KP, SLEW_KI, SLEW_KD, SLEW_PID_MIN, SLEW_PID_MAX, false);
        slewPIDController.setMinValue(-180);
        slewPIDController.setMaxValue(180);
        slewPIDController.start(20, true, slewPIDData);

        // Init the global variables
        startingAngle = 0;
    }

    /**
     * Sets the drive motor to the desired speed.
     *
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the drive motor to.
     */
    public void setDriveSpeed(final double speed) {
        driveMotor.set(speed);
    }

    /**
     * Returns the current drive motor speed.
     *
     * @return The current drive motor speed, from -1.0 to 1.0.
     */
    public double getDriveSpeed() {
        return driveMotor.get();
    }

    /**
     * Gets whether or not the direction of the drive motor is inverted.
     *
     * @return True, if the drive motor is inverted, false otherwise.
     */
    public boolean getDriveInverted() {
        return driveMotor.configAccessor.getInverted();
    }

    /**
     * Updates the slew motors speed from the pid controller using the last updated
     * target angle.
     */
    public void updateSlewAngle() {
        // Update Slew Motor Speed
        slewMotor.set(slewPIDController.getPIDValue());
    }

    /**
     * Sets the slew motor to the desired angle.
     *
     * @param angle
     *            The angle, from -180.0 to 180.0, to set the slew motor to.
     */
    public void updateSlewAngle(final double angle) {
        // Update the target angle of slew motor PID controller
        slewPIDController.updateSensorLockValueWithoutReset(angle);
        updateSlewAngle();
    }

    /**
     * Sets the slew motor to the desired speed.
     *
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the slew motor to.
     */
    public void setSlewSpeed(final double speed) {
        slewMotor.set(speed);
    }

    /**
     * Returns the raw value of the {@link MotorController} integrated encoder.
     *
     * @return The raw value from the {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderValue() {
        return relativeDriveEncoder != null ? relativeDriveEncoder.getPosition() : 0;
    }

    /**
     * Returns the velocity of the {@link MotorController} integrated encoder. If the drive motor type is of
     * {@link MOTOR_TYPE.CTRE}, then the encoder 2048 ticks per revolution and the return units of the velocity is in ticks
     * per 100ms. If the drive motor is of {@link MOTOR_TYPE.REV} then it returns the RPM of the motor.
     *
     * @return The velocity, in ticks per 100ms or RPM of the
     *         {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderVelocity() {
        return relativeDriveEncoder != null ? relativeDriveEncoder.getVelocity() : 0;
    }

    public double getRawSlewAngle() {
        return absoluteEncoder.get();
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     *
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewAngle() {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        mappedEncoderAngle = Map(absoluteEncoder.get(), 0, 1, 0, 360);

        return Normalize_Gryo_Value(mappedEncoderAngle - startingAngle);
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     *
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewTargetAngle() {
        return slewPIDController.getSensorLockValue();
    }

    /**
     * Returns the current velocity of the {@link CANCoder}.
     *
     * @return The velocity, in degrees per second.
     */
    public double getSlewVelocity() {
        final double slewVelocity;
        // Get the absolute encoder value based on encoder type
        // Assumes the encoder is wired into the slew motor spark max
        slewVelocity = relativeSlewEncoder != null ? relativeSlewEncoder.getVelocity() : 0;

        return slewVelocity;
    }

    /**
     * Enables the slew unit
     */
    public void enable() {
        slewPIDController.enablePID();
    }

    /**
     * Disables the drive and slew {@link TalonFX} motors.
     */
    public void disable() {
        driveMotor.disable();
        slewMotor.disable();
        slewPIDController.disablePID();
    }

    /**
     *
     * @param slewOffset
     */
    public void zeroModule(final double slewOffset) {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        mappedEncoderAngle = Map(absoluteEncoder.get(), 0, 1, 0, 360);

        startingAngle = Normalize_Gryo_Value(mappedEncoderAngle - slewOffset);
    }

    /**
     *
     */
    public void zeroModule() {
        zeroModule(0);
    }

    /**
     *
     * @return
     */
    public double getSlewOffset() {
        return startingAngle;
    }

    /**
     *
     * @param slewOffset
     */
    public void setSlewOffset(final double slewOffset) {
        startingAngle = slewOffset;
    }

    /**
     *
     * @return
     */
    public ConcurrentLinkedQueue<String> getSlewPIDData() {
        return slewPIDData;
    }

}