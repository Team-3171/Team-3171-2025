package frc.team3171.drive;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

// CTRE Imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// REV Imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.controllers.ThreadedPIDController;
import frc.team3171.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import static frc.team3171.HelperFunctions.Map;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class SwerveUnit implements RobotProperties {

    // Motor Controllers
    private final MOTOR_TYPE driveMotorType, slewMotorType;
    private final MotorController driveMotor, slewMotor;

    // Absolute Encoder
    private final ENCODER_TYPE selectedEncoderType;
    private final Object absoluteEncoder;

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
        driveMotorType = swerveUnitConfig.getDriveMotorType();
        switch (driveMotorType) {
            case REV_SPARKFLEX:
                driveMotor = new SparkFlex(swerveUnitConfig.getDriveMotorID(), MotorType.kBrushless);
                //((SparkFlex) driveMotor).restoreFactoryDefaults();
                ((SparkFlex) driveMotor).setInverted(swerveUnitConfig.isDriveMotorInverted());
                //((SparkFlex) driveMotor).setIdleMode(IdleMode.kBrake);
                //((SparkFlex) driveMotor).burnFlash();
                relativeDriveEncoder = ((SparkFlex) driveMotor).getEncoder();
                break;
            default:
                driveMotor = new SparkMax(swerveUnitConfig.getDriveMotorID(), MotorType.kBrushless);
                //((SparkMax) driveMotor).restoreFactoryDefaults();
                ((SparkMax) driveMotor).setInverted(swerveUnitConfig.isDriveMotorInverted());
                //((SparkMax) driveMotor).setIdleMode(IdleMode.kBrake);
                //((SparkMax) driveMotor).burnFlash();
                relativeDriveEncoder = ((SparkMax) driveMotor).getEncoder();
                break;
        }

        // Init the slew motor
        slewMotorType = swerveUnitConfig.getSlewMotorType();
        switch (slewMotorType) {
            case REV_SPARKFLEX:
                slewMotor = new SparkFlex(swerveUnitConfig.getSlewMotorID(), MotorType.kBrushless);
                //((SparkFlex) slewMotor).restoreFactoryDefaults();
                ((SparkFlex) slewMotor).setInverted(swerveUnitConfig.isSlewMotorInverted());
                //((SparkFlex) slewMotor).setIdleMode(IdleMode.kBrake);
                //((SparkFlex) slewMotor).burnFlash();
                relativeSlewEncoder = ((SparkFlex) slewMotor).getEncoder();
                break;
            default:
                slewMotor = new SparkMax(swerveUnitConfig.getSlewMotorID(), MotorType.kBrushless);
                //((SparkMax) slewMotor).restoreFactoryDefaults();
                ((SparkMax) slewMotor).setInverted(swerveUnitConfig.isSlewMotorInverted());
                //((SparkMax) slewMotor).setIdleMode(IdleMode.kBrake);
                //((SparkMax) slewMotor).burnFlash();
                relativeSlewEncoder = ((SparkMax) slewMotor).getEncoder();
                break;
        }

        // Init the absolute position encoder used for the slew angle
        selectedEncoderType = swerveUnitConfig.getAbsoluteEncoderType();
        switch (selectedEncoderType) {
            case CTRE:
                absoluteEncoder = new CANcoder(swerveUnitConfig.getAbsoluteEncoderID(), swerveUnitConfig.getCANBUS());
                CANcoderConfiguration absoluteEncoderConfiguration = new CANcoderConfiguration();
                //absoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
                absoluteEncoderConfiguration.MagnetSensor.SensorDirection = swerveUnitConfig.isAbsoluteEncoderInverted()
                        ? SensorDirectionValue.CounterClockwise_Positive
                        : SensorDirectionValue.Clockwise_Positive;
                ((CANcoder) absoluteEncoder).getConfigurator().apply(absoluteEncoderConfiguration);
                break;
            default:
                // Assumes duty cycle absolute encoder because the REV connector on the SparkMax SUCK
                absoluteEncoder = new DutyCycleEncoder(swerveUnitConfig.getAbsoluteEncoderID());
                break;
        }

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
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * given value
         */
        switch (driveMotorType) {
            case CTRE:
                ((TalonFX) driveMotor).set(speed);
                break;
            case REV_SPARKFLEX:
                ((SparkFlex) driveMotor).set(speed);
                break;
            default:
                ((SparkMax) driveMotor).set(speed);
                break;
        }
    }

    /**
     * Returns the current drive motor speed.
     * 
     * @return The current drive motor speed, from -1.0 to 1.0.
     */
    public double getDriveSpeed() {
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).get();
            case REV_SPARKFLEX:
                return ((SparkFlex) driveMotor).get();
            default:
                return ((SparkMax) driveMotor).get();
        }
    }

    /**
     * Sets whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param inverted
     *            Whether or not the direction of the drive motor need to be
     *            inverted.
     */
    public void setDriveInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonFX, and therefore it's
         * followers, need to be inverted
         */
        switch (driveMotorType) {
            case CTRE:
                ((TalonFX) driveMotor).setInverted(inverted);
                break;
            case REV_SPARKFLEX:
                ((SparkFlex) driveMotor).setInverted(inverted);
                break;
            default:
                ((SparkMax) driveMotor).setInverted(inverted);
                break;
        }
    }

    /**
     * Gets whether or not the direction of the drive motor is inverted.
     * 
     * @return True, if the drive motor is inverted, false otherwise.
     */
    public boolean getDriveInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's
         * followers, are inverted
         */
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).getInverted();
            case REV_SPARKFLEX:
                return ((SparkFlex) driveMotor).getInverted();
            default:
                return ((SparkMax) driveMotor).getInverted();
        }
    }

    /**
     * Updates the slew motors speed from the pid controller using the last updated
     * target angle.
     */
    public void updateSlewAngle() {
        // Update Slew Motor Speed
        switch (slewMotorType) {
            case CTRE:
                ((TalonFX) slewMotor).set(slewPIDController.getPIDValue());
                break;
            case REV_SPARKFLEX:
                ((SparkFlex) slewMotor).set(slewPIDController.getPIDValue());
                break;
            default:
                ((SparkMax) slewMotor).set(slewPIDController.getPIDValue());
                break;
        }
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
        switch (slewMotorType) {
            case CTRE:
                ((TalonFX) slewMotor).set(speed);
                break;
            case REV_SPARKFLEX:
                ((SparkFlex) slewMotor).set(speed);
                break;
            default:
                ((SparkMax) slewMotor).set(speed);
                break;
        }
    }

    /**
     * Returns the raw value of the {@link MotorController} integrated encoder.
     * 
     * @return The raw value from the {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderValue() {
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).getPosition().getValueAsDouble();
            default:
                return relativeDriveEncoder != null ? relativeDriveEncoder.getPosition() : 0;
        }
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
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).getVelocity().getValueAsDouble();
            default:
                return relativeDriveEncoder != null ? relativeDriveEncoder.getVelocity() : 0;
        }
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewAngle() {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        switch (selectedEncoderType) {
            case CTRE:
                mappedEncoderAngle = Map(((CANcoder) absoluteEncoder).getAbsolutePosition().getValueAsDouble(), 0, 1, 0, 360);
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                mappedEncoderAngle = 0;//Map(((DutyCycleEncoder) absoluteEncoder).getAbsolutePosition(), 0, 1, 0, 360);
                break;
        }
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
        switch (selectedEncoderType) {
            case CTRE:
                slewVelocity = ((CANcoder) absoluteEncoder).getVelocity().getValueAsDouble();
                break;
            default:
                switch (slewMotorType) {
                    case CTRE:
                        // If theres no cancoder and not a REV motor then get the value from the TalonFX integrated encoder
                        slewVelocity = ((TalonFX) slewMotor).getVelocity().getValueAsDouble();
                        break;
                    default:
                        // Assumes the encoder is wired into the slew motor spark max
                        slewVelocity = relativeSlewEncoder != null ? relativeSlewEncoder.getVelocity() : 0;
                        break;
                }
                break;
        }
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
        switch (driveMotorType) {
            case CTRE:
                ((TalonFX) driveMotor).disable();
                break;
            case REV_SPARKFLEX:
                ((SparkFlex) driveMotor).disable();
                break;
            default:
                ((SparkMax) driveMotor).disable();
                break;
        }
        switch (slewMotorType) {
            case CTRE:
                ((TalonFX) slewMotor).disable();
                break;
            case REV_SPARKFLEX:
                ((SparkFlex) slewMotor).disable();
                break;
            default:
                ((SparkMax) slewMotor).disable();
                break;
        }
        slewPIDController.disablePID();
    }

    /**
     * 
     * @param slewOffset
     */
    public void zeroModule(final double slewOffset) {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        switch (selectedEncoderType) {
            case CTRE:
                mappedEncoderAngle = Map(((CANcoder) absoluteEncoder).getAbsolutePosition().getValueAsDouble(), 0, 1, 0, 360);
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                mappedEncoderAngle = 0;//Map(((DutyCycleEncoder) absoluteEncoder).getAbsolutePosition(), 0, 1, 0, 360);
                break;
        }
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