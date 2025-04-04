package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

// FRC Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DigitalInput;

// REV Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.pnuematics.DoublePistonController;
import frc.team3171.sensors.RevEncoderRelative;

/**
 * @author Mark Ebert
 */
public class Elevator implements RobotProperties {

    // Motor Controllers
    private final SparkMax elevatorLeaderMotor, elevatorFollowerMotor;
    private final SparkMax feedLeaderMotor, feedFollowerMotor;

    // Piston
    private DoublePistonController pickup;

    // Sensors
    private final RevEncoderRelative elevatorEncoder;

    // PID Controller
    private final ThreadedPIDController elevatorPIDController;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean executorActive;

    /**
     *
     */
    public Elevator() {
        // Init the elevator motors
        elevatorLeaderMotor = new SparkMax(ELEVATOR_LEADER_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(IdleMode.kBrake).inverted(ELEVATOR_INVERTED);
        elevatorLeaderMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorFollowerMotor = new SparkMax(ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
        elevatorFollowerConfig.idleMode(IdleMode.kBrake).follow(elevatorLeaderMotor, true);
        elevatorFollowerMotor.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Init the feeder motors
        feedLeaderMotor = new SparkMax(FEED_LEADER_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig lowerShooterConfig = new SparkMaxConfig();
        lowerShooterConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(FEEDER_INVERTED);
        feedLeaderMotor.configure(lowerShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Init the slave motors
        feedFollowerMotor = new SparkMax(FEED_FOLLOWER_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig upperShooterConfig = new SparkMaxConfig();
        upperShooterConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).follow(feedLeaderMotor, true);
        feedFollowerMotor.configure(upperShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Elevator Encoder Init
        elevatorEncoder = new RevEncoderRelative(ELEVATOR_ENCODER_CHANNEL_A, ELEVATOR_ENCODER_CHANNEL_B);
        elevatorEncoder.reset();

        // Pneumatics Init
        pickup = new DoublePistonController(PCM_CAN_ID, PneumaticsModuleType.REVPH, PICKUP_FORWARD_CHANNEL, PICKUP_REVERSE_CHANNEL, PICKUP_INVERTED);

        // PID Controller Init
        elevatorPIDController = new ThreadedPIDController(this::getElevatorPosition, ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD, ELEVATOR_PID_MIN, ELEVATOR_PID_MAX, true);
        elevatorPIDController.start(false);

        // Executor Services
        this.executorService = Executors.newSingleThreadExecutor();
        this.executorLock = new ReentrantLock(true);
        this.executorActive = new AtomicBoolean();
    }

    /**
     *
     * @param speed
     */
    public void setElevatorSpeed(final double speed) {
        elevatorPIDController.disablePID();
        elevatorLeaderMotor.set(speed);
    }

    /**
     *
     * @param speed
     */
    public void setElevatorSpeed(final double speed, final double lowerBound, final double upperBound) {
        elevatorPIDController.disablePID();
        final double elevatorPosition = getElevatorPosition();
        if (speed < 0 && elevatorPosition <= lowerBound) {
            elevatorLeaderMotor.set(0);
        } else if (speed > 0 && elevatorPosition >= upperBound) {
            elevatorLeaderMotor.set(0);
        } else {
            elevatorLeaderMotor.set(speed);
        }
    }

    /**
     *
     * @param desiredPosition
     */
    public void setElevatorPosition(final double desiredPosition, final double lowerBound, final double upperBound) {
        final double currentPosition = getElevatorPosition();
        // final int winchTwoPosition = elevatorTwo.getEncoderValue();
        if (desiredPosition < currentPosition && currentPosition <= lowerBound) {
            elevatorPIDController.disablePID();
            elevatorLeaderMotor.set(0);
        } else if (desiredPosition > currentPosition && currentPosition >= upperBound) {
            elevatorPIDController.disablePID();
            elevatorLeaderMotor.set(0);
        } else {
            elevatorPIDController.enablePID();
            elevatorPIDController.updateSensorLockValueWithoutReset(desiredPosition);
            elevatorLeaderMotor.set(elevatorPIDController.getPIDValue());
        }
    }

    /**
     *
     * @return The current encoder clicks.
     */
    public double getElevatorPosition() {
        return elevatorEncoder.getAsDouble();
    }

    /**
     *
     * @return
     */
    public double getElevatorDesiredPoition() {
        return elevatorPIDController.getSensorLockValue();
    }

    /**
     *
     * @param speed
     */
    public void setFeederSpeed(final double speed) {
        if (!executorActive.get()) {
            feedLeaderMotor.set(speed);
        }
    }

    /**
     * Sets the acuator to the given speed for the specified duration.
     *
     * @param duration
     *            The duration that the acuator will run for before stopping.
     */
    public void setFeederSpeed(final double speed, final double duration) {
        try {
            executorLock.lock();
            if (executorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + duration;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            feedLeaderMotor.set(speed);
                            Timer.delay(.02);
                        }
                        feedLeaderMotor.set(0);
                    } finally {
                        executorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the acuator to the given speed for the specified duration.
     *
     * @param timeout
     *            The timeout that the acuator will not exeed.
     */
    public void feedUntilClear(final double speed, final DigitalInput sensor, final double timeout, final double delay) {
        try {
            executorLock.lock();
            if (executorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        pickup.extend();
                        Timer.delay(delay);
                        final double endTime = Timer.getFPGATimestamp() + timeout;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (!sensor.get() || DriverStation.isDisabled()) {
                                break;
                            }
                            feedLeaderMotor.set(speed);
                            Timer.delay(.02);
                        }
                        feedLeaderMotor.set(0);
                        pickup.retract();
                    } finally {
                        executorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    public void retractPickup() {
        pickup.retract();
    }

    public void extendPickup() {
        pickup.extend();
    }

    /**
     *
     */
    public void disable() {
        elevatorPIDController.disablePID();
        elevatorLeaderMotor.disable();
        feedLeaderMotor.disable();
        pickup.disable();
    }

    public void shuffleboardInit(final String tabName) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab(tabName);

        // shooterTab.addBoolean("Shooter At Speed:", () ->
        // isBothShootersAtVelocity(SHOOTER_TILT_ALLOWED_DEVIATION));
        shooterTab.addString("Elevator Position:", () -> String.format("%.2f | %.2f", getElevatorPosition(), getElevatorDesiredPoition()));
        shooterTab.addString("PID:", () -> String.format("%.5f", elevatorPIDController.getPIDValue()));
        // shooterTab.addString("Shooter Tilt:", () -> String.format("%.2f | %.2f",
        // getShooterTilt(), getShooterTiltSetPosition()));
        // shooterTab.addString("Shooter Tilt Raw:", () -> String.format("%.2f",
        // getShooterTiltRaw()));
    }

}
