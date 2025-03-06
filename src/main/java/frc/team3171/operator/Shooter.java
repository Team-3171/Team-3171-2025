package frc.team3171.operator;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

// FRC Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

// REV Imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// Team 3171 Imports
import frc.robot.RobotProperties;

/**
 * @author Mark Ebert
 */
public class Shooter implements RobotProperties {

    // Motor Controllers
    private final SparkMax leaderShooterMotor, followerShooterMotor;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean shooterExecutorActive;

    /**
     * Constructor
     * 
     * @throws Exception
     *                   Throws a new exception if there are an invalid amount of
     *                   motors in the feederCANIDArray.
     */
    public Shooter() throws Exception {
        // Init all of the motors
        leaderShooterMotor = new SparkMax(LEADER_SHOOTER_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig lowerShooterConfig = new SparkMaxConfig();
        lowerShooterConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(SHOOTER_INVERTED);
        leaderShooterMotor.configure(lowerShooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Init the slave motors
        followerShooterMotor = new SparkMax(FOLLOWER_SHOOTER_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig upperShooterConfig = new SparkMaxConfig();
        upperShooterConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).follow(leaderShooterMotor, true);
        followerShooterMotor.configure(upperShooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Initialize the executor service for concurrency
        executorService = Executors.newFixedThreadPool(1);
        executorLock = new ReentrantLock(true);

        // Initialize the AtomicBooleans to control the thread executors
        shooterExecutorActive = new AtomicBoolean(false);
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     * 
     * @param speed
     *              The speed, from -1.0 to 1.0, to set the shooter motors to.
     */
    public void setShooterSpeed(final double speed) {
        leaderShooterMotor.set(speed);
    }

    /**
     * Sets the speed of the shooter motors to the given value and keeps them
     * running for the desired time.
     * 
     * @param speed
     *                The speed, from -1.0 to 1.0, to set the shooter motors to.
     * @param runTime
     *                The amount of time, in seconds, to keep the motors spinning
     *                for.
     */
    public void runShooter(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (shooterExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            leaderShooterMotor.set(speed);
                            Timer.delay(TimedRobot.kDefaultPeriod);
                        }
                        leaderShooterMotor.set(0);
                    } finally {
                        shooterExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed
     *                The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime
     *                The amount of time, in seconds, to keep the motors spinning
     *                for.
     */
    public void pulseShooter(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (shooterExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            leaderShooterMotor.set(speed);
                            Timer.delay(.02);
                        }
                        endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            leaderShooterMotor.set(0);
                            Timer.delay(.02);
                        }
                        leaderShooterMotor.set(0);
                    } finally {
                        shooterExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Disables all motors in the {@link Shooter} class.
     */
    public final void disable() {
        leaderShooterMotor.disable();
        followerShooterMotor.disable();
    }

    public void shuffleboardInit(final String tabName) {
        // ShuffleboardTab shooterTab = Shuffleboard.getTab(tabName);

        // shooterTab.addBoolean("Shooter At Speed:", () ->
        // isBothShootersAtVelocity(SHOOTER_TILT_ALLOWED_DEVIATION));
        // shooterTab.addString("Lower Shooter RPM:", () -> String.format("%.2f | %.2f",
        // getLowerShooterVelocity(), getLowerShooterTargetVelocity()));
        // shooterTab.addString("Upper Shooter RPM:", () -> String.format("%.2f | %.2f",
        // getUpperShooterVelocity(), getUpperShooterTargetVelocity()));
        // shooterTab.addString("Shooter Tilt:", () -> String.format("%.2f | %.2f",
        // getShooterTilt(), getShooterTiltSetPosition()));
        // shooterTab.addString("Shooter Tilt Raw:", () -> String.format("%.2f",
        // getShooterTiltRaw()));
    }

}