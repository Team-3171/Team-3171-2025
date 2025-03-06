package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

// FRC Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.team3171.sensors.RevEncoderAbsolute;
import frc.team3171.sensors.RevEncoderRelative;
import frc.team3171.sensors.ThreadedPIDController;

/**
 * @author Mark Ebert
 */
public class Elevator implements RobotProperties {

    // Elevator Motors
    private final SparkMax elevatorMotor, elevatorFollower;
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
        // Init the master motor
        elevatorMotor = new SparkMax(ELEVATOR_ONE_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(ELEVATOR_INVERTED);
        elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Init the slave motors
        elevatorFollower = new SparkMax(ELEVATOR_TWO_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
        elevatorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).follow(elevatorMotor, true);
        elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Elevator encoder init
        elevatorEncoder = new RevEncoderRelative(ELEVATOR_ENCODER_CHANNEL_A, ELEVATOR_ENCODER_CHANNEL_B);
        elevatorEncoder.reset();

        elevatorPIDController = new ThreadedPIDController(elevatorEncoder, ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD,
                ELEVATOR_PID_MIN, ELEVATOR_PID_MAX, false);
        elevatorPIDController.start(false);

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
        elevatorMotor.set(speed);
    }

    /**
     * 
     * @param speed
     */
    public void setElevatorSpeed(final double speed, final double lowerBound, final double upperBound) {
        elevatorPIDController.disablePID();
        final double elevatorPosition = getElevatorPosition();
        if (speed < 0 && elevatorPosition <= lowerBound) {
            elevatorMotor.set(0);
        } else if (speed > 0 && elevatorPosition >= upperBound) {
            elevatorMotor.set(0);
        } else {
            elevatorMotor.set(speed);
        }
    }

    /**
     * 
     * @param position
     */
    public void setElevatorPosition(final double position, final double lowerBound, final double upperBound) {
        final double ElevatorPosition = getElevatorPosition();
        // final int winchTwoPosition = elevatorTwo.getEncoderValue();
        if (position < ElevatorPosition && ElevatorPosition <= lowerBound) {
            elevatorPIDController.disablePID();
            elevatorMotor.set(0);
        } else if (position > ElevatorPosition && ElevatorPosition >= upperBound) {
            elevatorPIDController.disablePID();
            elevatorMotor.set(0);
        } else {
            elevatorPIDController.enablePID();
            elevatorPIDController.updateSensorLockValueWithoutReset(position);
            elevatorMotor.set(elevatorPIDController.getPIDValue());
        }
        SmartDashboard.putNumber("PID", elevatorPIDController.getPIDValue());
    }

    /**
     * 
     * @return The current encoder clicks.
     */
    public double getElevatorPosition() {
        return elevatorPIDController.getSensorValue();
    }

    public double getElevatorDesiredPoition() {
        return elevatorPIDController.getSensorLockValue();
    }

    /**
     * Sets the acuator to the given speed for the specified duration.
     * 
     * @param duration
     *                 The duration that the acuator will run for before stopping.
     */
    public void setAcuatorSpeed(final double speed, final double duration) {
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
                            // leftAcuator.set(ControlMode.PercentOutput, speed);
                            // rightAcuator.set(ControlMode.PercentOutput, speed);
                            Timer.delay(.02);
                        }
                        // leftAcuator.set(ControlMode.PercentOutput, 0);
                        // rightAcuator.set(ControlMode.PercentOutput, 0);
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
     *                The timeout that the acuator will not exeed.
     */
    public void extendAcuatorToSensor(final double speed, final DigitalInput sensor, final double timeout) {
        try {
            executorLock.lock();
            if (executorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + timeout;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (!sensor.get() || DriverStation.isDisabled()) {
                                break;
                            }
                            // leftAcuator.set(ControlMode.PercentOutput, speed);
                            // rightAcuator.set(ControlMode.PercentOutput, speed);
                            Timer.delay(.02);
                        }
                        // leftAcuator.set(ControlMode.PercentOutput, 0);
                        // rightAcuator.set(ControlMode.PercentOutput, 0);
                    } finally {
                        executorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    public void setAcuatorSpeed(final double speed) {
        if (!executorActive.get()) {
            // leftAcuator.set(ControlMode.PercentOutput, speed);
            // rightAcuator.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * 
     */
    public void disable() {
        elevatorPIDController.disablePID();
        elevatorMotor.disable();
    }

    public void shuffleboardInit(final String tabName) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab(tabName);

        // shooterTab.addBoolean("Shooter At Speed:", () ->
        // isBothShootersAtVelocity(SHOOTER_TILT_ALLOWED_DEVIATION));
        shooterTab.addString("Elevator Position:",
                () -> String.format("%.2f | %.2f", getElevatorPosition(), getElevatorDesiredPoition()));
        // shooterTab.addString("Upper Shooter RPM:", () -> String.format("%.2f | %.2f",
        // getUpperShooterVelocity(), getUpperShooterTargetVelocity()));
        // shooterTab.addString("Shooter Tilt:", () -> String.format("%.2f | %.2f",
        // getShooterTilt(), getShooterTiltSetPosition()));
        // shooterTab.addString("Shooter Tilt Raw:", () -> String.format("%.2f",
        // getShooterTiltRaw()));
    }

}
