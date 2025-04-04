// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

// CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// Team 3171 Imports
import frc.team3171.drive.SwerveDrive;
import frc.team3171.models.PhotonAprilTagTarget;
import frc.team3171.models.XboxControllerState;
import frc.team3171.pnuematics.DoublePistonController;
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.controllers.Elevator;
import frc.team3171.controllers.ThreadedPIDController;
import frc.team3171.controllers.VisionController;
//import static frc.team3171.HelperFunctions.Deadzone;
import static frc.team3171.HelperFunctions.Deadzone_With_Map;
import static frc.team3171.HelperFunctions.Get_Gyro_Displacement;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  // Controllers
  private XboxController driveController, operatorController;

  // Drive Objects
  private SwerveDrive swerveDrive;
  private Pigeon2 gyro;
  private ThreadedPIDController gyroPIDController;

  // Elevator Controller
  private DigitalInput feedSensor;
  private Elevator elevatorController;

  // Climber Objects
  private SparkMax climberMotor;

  // Pneumatics
  private Compressor compressor;

  // Auton Recorder
  private AutonRecorder autonRecorder;
  private ConcurrentLinkedQueue<AutonRecorderData> autonPlaybackQueue;
  private AutonRecorderData playbackData;
  private double autonStartTime;
  private boolean saveNewAuton;

  // Selected Auton String
  private boolean selectedAutonType;
  private String selectedAutonMode;

  // Shuffleboard Choosers
  private SendableChooser<Boolean> autonTypeChooser, fieldOrientationChooser;
  private SendableChooser<String> autonModeChooser;

  // Vision Controller
  private VisionController visionController;

  // LED Objects
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  // Global Variables
  private XboxControllerState driveControllerState, operatorControllerState;
  private double desiredElevatorPosition;
  private boolean robotOffGround;
  private volatile boolean fieldOrientationFlipped;
  private volatile boolean elevatorSafety;

  // Edge Triggers
  private boolean zeroEdgeTrigger;
  private boolean elevatorPositionEdgeTrigger;
  private boolean feederEdgeTrigger;
  private boolean elevatorSafetyEdgeTrigger;

  @Override
  public void robotInit() {
    // Controllers Init
    driveController = new XboxController(0);
    operatorController = new XboxController(1);

    // Sensors
    feedSensor = new DigitalInput(ELEVATOR_LINE_SENSOR_CHANNEL);
    gyro = new Pigeon2(GYRO_CAN_ID, "canivore");
    gyro.reset();

    // Drive Controller Init
    swerveDrive = new SwerveDrive(lr_Unit_Config, lf_Unit_Config, rf_Unit_Config, rr_Unit_Config);

    // Elevator Controller Init
    elevatorController = new Elevator();

    // Init the climber motors
    climberMotor = new SparkMax(CLIMBER_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(CLIMBER_INVERTED);
    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Pneumatics Init
    compressor = new Compressor(PCM_CAN_ID, PneumaticsModuleType.REVPH);
    compressor.enableAnalog(MIN_PRESSURE, MAX_PRESSURE);

    // PID Controllers
    gyroPIDController = new ThreadedPIDController(() -> Normalize_Gryo_Value(gyro.getAngle() + (fieldOrientationFlipped ? 180 : 0)), GYRO_KP, GYRO_KI, GYRO_KD, GYRO_MIN, GYRO_MAX,
        false);
    gyroPIDController.setMinValue(-180);
    gyroPIDController.setMaxValue(180);
    gyroPIDController.start();

    // Auton Recorder init
    autonRecorder = new AutonRecorder();
    autonPlaybackQueue = new ConcurrentLinkedQueue<>();
    playbackData = null;
    saveNewAuton = false;

    // Auton Type init
    selectedAutonType = false;
    autonTypeChooser = new SendableChooser<>();
    autonTypeChooser.setDefaultOption("Playback Auton", false);
    autonTypeChooser.addOption("Record Auton", true);

    // Auton Modes init
    selectedAutonMode = DEFAULT_AUTON;
    autonModeChooser = new SendableChooser<>();
    autonModeChooser.setDefaultOption(DEFAULT_AUTON, DEFAULT_AUTON);
    for (final String autonMode : AUTON_OPTIONS) {
      autonModeChooser.addOption(autonMode, autonMode);
    }

    // Field Orientation Chooser
    fieldOrientationChooser = new SendableChooser<>();
    fieldOrientationChooser.setDefaultOption("Pick an option", false);
    fieldOrientationChooser.addOption("0\u00B0", false);
    fieldOrientationChooser.addOption("180\u00B0", true);

    // Vision Controller Init
    visionController = new VisionController();
    visionController.shuffleboardTabInit("photonvision-front", "Front Cameras");
    visionController.shuffleboardTabInit("photonvision-rear", "Rear Cameras");

    // Global Variable Init
    driveControllerState = new XboxControllerState();
    operatorControllerState = new XboxControllerState();
    desiredElevatorPosition = 0;
    fieldOrientationFlipped = false;
    elevatorSafety = true;

    // Edge Triggers Init
    zeroEdgeTrigger = false;
    elevatorPositionEdgeTrigger = false;
    feederEdgeTrigger = false;
    elevatorSafetyEdgeTrigger = false;

    shuffleboardInit();

    m_ledBuffer = new AddressableLEDBuffer(288);
    // Set the data
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 128, 0);
    }
    m_led = new AddressableLED(0);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  private void shuffleboardInit() {
    ShuffleboardTab periodicTab = Shuffleboard.getTab("Periodic");

    // Auton Selectors
    periodicTab.add("Auton Type", autonTypeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    periodicTab.add("Auton Modes", autonModeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    periodicTab.add("Field Orientation Chooser", fieldOrientationChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    periodicTab.addBoolean("Flipped", () -> fieldOrientationFlipped);

    // Put the values on Shuffleboard
    periodicTab.addString("Gyro", () -> String.format("%.2f\u00B0 | %.2f\u00B0", gyroPIDController.getSensorValue(), gyroPIDController.getSensorLockValue()));
    periodicTab.addBoolean("Off Ground:", () -> robotOffGround);
    periodicTab.addBoolean("Line Sensor:", () -> feedSensor.get());
    periodicTab.addString("Air Pressure", () -> String.format("%.2f PSI", compressor.getPressure()));
    periodicTab.addBoolean("Elevator Safety:", () -> elevatorSafety);

    // Controller Values
    swerveDrive.shuffleboardInit("Swerve Debug");
    elevatorController.shuffleboardInit("Elevator Debug");
    visionController.shuffleboardTabInit("FRONT_TARGETING_CAMERA", "FRONT_TARGETING_CAMERA");

    if (DEBUG) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      debugTab.addString("Match Time:", () -> String.format("%.2f seconds", Timer.getMatchTime()));

      // Get the needed joystick values after calculating the deadzones
      double leftStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
      double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
      double rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

      // Calculate the left stick angle and magnitude
      double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
      double leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));

      // Calculate the field corrected drive angle
      double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroPIDController.getSensorValue()) : leftStickAngle;

      // Operator Controller Values
      debugTab.addString("Left Stick Angle", () -> String.format("%.2f\u00B0", leftStickAngle));
      debugTab.addString("Left Stick Velocity", () -> String.format("%.2f", leftStickMagnitude));
      debugTab.addString("Right Stick X", () -> String.format("%.2f", rightStickX));
      debugTab.addString("Field Adjusted Angle", () -> String.format("%.2f\u00B0", fieldCorrectedAngle));
    }
  }

  @Override
  public void robotPeriodic() {
    // Update the controller states
    driveControllerState = driveController.isConnected() ? new XboxControllerState(driveController) : new XboxControllerState();
    operatorControllerState = operatorController.isConnected() ? new XboxControllerState(operatorController) : new XboxControllerState();

    // Update off ground value
    robotOffGround = Math.abs(gyro.getRoll().getValueAsDouble()) > 5 || Math.abs(gyro.getPitch().getValueAsDouble()) > 5;

    // Field Orientation Chooser
    fieldOrientationFlipped = fieldOrientationChooser.getSelected().booleanValue();

    // Calibrate Swerve Drive
    final boolean zeroTrigger = driveController.getBackButton() && driveController.getStartButton() && isDisabled();
    if (zeroTrigger && !zeroEdgeTrigger) {
      // Zero the swerve units
      swerveDrive.zero();
      System.out.println("Swerve Drive has been calibrated!");
    }
    zeroEdgeTrigger = zeroTrigger;

    // LED Updates
    Color color = feedSensor.get() ? Color.kGreen
        : DriverStation.getAlliance().get() == Alliance.Blue ? Color.kBlue : Color.kRed;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void autonomousInit() {
    // Update Auton Selected Mode and load the auton
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    if (selectedAutonType) {
      playbackData = null;
    } else {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          disabledInit();
          playbackData = null;
          break;
        default:
          AutonRecorder.loadFromFile(autonPlaybackQueue, selectedAutonMode);
          playbackData = autonPlaybackQueue.poll();
          robotControlsInit();
          break;
      }
    }
    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAutonMode) {
      case DEFAULT_AUTON:
        disabledPeriodic();
        break;
      default:
        // Plays the recorded auton if theres a valid next step, otherwise disables
        if (playbackData != null) {
          // Get the controller states
          final XboxControllerState driveControllerState = playbackData.getDriveControllerState();
          final XboxControllerState operatorControllerState = playbackData.getOperatorControllerState();

          // Gyro Value
          final double gyroValue = gyroPIDController.getSensorValue();

          // Robot drive controls
          driveControlsPeriodic(driveControllerState, gyroValue);
          operatorControlsPeriodic(operatorControllerState, gyroValue);

          // Checks for new data and when to switch to it
          if ((Timer.getFPGATimestamp() - autonStartTime) >= playbackData.getFPGATimestamp()) {
            playbackData = autonPlaybackQueue.poll();
          }
        } else {
          selectedAutonMode = DEFAULT_AUTON;
          disabledInit();
        }
        break;
    }
  }

  @Override
  public void teleopInit() {
    // Update Auton Selected Mode and reset the data recorder
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = selectedAutonType;

    // Reset the robot controls
    robotControlsInit();

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void teleopPeriodic() {
    // Gyro Value
    final double gyroValue = gyroPIDController.getSensorValue();

    // Robot drive controls
    driveControlsPeriodic(driveControllerState, gyroValue);
    operatorControlsPeriodic(operatorControllerState, gyroValue);

    // Auton Recording
    final double autonTimeStamp = Timer.getFPGATimestamp() - autonStartTime;
    if (saveNewAuton && autonTimeStamp <= 15) {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          // Adds the recorded data to the auton recorder, but only if the data is new
          autonRecorder
              .addNewData(new AutonRecorderData(autonTimeStamp, driveControllerState, operatorControllerState));
          break;
      }
    }
  }

  @Override
  public void disabledInit() {
    // Disable all controllers
    swerveDrive.disable();
    gyroPIDController.disablePID();
    elevatorController.disable();
    climberMotor.disable();

    // Once auton recording is done, save the data to a file, if there is any
    if (saveNewAuton) {
      saveNewAuton = false;
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          autonRecorder.saveToFile(selectedAutonMode);
          break;
      }
    }
  }

  @Override
  public void disabledPeriodic() {
    // Do Nothing
  }

  @Override
  public void testInit() {
    // Do Nothing
  }

  @Override
  public void testPeriodic() {
    // Do Nothing
  }

  private void robotControlsInit() {
    // Reset the drive controller
    swerveDrive.driveInit();
    gyroPIDController.enablePID();
    gyroPIDController.updateSensorLockValue();

    desiredElevatorPosition = 0;
    elevatorSafety = true;

    // Edge Triggers reset
    zeroEdgeTrigger = false;
    elevatorPositionEdgeTrigger = false;
    feederEdgeTrigger = false;
    elevatorSafetyEdgeTrigger = false;
  }

  private void driveControlsPeriodic(final XboxControllerState driveControllerState, final double gyroValue) {
    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
    final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
    final double rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Calculate the left stick angle and magnitude
    final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
    double leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
    leftStickMagnitude = leftStickMagnitude > 1 ? 1 : leftStickMagnitude;

    // Calculate the field corrected drive angle
    final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

    // Drive Controls
    final boolean boostMode = driveControllerState.getYButton();
    final boolean targetLocking = false;// driveControllerState.getAButton();
    final boolean pickupLocking = false;// driveControllerState.getBButton();
    if (rightStickX != 0 || robotOffGround) {
      // Manual turning
      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, boostMode);
    } else if (targetLocking) {
      // April Tag Target Locking
      gyroPIDController.enablePID();

      // Aquire Targets
      final PhotonAprilTagTarget aprilTagTarget;
      final double offset;
      switch (DriverStation.getAlliance().get()) {
        case Red:
          // Target priority: 4, 3 w/ -5 offset, 5, 9 or 10
          aprilTagTarget = visionController.getAllVisibleAprilTagsByPriority(new int[] { 4, 5, 9, 10 }, "FRONT_TARGETING_CAMERA", "REAR_TARGETING_CAMERA");
          offset = aprilTagTarget == null ? 0 : aprilTagTarget.getPHOTON_TRACKED_TARGET().getFiducialId() == 3 ? -5 : 0;
          break;
        default:
          // Target priority: 7, 8 w/ -5 offset, 6, 1 or 2
          aprilTagTarget = visionController.getAllVisibleAprilTagsByPriority(new int[] { 7, 6, 1, 2 }, "FRONT_TARGETING_CAMERA", "REAR_TARGETING_CAMERA");
          offset = aprilTagTarget == null ? 0 : aprilTagTarget.getPHOTON_TRACKED_TARGET().getFiducialId() == 8 ? -5 : 0;
          break;
      }

      if (aprilTagTarget != null) {
        // Adjust the gyro lock to point torwards the target
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + aprilTagTarget.getPHOTON_TRACKED_TARGET().getYaw() + offset));
      }

      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, boostMode);
    } else if (pickupLocking) {
      // TODO Pickup Locking
      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, boostMode);
    } else if (driveControllerState.getPOV() != -1) {
      gyroPIDController.enablePID();

      final double dPadAngle = driveControllerState.getPOV();
      final boolean closeEnough = Math.abs(Get_Gyro_Displacement(gyroValue, gyroPIDController.getSensorLockValue())) <= 1;
      swerveDrive.drive(dPadAngle, .15, FIELD_ORIENTED_SWERVE ? (closeEnough ? 0 : gyroPIDController.getPIDValue()) : 0, boostMode);
    } else {
      // Normal gyro locking
      gyroPIDController.enablePID();

      final boolean closeEnough = Math.abs(Get_Gyro_Displacement(gyroValue, gyroPIDController.getSensorLockValue())) <= 1;
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? (closeEnough ? 0 : gyroPIDController.getPIDValue()) : 0, boostMode);
    }

    // Climber Controls
    final boolean button_climber_down = driveControllerState.getLeftBumper();
    final boolean button_climber_up = driveControllerState.getRightBumper();

    if (button_climber_down) {
      climberMotor.set(.75);
    } else if (button_climber_up) {
      climberMotor.set(-.75);
    } else {
      climberMotor.set(0);
    }

  }

  private void operatorControlsPeriodic(final XboxControllerState operatorControllerState, final double gyroValue) {
    // Get the needed joystick values after calculating the deadzones
    final double leftStickY = Deadzone_With_Map(OPERATOR_JOYSTICK_DEADZONE, -operatorControllerState.getLeftY(), -1, 1);

    // Get controls
    final boolean button_elevator_feed = operatorControllerState.getLeftBumper();
    final boolean button_elevator_pos_one = operatorControllerState.getAButton();
    final boolean button_elevator_pos_two = operatorControllerState.getBButton();
    final boolean button_elevator_pos_three = operatorControllerState.getYButton();
    final boolean button_elevator_pos_four = operatorControllerState.getXButton();

    final boolean button_run_feed_reverse = operatorControllerState.getLeftTriggerAxis() > .1;
    final boolean button_run_feed_forward = operatorControllerState.getRightTriggerAxis() > .1;
    final boolean button_run_feed_auto = operatorControllerState.getRightBumper();

    final boolean button_extend_pickup = ((int) operatorControllerState.getPOV()) == 0;
    final boolean button_retract_pickup = ((int) operatorControllerState.getPOV()) == 180;

    // Elevator Controls
    if (button_elevator_feed && !elevatorPositionEdgeTrigger) {
      elevatorController.retractPickup();
      desiredElevatorPosition = 2690;
    } else if (button_elevator_pos_one && !elevatorPositionEdgeTrigger) {
      elevatorController.retractPickup();
      desiredElevatorPosition = 0;
    } else if (button_elevator_pos_two && !elevatorPositionEdgeTrigger) {
      elevatorController.retractPickup();
      desiredElevatorPosition = 4143;
    } else if (button_elevator_pos_three && !elevatorPositionEdgeTrigger) {
      elevatorController.retractPickup();
      desiredElevatorPosition = 7305;
    } else if (button_elevator_pos_four && !elevatorPositionEdgeTrigger) {
      elevatorController.retractPickup();
      desiredElevatorPosition = 11561;
    } else if (Math.abs(leftStickY) > 0) {
      elevatorController.retractPickup();
      desiredElevatorPosition = elevatorController.getElevatorPosition();
      if (elevatorSafety) {
        elevatorController.setElevatorSpeed(leftStickY, ELEVATOR_LOWER_CUTOFF, ELEVATOR_UPPER_CUTOFF);
      } else {
        elevatorController.setElevatorSpeed(leftStickY);
      }
    } else {
      elevatorController.setElevatorPosition(desiredElevatorPosition, ELEVATOR_LOWER_CUTOFF, ELEVATOR_UPPER_CUTOFF);
    }
    elevatorPositionEdgeTrigger = button_elevator_feed || button_elevator_pos_one || button_elevator_pos_two || button_elevator_pos_three || button_elevator_pos_four;

    // Feed Controls
    if (button_run_feed_reverse) {
      elevatorController.setFeederSpeed(-.5);
    } else if (button_run_feed_forward) {
      elevatorController.setFeederSpeed(.5);
    } else if (button_run_feed_auto && !feederEdgeTrigger) {
      elevatorController.feedUntilClear(.5, feedSensor, .75, 1);
    } else {
      elevatorController.setFeederSpeed(0);
    }
    feederEdgeTrigger = button_run_feed_auto;

    if (button_extend_pickup) {
      elevatorController.extendPickup();
    } else if (button_retract_pickup) {
      elevatorController.retractPickup();
    }

    // Calibrate Swerve Drive
    final boolean elevatorSafetyTrigger = operatorControllerState.getBackButton() && operatorControllerState.getStartButton();
    if (elevatorSafetyTrigger && !elevatorSafetyEdgeTrigger) {
      elevatorSafety = false;
    }
    elevatorSafetyEdgeTrigger = elevatorSafetyTrigger;

  }

}
