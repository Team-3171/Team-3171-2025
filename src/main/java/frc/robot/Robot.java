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
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

// CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;

// REV Imports
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Team 3171 Imports
import frc.team3171.drive.SwerveDrive;
import frc.team3171.models.PhotonAprilTagTarget;
import frc.team3171.models.ShooterShot;
import frc.team3171.models.XboxControllerState;
import frc.team3171.operator.Shooter;
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.controllers.ThreadedPIDController;
import frc.team3171.controllers.VisionController;
import static frc.team3171.HelperFunctions.Deadzone;
import static frc.team3171.HelperFunctions.Deadzone_With_Map;
import static frc.team3171.HelperFunctions.Get_Gyro_Displacement;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  // Controllers
  private XboxController driveController, operatorController;

  // Drive Objects
  private SwerveDrive swerveDrive;
  private Pigeon2 gyro;
  private ThreadedPIDController gyroPIDController;

  // Shooter Objects
  private Shooter shooterController;
  private DigitalInput feedSensor;
  private ColorSensorV3 upperFeedColorSensor;

  // Linear Actuators
  //private CANSparkMax leftAcuator, rightAcuator;

  // New Lift Motor
  private SparkMax newLiftMotor;

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
  private boolean robotOffGround;
  private volatile boolean fieldOrientationFlipped;
  private double shooterAtSpeedStartTime;

  // Edge Triggers
  private boolean zeroEdgeTrigger;
  private boolean pickupEdgeTrigger;
  private boolean shooterButtonEdgeTrigger;
  private boolean shooterAtSpeedEdgeTrigger;

  private boolean doLayoverShot;

  @Override
  public void robotInit() {
    // Controllers Init
    driveController = new XboxController(0);
    operatorController = new XboxController(1);

    doLayoverShot = false;

    // Drive Controller Init
    swerveDrive = new SwerveDrive(lr_Unit_Config, lf_Unit_Config, rf_Unit_Config, rr_Unit_Config);

    // Shooter Controller Init
    try {
      shooterController = new Shooter();
    } catch (Exception e) {
      e.printStackTrace();
    }
    feedSensor = new DigitalInput(5);

    // Linear Acuators
    /*leftAcuator*/newLiftMotor = new SparkMax(LEFT_ACUATOR_CAN_ID, MotorType.kBrushless);
    // rightAcuator = new CANSparkMax(RIGHT_ACUATOR_CAN_ID, MotorType.kBrushed);

    // Sensors
    gyro = new Pigeon2(GYRO_CAN_ID, "canivore");
    gyro.reset();
    upperFeedColorSensor = new ColorSensorV3(Port.kMXP);

    // PID Controllers
    gyroPIDController = new ThreadedPIDController(() -> Normalize_Gryo_Value(gyro.getAngle() + (fieldOrientationFlipped ? 180 : 0)), GYRO_KP, GYRO_KI, GYRO_KD,
        GYRO_MIN, GYRO_MAX, false);
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
    // visionController.shuffleboardTabInit("FRONT_TARGETING_CAMERA", "Front Cameras");
    // visionController.shuffleboardTabInit("REAR_TARGETING_CAMERA", "Rear Cameras");

    // Global Variable Init
    driveControllerState = new XboxControllerState();
    operatorControllerState = new XboxControllerState();
    fieldOrientationFlipped = false;
    shooterAtSpeedStartTime = 0;

    // Edge Triggers Init
    zeroEdgeTrigger = false;
    pickupEdgeTrigger = false;
    shooterButtonEdgeTrigger = false;
    shooterAtSpeedEdgeTrigger = false;

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

  @SuppressWarnings("resource")
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

    // Colors Sensor Values
    periodicTab.addBoolean("Line Sensor", () -> !feedSensor.get());
    final int red = upperFeedColorSensor.getBlue(), green = upperFeedColorSensor.getGreen(), blue = upperFeedColorSensor.getBlue();
    periodicTab.addString("Upper Feed Sensor:", () -> String.format("[R: %d, G: %d, B: %d] | Prox: %d", red, green, blue, upperFeedColorSensor.getProximity()));

    // Controller Values
    swerveDrive.shuffleboardInit("Swerve Debug");
    shooterController.shuffleboardInit("Shooter Debug");
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
      leftStickMagnitude = leftStickMagnitude > 1 ? 1 : leftStickMagnitude;

      // Calculate the field corrected drive angle
      double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroPIDController.getSensorValue()) : leftStickAngle;

      // Operator Controller Values
      // debugTab.addString("Left Stick Angle", () -> String.format("%.2f\u00B0", leftStickAngle));
      // debugTab.addString("Left Stick Velocity", () -> String.format("%.2f", leftStickMagnitude));
      // debugTab.addString("Right Stick X", () -> String.format("%.2f", rightStickX));
      // debugTab.addString("Field Adjusted Angle", () -> String.format("%.2f\u00B0", fieldCorrectedAngle));
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
    Color color = !feedSensor.get() ? Color.kGreen : DriverStation.getAlliance().get() == Alliance.Blue ? Color.kBlue : Color.kRed;
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
          autonRecorder.addNewData(new AutonRecorderData(autonTimeStamp, driveControllerState, operatorControllerState));
          break;
      }
    }
  }

  @Override
  public void disabledInit() {
    // Disable all controllers
    swerveDrive.disable();
    gyroPIDController.disablePID();
    /*leftAcuator*/newLiftMotor.disable();
    //rightAcuator.disable();
    shooterController.disable();

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
    shooterController.shooterTiltStartMatch();

    // Global Variable reset
    shooterAtSpeedStartTime = 0;

    // Edge Triggers reset
    zeroEdgeTrigger = false;
    pickupEdgeTrigger = false;
    shooterButtonEdgeTrigger = false;
    shooterAtSpeedEdgeTrigger = false;
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

    // Layover button
    final boolean button_layover = operatorControllerState.getLeftBumper();
    if (button_layover) {
      shooterController.setShooterTiltPosition(60);
      doLayoverShot = true;
    }

    // If the layover button is not pressed, but the previous cycle it was.
    if (!button_layover && doLayoverShot) {
      shooterController.setShooterTiltPosition(0);
      doLayoverShot = false;
    }

    // Drive Controls
    final boolean boostMode = driveControllerState.getYButton();
    final boolean targetLocking = driveControllerState.getAButton();
    final boolean pickupLocking = driveControllerState.getBButton();
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
    } else {
      // Normal gyro locking
      gyroPIDController.enablePID();

      // Quick Turning
      if (driveControllerState.getPOV() != -1) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(driveControllerState.getPOV()));
      }

      final boolean closeEnough = Math.abs(Get_Gyro_Displacement(gyroValue, gyroPIDController.getSensorLockValue())) <= 1;
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? (closeEnough ? 0 : gyroPIDController.getPIDValue()) : 0, boostMode);
    }

    if (button_layover) {

    }

    // Lift Controls
    if (DriverStation.isFMSAttached() && DriverStation.isTeleop() && Timer.getMatchTime() < 30) {
      final boolean raiseLeftArm = driveControllerState.getLeftBumper();
      final boolean lowerLeftArm = Deadzone(.02, driveControllerState.getLeftTriggerAxis()) != 0;
      /*leftAcuator*/newLiftMotor.set(raiseLeftArm ? 1 : lowerLeftArm ? -1 : 0);

      final boolean raiseRightArm = driveControllerState.getRightBumper();
      final boolean lowerRightArm = Deadzone(.02, driveControllerState.getRightTriggerAxis()) != 0;
      // rightAcuator.set(raiseRightArm ? 1 : lowerRightArm ? -1 : 0);

      if (raiseLeftArm || lowerLeftArm || raiseRightArm || lowerRightArm) {
        // Force the shooter down
        shooterController.shooterTiltEndMatch();
      }
    } else {
      final boolean raiseLeftArm = driveControllerState.getLeftBumper();
      final boolean lowerLeftArm = Deadzone(.02, driveControllerState.getLeftTriggerAxis()) != 0;
      /*leftAcuator*/newLiftMotor.set(raiseLeftArm ? 1 : lowerLeftArm ? -1 : 0);

      final boolean raiseRightArm = driveControllerState.getRightBumper();
      final boolean lowerRightArm = Deadzone(.02, driveControllerState.getRightTriggerAxis()) != 0;
      // rightAcuator.set(raiseRightArm ? 1 : lowerRightArm ? -1 : 0);

      if (raiseLeftArm || lowerLeftArm || raiseRightArm || lowerRightArm) {
        // Force the shooter down
        shooterController.shooterTiltEndMatch();
      }
    }

  }

  private void operatorControlsPeriodic(final XboxControllerState operatorControllerState, final double gyroValue) {
    // Get the needed joystick values after calculating the deadzones
    final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -operatorControllerState.getLeftY());

    // Get shooter controls
    final boolean button_Pickup = operatorControllerState.getRightTriggerAxis() > .02;
    final boolean button_Reverse_Feed = operatorControllerState.getLeftTriggerAxis() > .02;
    final boolean button_Short_Shot = operatorControllerState.getBButton();
    final boolean button_Normal_Shot = operatorControllerState.getAButton();
    final boolean button_Far_Shot = operatorControllerState.getXButton();
    final boolean button_Yeet_Shot = operatorControllerState.getYButton();
    final boolean button_Shooter = button_Short_Shot || button_Normal_Shot || button_Far_Shot || button_Yeet_Shot;

    // Shooter controls start
    final ShooterShot selectedShot;
    if (button_Short_Shot) {
      // Short shot velocity
      selectedShot = SHOOTER_SHOTS.get("SHORT_SHOT");
    } else if (button_Normal_Shot) {
      // Normal shot velocity
      selectedShot = SHOOTER_SHOTS.get("NORMAL_SHOT");
    } else if (button_Far_Shot) {
      // Far shot velocity
      selectedShot = SHOOTER_SHOTS.get("FAR_SHOT");
    } else if (button_Yeet_Shot) {
      // Yeet shot velocity
      selectedShot = SHOOTER_SHOTS.get("YEET_SHOT");
    } else {
      selectedShot = null;
    }

    // Shooter Control
    final boolean shooterTiltWithinRange = Math
        .abs(Get_Gyro_Displacement(shooterController.getShooterTilt(), shooterController.getShooterTiltSetPosition())) < SHOOTER_TILT_ALLOWED_DEVIATION;
    if (button_Shooter && !shooterButtonEdgeTrigger) {
      // Shooter Start
      shooterAtSpeedEdgeTrigger = false;
      if (selectedShot != null) {
        shooterController.setShooterVelocity(selectedShot.lowerShooterRPM, selectedShot.upperShooterRPM);
      } else {
        shooterController.setShooterSpeed(1);
      }
    } else if (button_Shooter) {
      switch (DriverStation.getAlliance().get()) {
        case Red:
          shooterController.setShooterTiltPosition(gyroValue < -135 || gyroValue > 45 ? selectedShot.getShooterAngle() : -selectedShot.getShooterAngle());
          break;
        default:
          shooterController.setShooterTiltPosition(gyroValue < -45 || gyroValue > 135 ? selectedShot.getShooterAngle() : -selectedShot.getShooterAngle());
          break;
      }
      // Check if the shooter is at speed
      final boolean isShooterAtSpeed = selectedShot == null ? true : shooterController.isBothShootersAtVelocity(SHOOTER_TILT_ALLOWED_DEVIATION);
      if (isShooterAtSpeed && !shooterAtSpeedEdgeTrigger) {
        // Get time that shooter first designated at speed
        shooterAtSpeedStartTime = Timer.getFPGATimestamp();
      } else if (shooterTiltWithinRange && isShooterAtSpeed && (Timer.getFPGATimestamp() >= shooterAtSpeedStartTime + SHOOTER_DESIRED_AT_SPEED_TIME)) {
        // Feed the ball through the shooter
        shooterController.setLowerFeederSpeed(LOWER_FEED_SHOOT_SPEED);
        shooterController.setUpperFeederSpeed(UPPER_FEED_SHOOT_SPEED);
      } else if (!feedSensor.get()) {
        // Back off the ball from the feed sensor
        shooterController.setLowerFeederSpeed(LOWER_FEED_BACKFEED_SPEED);
        shooterController.setUpperFeederSpeed(UPPER_FEED_BACKFEED_SPEED);
      } else {
        // Feeder stopped while shooter gets up to speeed
        shooterController.setLowerFeederSpeed(0);
        shooterController.setUpperFeederSpeed(0);
      }
      shooterAtSpeedEdgeTrigger = isShooterAtSpeed;
    } else {
      // Ring Pickup Controls
      if (button_Pickup && !pickupEdgeTrigger) {
        // Pickup controls start
        shooterController.setShooterTiltPosition(0);
        shooterController.setShooterSpeed(0);
      } else if (button_Pickup) {
        // Pickup controls while held
        shooterController.setShooterTiltPosition(0);
        if (shooterTiltWithinRange) {
          if (!feedSensor.get()) {
            shooterController.setLowerFeederSpeed(0);
            shooterController.setUpperFeederSpeed(0);
          } else {
            shooterController.setLowerFeederSpeed(LOWER_FEED_PICKUP_SPEED);
            shooterController.setUpperFeederSpeed(UPPER_FEED_PICKUP_SPEED);
          }
        } else {
          shooterController.setLowerFeederSpeed(0);
          shooterController.setUpperFeederSpeed(0);
        }
      } else if (pickupEdgeTrigger && (!feedSensor.get())) {
        // Pickup control when ended
        shooterController.setLowerFeederSpeed(0);
        shooterController.runUpperFeeder(UPPER_FEED_END_SPEED, UPPER_FEED_END_TIME);
      } else if (button_Reverse_Feed) {
        switch (DriverStation.getAlliance().get()) {
          case Red:
            shooterController.setShooterTiltPosition(gyroValue > -135 && gyroValue < 45 ? 40 : -40);
            break;
          default:
            shooterController.setShooterTiltPosition(gyroValue > -45 && gyroValue < 135 ? 40 : -40);
            break;
        }
        shooterController.setShooterSpeed(SHOOTER_REVERSE_FEED_SPEED);
        shooterController.setLowerFeederSpeed(0);
        if (!feedSensor.get()) {
          shooterController.setUpperFeederSpeed(UPPER_FEED_BACKFEED_SPEED);
        } else {
          shooterController.setUpperFeederSpeed(0);
        }
      } else {
        // Disable both feeders and the shooter
        shooterController.setShooterSpeed(0);
        shooterController.setLowerFeederSpeed(0);
        shooterController.setUpperFeederSpeed(0);
      }
    }

    // Manual Tilt Controls
    if (leftStickY != 0) {
      shooterController.setShooterTiltSpeed(leftStickY / 4);
    } else {
      shooterController.setShooterTiltPosition();
    }

    // Edge Trigger Updates
    pickupEdgeTrigger = button_Pickup;
    shooterButtonEdgeTrigger = button_Shooter;
  }

}
