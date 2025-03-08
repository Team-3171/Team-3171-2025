package frc.robot;

// Java Imports
import java.util.HashMap;

// FRC Imports
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

// Team 3171 Imports
import frc.team3171.drive.SwerveUnitConfig;
import frc.team3171.drive.SwerveUnitConfig.SwerveUnitConfigBuilder;
import frc.team3171.models.PhotonCameraConfig;
import frc.team3171.models.ShooterShot;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Debug Options **/
        public static final boolean DEBUG = true;
        public static final boolean SWERVE_DIRECTION_DEBUG = false;
        public static final String PID_LOG_ADDRESS = "10.31.71.202";

        /** Drive Variables **/
        public static final boolean FIELD_ORIENTED_SWERVE = true;
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = .85, MAX_ROTATION_SPEED = .6;
        public static final boolean PINWHEEL_ZERO_ORIENTATION = false;
        public static final boolean SWERVE_UNIT_ORIENTATION_OPTIMIZATION = true;

        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig lf_Unit_Config = new SwerveUnitConfigBuilder(1, 2, 3).setAbsoluteEncoderInverted(true).build();
        public static final SwerveUnitConfig lr_Unit_Config = new SwerveUnitConfigBuilder(5, 6, 2).setAbsoluteEncoderInverted(true).build();
        public static final SwerveUnitConfig rf_Unit_Config = new SwerveUnitConfigBuilder(3, 4, 1).setAbsoluteEncoderInverted(true).build();
        public static final SwerveUnitConfig rr_Unit_Config = new SwerveUnitConfigBuilder(7, 8, 0).setAbsoluteEncoderInverted(true).build();

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 10;
        public static final int FEED_LEADER_CAN_ID = 9, FEED_FOLLOWER_CAN_ID = 10;
        public static final int ELEVATOR_LEADER_CAN_ID = 11, ELEVATOR_FOLLOWER_CAN_ID = 12;
        public static final int CLIMBER_CAN_ID = 13;
        public static final int PCM_CAN_ID = 21;

        /** Pneumatic Channels **/
        public static final int PICKUP_FORWARD_CHANNEL = 0, PICKUP_REVERSE_CHANNEL = 1;

        /** CAN BUS Properties **/
        public static final String GYRO_CAN_BUS = "canivore";

        /** Inversion Properties **/
        public static final boolean CLIMBER_INVERTED = false;
        public static final boolean ELEVATOR_INVERTED = true;
        public static final boolean FEEDER_INVERTED = true;
        public static final boolean PICKUP_INVERTED = false;

        /** Shooter Variables **/
        public static final double SHOOTER_TILT_ALLOWED_DEVIATION = 3; // Shooter Tilt Accuracy Settings
        public static final double SHOOTER_ALLOWED_PERCENT_ERROR = .05; // Shooter Veloctity Accuracy Settings
        public static final double SHOOTER_DESIRED_AT_SPEED_TIME = .75; // Shooter Veloctity Time Window Settings
        public static final double SHOOTER_REVERSE_FEED_SPEED = -.5;
        public static final HashMap<String, ShooterShot> SHOOTER_SHOTS = new HashMap<>() {
                {
                        put("SHORT_SHOT", new ShooterShot(22, 550, 800));
                        put("NORMAL_SHOT", new ShooterShot(37, 2500, 3000));
                        put("FAR_SHOT", new ShooterShot(59, 4000, 4000));
                        put("YEET_SHOT", new ShooterShot(45, 2900, 2900));
                }
        };

        /** Compressor Properties **/
        public static final double MIN_PRESSURE = 95, MAX_PRESSURE = 110;

        /** Sensor Channels **/
        public static int ELEVATOR_LINE_SENSOR_CHANNEL = 2;
        public static int ELEVATOR_ENCODER_CHANNEL_A = 0, ELEVATOR_ENCODER_CHANNEL_B = 1;

        /** Elevator Properties **/
        public static int ELEVATOR_LOWER_CUTOFF = 300, ELEVATOR_UPPER_CUTOFF = 11500;

        /** Photon Vision Constants **/
        public static final HashMap<String, PhotonCameraConfig> PHOTON_CAMERAS_CONFIGS = new HashMap<>() {
                {
                        put("FRONT_TARGETING_CAMERA", new PhotonCameraConfig("FRONT_TARGETING_CAMERA", Units.inchesToMeters(15), Units.degreesToRadians(22.5)));
                        put("REAR_TARGETING_CAMERA", new PhotonCameraConfig("REAR_TARGETING_CAMERA", Units.inchesToMeters(15), Units.degreesToRadians(22.5)));
                }
        };

        public static final HashMap<Integer, Alliance> APRILTAG_FIELD_COLOR = new HashMap<>() {
                {
                        // Blue Feed Station
                        put(1, Alliance.Blue);
                        put(2, Alliance.Blue);
                        // Red Speaker
                        put(3, Alliance.Red);
                        put(4, Alliance.Red); // More Centered
                        // Red Low Goal
                        put(5, Alliance.Red);
                        // Blue Low Goal
                        put(6, Alliance.Blue);
                        // Blue Speaker
                        put(7, Alliance.Blue); // More Centered
                        put(8, Alliance.Blue);
                        // Red Feed Station
                        put(9, Alliance.Red);
                        put(10, Alliance.Red);
                        // Red Center Goals
                        put(11, Alliance.Red);
                        put(12, Alliance.Red);
                        put(13, Alliance.Red);
                        // Blue Center Goals
                        put(14, Alliance.Blue);
                        put(15, Alliance.Blue);
                        put(16, Alliance.Blue);

                }
        };

        public static final AprilTagFieldLayout AprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

        /** PID Variables **/
        public static final double GYRO_KP = .01, GYRO_KI = .0001, GYRO_KD = .0, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = -.015, SLEW_KI = -0.002, SLEW_KD = .003, SLEW_PID_MIN = -.6, SLEW_PID_MAX = .6;
        public static final double SHOOTER_KP = .00025, SHOOTER_KI = .0004, SHOOTER_KD = -.002, SHOOTER_MIN = -1, SHOOTER_MAX = 1;
        public static final double LIMELIGHT_KP = -.0175, LIMELIGHT_KI = -.0022, LIMELIGHT_KD = -.0022, LIMELIGHT_MIN = -.5, LIMELIGHT_MAX = .5;
        public static final double ELEVATOR_KP = -.6, ELEVATOR_KI = 0, ELEVATOR_KD = 0, ELEVATOR_KF = 0, ELEVATOR_PID_MIN = -5, ELEVATOR_PID_MAX = .75;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16" };

}
