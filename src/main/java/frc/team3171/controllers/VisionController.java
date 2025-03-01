package frc.team3171.controllers;

// Java Imports
import java.util.HashMap;

// FRC Imports
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Photon Vision Imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.models.PhotonAprilTagTarget;
import frc.team3171.models.PhotonCameraConfig;

/**
 * @author Mark Ebert
 */
public class VisionController implements RobotProperties {

    // Photon Vision Objects
    private final HashMap<String, PhotonCamera> PHOTON_CAMERAS;

    public VisionController() {
        PHOTON_CAMERAS = new HashMap<>();

        // Photon Vision init
        PHOTON_CAMERAS_CONFIGS.forEach((photonCameraName, photonCameraConfig) -> {
            PHOTON_CAMERAS.put(photonCameraName, new PhotonCamera(photonCameraConfig.getCAMERA_NAME()));
        });
    }

    public PhotonTrackedTarget getCameraBestTarget(final String photonCameraName) {
        final PhotonCameraConfig photonCameraConfig = PHOTON_CAMERAS_CONFIGS.get(photonCameraName);
        final PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);

        // Check that the provided camera name returned an actual result
        if (photonCameraConfig == null || photonCamera == null) {
            System.err.printf("The provided camera name [%s] did not return a valid result!\nPlease verify that the camera was initialized correctly.\n",
                    photonCameraName);
            return null;
        }

        if (photonCamera.isConnected()) {
            PhotonPipelineResult result = photonCamera.getLatestResult();
            if (result.hasTargets()) {
                return result.getBestTarget();
            }
        }
        return null;
    }

    public HashMap<Integer, PhotonAprilTagTarget> getAllVisibleAprilTags() {
        final HashMap<Integer, PhotonAprilTagTarget> targetData = new HashMap<Integer, PhotonAprilTagTarget>();
        // Query all cameras for aprils tags
        PHOTON_CAMERAS.forEach((photonCameraName, photonCamera) -> {
            if (photonCamera.isConnected()) {
                PhotonPipelineResult result = photonCamera.getLatestResult();
                if (result.hasTargets()) {
                    result.targets.forEach((target) -> {
                        PhotonAprilTagTarget existingTarget = targetData.get(target.getFiducialId());
                        if (existingTarget != null) {
                            if (target.getArea() > existingTarget.getPHOTON_TRACKED_TARGET().getArea()) {
                                targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                            }
                        } else {
                            targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                        }
                    });
                }
            }
        });
        return targetData;
    }

    public HashMap<Integer, PhotonAprilTagTarget> getAllVisibleAprilTags(final String... photonCameraNames) {
        final HashMap<Integer, PhotonAprilTagTarget> targetData = new HashMap<Integer, PhotonAprilTagTarget>();
        // Query all cameras for aprils tags
        for (String photonCameraName : photonCameraNames) {
            PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);
            if (photonCamera != null) {
                if (photonCamera.isConnected()) {
                    PhotonPipelineResult result = photonCamera.getLatestResult();
                    if (result.hasTargets()) {
                        result.targets.forEach((target) -> {
                            PhotonAprilTagTarget existingTarget = targetData.get(target.getFiducialId());
                            if (existingTarget != null) {
                                if (target.getArea() > existingTarget.getPHOTON_TRACKED_TARGET().getArea()) {
                                    targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                                }
                            } else {
                                targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                            }
                        });
                    }
                }
            }
        }
        return targetData;
    }

    public PhotonAprilTagTarget getAllVisibleAprilTagsByPriority(final int[] preferedFiducialIdOrder, final String... photonCameraNames) {
        final HashMap<Integer, PhotonAprilTagTarget> targetData = new HashMap<Integer, PhotonAprilTagTarget>();
        // Query all cameras for aprils tags
        for (String photonCameraName : photonCameraNames) {
            PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);
            if (photonCamera != null) {
                if (photonCamera.isConnected()) {
                    PhotonPipelineResult result = photonCamera.getLatestResult();
                    if (result.hasTargets()) {
                        result.targets.forEach((target) -> {
                            PhotonAprilTagTarget existingTarget = targetData.get(target.getFiducialId());
                            if (existingTarget != null) {
                                if (target.getArea() > existingTarget.getPHOTON_TRACKED_TARGET().getArea()) {
                                    targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                                }
                            } else {
                                targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                            }
                        });
                    }
                }
            }
        }

        PhotonAprilTagTarget preferedTarget = null;
        for (int fiducialId : preferedFiducialIdOrder) {
            PhotonAprilTagTarget currentTarget = targetData.get(fiducialId);
            if (preferedTarget == null && currentTarget != null) {
                preferedTarget = currentTarget;
            } else if (currentTarget != null) {
                preferedTarget = preferedTarget.getPHOTON_TRACKED_TARGET().getArea() < currentTarget.getPHOTON_TRACKED_TARGET().getArea() ? currentTarget
                        : preferedTarget;
            }
        }

        return preferedTarget;
    }

    public void shuffleboardTabInit(final String photonCameraName, final String tabName) {
        final PhotonCameraConfig photonCameraConfig = PHOTON_CAMERAS_CONFIGS.get(photonCameraName);
        final PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);

        // Check that the provided camera name returned an actual result
        if (photonCameraConfig == null || photonCamera == null) {
            System.err.printf("The provided camera name [%s] did not return a valid result!\nPlease verify that the camera was initialized correctly.\n",
                    photonCameraName);
            return;
        }

        ShuffleboardTab selectedTab = Shuffleboard.getTab(tabName);
        selectedTab.addBoolean(String.format("%s:", photonCameraName), () -> photonCamera.isConnected());
        APRILTAG_FIELD_COLOR.forEach((targetID, alliance) -> {
            selectedTab.addString(String.format("ID: %d", targetID), () -> {
                // Photon Vision Read Outs
                double range = -1, yaw = 0, pitch = 0;
                if (photonCamera.isConnected()) {
                    PhotonPipelineResult result = photonCamera.getLatestResult();
                    if (result.hasTargets()) {
                        for (PhotonTrackedTarget target : result.getTargets()) {
                            if (target.getFiducialId() == targetID) {
                                final Pose3d aprilTagPose = AprilTagLayout.getTagPose(targetID).get();
                                final double targetHeightMeters = aprilTagPose.getZ();
                                range = PhotonUtils.calculateDistanceToTargetMeters(
                                        photonCameraConfig.getCAMERA_HEIGHT_METERS(),
                                        targetHeightMeters,
                                        photonCameraConfig.getCAMERA_PITCH_RADIANS(),
                                        Units.degreesToRadians(target.getPitch()));
                                yaw = target.getYaw();
                            }
                        }
                    }
                }
                return String.format("R: %.2f | Y: %.2f | P: %.2f", range, yaw, pitch);
            });
        });
    }

    public void smartdashboard(final String photonCameraName) {
        final PhotonCameraConfig photonCameraConfig = PHOTON_CAMERAS_CONFIGS.get(photonCameraName);
        final PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);

        // Check that the provided camera name returned an actual result
        if (photonCameraConfig == null || photonCamera == null) {
            System.err.printf("The provided camera name [%s] did not return a valid result!\nPlease verify that the camera was initialized correctly.\n",
                    photonCameraName);
            return;
        }

        // Photon Vision Read Outs
        int targetID = 0;
        double range = -1, yaw = 0, pitch = 0;
        if (photonCamera.isConnected()) {
            PhotonPipelineResult result = photonCamera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                targetID = bestTarget.getFiducialId();
                final AprilTag aprilTag = AprilTagLayout.getTags().get(targetID);
                final double targetHeightMeters = aprilTag.pose.getZ();
                range = PhotonUtils.calculateDistanceToTargetMeters(
                        photonCameraConfig.getCAMERA_HEIGHT_METERS(),
                        targetHeightMeters,
                        photonCameraConfig.getCAMERA_PITCH_RADIANS(),
                        Units.degreesToRadians(bestTarget.getPitch()));
                yaw = bestTarget.getYaw();
            }
        }
        SmartDashboard.putBoolean(String.format("%s:", photonCameraName), photonCamera.isConnected());
        SmartDashboard.putString(String.format("%s Best Target:", photonCameraName),
                String.format("ID: %d | R: %.2f | Y: %.2f | P: %.2f", targetID, range, yaw, pitch));
    }

}
