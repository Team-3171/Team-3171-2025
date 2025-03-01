package frc.team3171.models;

/**
 * @author Mark Ebert
 */
public class PhotonCameraConfig {

    private final String CAMERA_NAME;
    private final double CAMERA_HEIGHT_METERS, CAMERA_PITCH_RADIANS;

    public PhotonCameraConfig(final String CAMERA_NAME, final double CAMERA_HEIGHT_METERS, final double CAMERA_PITCH_RADIANS) {
        this.CAMERA_NAME = CAMERA_NAME;
        this.CAMERA_HEIGHT_METERS = CAMERA_HEIGHT_METERS;
        this.CAMERA_PITCH_RADIANS = CAMERA_PITCH_RADIANS;
    }

    /**
     * @return the CAMERA_NAME
     */
    public String getCAMERA_NAME() {
        return CAMERA_NAME;
    }

    /**
     * @return the CAMERA_HEIGHT_METERS
     */
    public double getCAMERA_HEIGHT_METERS() {
        return CAMERA_HEIGHT_METERS;
    }

    /**
     * @return the CAMERA_PITCH_RADIANS
     */
    public double getCAMERA_PITCH_RADIANS() {
        return CAMERA_PITCH_RADIANS;
    }

}
