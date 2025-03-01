package frc.team3171.models;

import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * @author Mark Ebert
 */
public class PhotonAprilTagTarget {

    private String PHOTON_CAMERA_NAME;
    private PhotonTrackedTarget PHOTON_TRACKED_TARGET;

    public PhotonAprilTagTarget(final String PHOTON_CAMERA_NAME, final PhotonTrackedTarget PHOTON_TRACKED_TARGET) {
        this.PHOTON_CAMERA_NAME = PHOTON_CAMERA_NAME;
        this.PHOTON_TRACKED_TARGET = PHOTON_TRACKED_TARGET;
    }

    public String getPHOTON_CAMERA_NAME() {
        return PHOTON_CAMERA_NAME;
    }

    public PhotonTrackedTarget getPHOTON_TRACKED_TARGET() {
        return PHOTON_TRACKED_TARGET;
    }

}
