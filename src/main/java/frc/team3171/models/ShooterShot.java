package frc.team3171.models;

/**
 * @author Mark Ebert
 */
public class ShooterShot {

    public final int shooterAngle, lowerShooterRPM, upperShooterRPM;

    public ShooterShot(final int shooterAngle, final int lowerShooterRPM, final int upperShooterRPM) {
        this.shooterAngle = shooterAngle;
        this.lowerShooterRPM = lowerShooterRPM;
        this.upperShooterRPM = upperShooterRPM;
    }

    public int getShooterAngle() {
        return shooterAngle;
    }

    public int getLowerShooterRPM() {
        return lowerShooterRPM;
    }

    public int getUpperShooterRPM() {
        return upperShooterRPM;
    }

}
