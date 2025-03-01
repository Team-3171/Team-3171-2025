package frc.team3171.auton;

// FRC Imports
import edu.wpi.first.wpilibj.XboxController;
import frc.team3171.models.XboxControllerState;

/**
 * @author Mark Ebert
 * @author Charles Fee
 * @author Elijah Hoda
 */
public class AutonRecorderData {

    // Variables to store
    private double fpgaTimestamp;
    private XboxControllerState driveControllerState, operatorControllerState;

    /**
     * 
     */
    public AutonRecorderData() {
        this.fpgaTimestamp = 0;
        this.driveControllerState = new XboxControllerState();
        this.operatorControllerState = new XboxControllerState();
    }

    /**
     * Constructor
     */
    public AutonRecorderData(final double fpgaTimestamp, final XboxController driveController, final XboxController operatorController) {
        this.fpgaTimestamp = fpgaTimestamp;
        this.driveControllerState = new XboxControllerState(driveController);
        this.operatorControllerState = new XboxControllerState(operatorController);
    }

    /**
     * Constructor
     */
    public AutonRecorderData(final double fpgaTimestamp, final XboxControllerState driveControllerState, final XboxControllerState operatorControllerState) {
        this.fpgaTimestamp = fpgaTimestamp;
        this.driveControllerState = driveControllerState;
        this.operatorControllerState = operatorControllerState;
    }

    /**
     * Returns the fpgaTimestamp variable stored in {@code AutonRecorderData}.
     * 
     * @return The current fpgaTimestamp variable value.
     */
    public double getFPGATimestamp() {
        return fpgaTimestamp;
    }

    /**
     * Returns the drive controller state stored in {@code AutonRecorderData}.
     * 
     * @return The saved state of the drive controller.
     */
    public XboxControllerState getDriveControllerState() {
        return driveControllerState;
    }

    /**
     * Returns the operator controller state stored in {@code AutonRecorderData}.
     * 
     * @return The saved state of the operator controller.
     */
    public XboxControllerState getOperatorControllerState() {
        return operatorControllerState;
    }

    public void setFPGATimestamp(double fpgaTimestamp) {
        this.fpgaTimestamp = fpgaTimestamp;
    }

    public void setDriveControllerState(XboxControllerState driveControllerState) {
        this.driveControllerState = driveControllerState;
    }

    public void setOperatorControllerState(XboxControllerState operatorControllerState) {
        this.operatorControllerState = operatorControllerState;
    }

    /**
     * Checks if the provided new data is different from the current data, as long
     * as the new data's timestamp is later then the current timestamp, if so
     * returns true, otherwise false.
     * 
     * @param newData
     *            The new set of data containing the new values to compare to
     *            the current values.
     * @return Returns true if the new data is different, otherwise false.
     */
    public boolean isChanged(final AutonRecorderData newData) {
        if (newData.getFPGATimestamp() >= fpgaTimestamp) {
            if (driveControllerState.isChanged(newData.getDriveControllerState())) {

            } else if (operatorControllerState.isChanged(newData.getOperatorControllerState())) {

            }
        }
        return false;
    }

    /**
     * Returns the string representation of the data. Formatted as
     * fpgaTimestamp,leftX,leftY,rightX,rightY;\n
     */
    @Override
    public String toString() {
        return String.format("%.3f;%s;%s;\n", fpgaTimestamp, driveControllerState.toString(), operatorControllerState.toString());
    }

    /**
     * Takes the data from a string and seperates it into the seperate data values.
     * 
     * @param dataString
     * @return
     */
    public static AutonRecorderData fromString(String dataString) {
        try {
            dataString = dataString.trim();
            if (dataString.endsWith(";")) {
                dataString = dataString.substring(0, dataString.length() - 1);
                final String[] data = dataString.split(";");
                if (data.length == 3) {
                    final AutonRecorderData autonData = new AutonRecorderData();
                    autonData.setFPGATimestamp(Double.parseDouble(data[0]));
                    autonData.setDriveControllerState(XboxControllerState.fromString(data[1]));
                    autonData.setOperatorControllerState(XboxControllerState.fromString(data[2]));
                    return autonData;
                }
            }
            return null;
        } catch (Exception e) {
            System.err.println(e.getMessage());
            return null;
        }
    }

}