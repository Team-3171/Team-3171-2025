package frc.team3171.models;

// FRC Imports
import edu.wpi.first.wpilibj.XboxController;

/**
 * @author Mark Ebert
 */
public class XboxControllerState {

    // Xbox Variables
    private double leftX, leftY, rightX, rightY, leftTriggerAxis, rightTriggerAxis, pov;
    private boolean aButton, bButton, xButton, yButton;
    private boolean startButton, backButton, leftBumper, rightBumper;

    public XboxControllerState() {
        // Doubles
        this.leftX = 0;
        this.leftY = 0;
        this.rightX = 0;
        this.rightY = 0;
        this.leftTriggerAxis = 0;
        this.rightTriggerAxis = 0;
        this.pov = -1;

        // Booleans
        this.aButton = false;
        this.bButton = false;
        this.xButton = false;
        this.yButton = false;
        this.startButton = false;
        this.backButton = false;
        this.leftBumper = false;
        this.rightBumper = false;
    }

    /**
     * 
     * @param xboxController
     */
    public XboxControllerState(final XboxController xboxController) {
        // Doubles
        this.leftX = xboxController.getLeftX();
        this.leftY = xboxController.getLeftY();
        this.rightX = xboxController.getRightX();
        this.rightY = xboxController.getRightY();
        this.leftTriggerAxis = xboxController.getLeftTriggerAxis();
        this.rightTriggerAxis = xboxController.getRightTriggerAxis();
        this.pov = xboxController.getPOV();

        // Booleans
        this.aButton = xboxController.getAButton();
        this.bButton = xboxController.getBButton();
        this.xButton = xboxController.getXButton();
        this.yButton = xboxController.getYButton();
        this.startButton = xboxController.getStartButton();
        this.backButton = xboxController.getBackButton();
        this.leftBumper = xboxController.getLeftBumper();
        this.rightBumper = xboxController.getRightBumper();
    }

    public double getLeftX() {
        return leftX;
    }

    public double getLeftY() {
        return leftY;
    }

    public double getRightX() {
        return rightX;
    }

    public double getRightY() {
        return rightY;
    }

    public double getLeftTriggerAxis() {
        return leftTriggerAxis;
    }

    public double getRightTriggerAxis() {
        return rightTriggerAxis;
    }

    public double getPOV() {
        return pov;
    }

    public boolean getAButton() {
        return aButton;
    }

    public boolean getBButton() {
        return bButton;
    }

    public boolean getXButton() {
        return xButton;
    }

    public boolean getYButton() {
        return yButton;
    }

    public boolean getStartButton() {
        return startButton;
    }

    public boolean getBackButton() {
        return backButton;
    }

    public boolean getLeftBumper() {
        return leftBumper;
    }

    public boolean getRightBumper() {
        return rightBumper;
    }

    public void setLeftX(double leftX) {
        this.leftX = leftX;
    }

    public void setLeftY(double leftY) {
        this.leftY = leftY;
    }

    public void setRightX(double rightX) {
        this.rightX = rightX;
    }

    public void setRightY(double rightY) {
        this.rightY = rightY;
    }

    public void setLeftTriggerAxis(double leftTrigger) {
        this.leftTriggerAxis = leftTrigger;
    }

    public void setRightTriggerAxis(double rightTrigger) {
        this.rightTriggerAxis = rightTrigger;
    }

    public void setPOV(double pov) {
        this.pov = pov;
    }

    public void setAButton(boolean aButton) {
        this.aButton = aButton;
    }

    public void setBButton(boolean bButton) {
        this.bButton = bButton;
    }

    public void setXButton(boolean xButton) {
        this.xButton = xButton;
    }

    public void setYButton(boolean yButton) {
        this.yButton = yButton;
    }

    public void setStartButton(boolean startButton) {
        this.startButton = startButton;
    }

    public void setBackButton(boolean backButton) {
        this.backButton = backButton;
    }

    public void setLeftBumper(boolean leftBumper) {
        this.leftBumper = leftBumper;
    }

    public void setRightBumper(boolean rightBumper) {
        this.rightBumper = rightBumper;
    }

    public boolean isChanged(XboxControllerState newState) {
        if (leftX != newState.getLeftX()) {
            return true;
        } else if (leftY != newState.getLeftY()) {
            return true;
        } else if (rightX != newState.getRightX()) {
            return true;
        } else if (rightY != newState.getRightY()) {
            return true;
        } else if (leftTriggerAxis != newState.getLeftTriggerAxis()) {
            return true;
        } else if (rightTriggerAxis != newState.getRightTriggerAxis()) {
            return true;
        } else if (pov != newState.getPOV()) {
            return true;
        } else if (aButton != newState.getAButton()) {
            return true;
        } else if (bButton != newState.getBButton()) {
            return true;
        } else if (xButton != newState.getXButton()) {
            return true;
        } else if (yButton != newState.getYButton()) {
            return true;
        } else if (startButton != newState.getStartButton()) {
            return true;
        } else if (backButton != newState.getBackButton()) {
            return true;
        } else if (leftBumper != newState.getLeftBumper()) {
            return true;
        } else if (rightBumper != newState.getRightBumper()) {
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%b,%b,%b,%b,%b,%b,%b,%b", leftX, leftY, rightX, rightY, leftTriggerAxis, rightTriggerAxis,
                pov, aButton, bButton, xButton, yButton, startButton, backButton, leftBumper, rightBumper);
    }

    /**
     * Takes the data from a string and seperates it into the seperate data values.
     * 
     * @param dataString
     * @return
     */
    public static XboxControllerState fromString(String dataString) {
        try {
            dataString = dataString.trim();
            final String[] data = dataString.split(",");
            if (data.length == 15) {
                // New empty controller state
                final XboxControllerState controllerState = new XboxControllerState();

                // Parse the doubles
                controllerState.setLeftX(Double.parseDouble(data[0]));
                controllerState.setLeftY(Double.parseDouble(data[1]));
                controllerState.setRightX(Double.parseDouble(data[2]));
                controllerState.setRightY(Double.parseDouble(data[3]));
                controllerState.setLeftTriggerAxis(Double.parseDouble(data[4]));
                controllerState.setRightTriggerAxis(Double.parseDouble(data[5]));
                controllerState.setPOV(Double.parseDouble(data[6]));

                // Parse the booleans
                controllerState.setAButton(Boolean.parseBoolean(data[7]));
                controllerState.setBButton(Boolean.parseBoolean(data[8]));
                controllerState.setXButton(Boolean.parseBoolean(data[9]));
                controllerState.setYButton(Boolean.parseBoolean(data[10]));
                controllerState.setStartButton(Boolean.parseBoolean(data[11]));
                controllerState.setBackButton(Boolean.parseBoolean(data[12]));
                controllerState.setLeftBumper(Boolean.parseBoolean(data[13]));
                controllerState.setRightBumper(Boolean.parseBoolean(data[14]));

                // Return the parsed controller state
                return controllerState;
            }
            return null;
        } catch (Exception e) {
            System.err.println(e.getMessage());
            return null;
        }
    }

}
