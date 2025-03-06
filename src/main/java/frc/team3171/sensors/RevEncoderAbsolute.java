package frc.team3171.sensors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class RevEncoderAbsolute extends DutyCycleEncoder implements DoubleSupplier {

    public RevEncoderAbsolute(int channel) {
        super(channel);
    }

    @Override
    public double getAsDouble() {
        return get();
    }

}
