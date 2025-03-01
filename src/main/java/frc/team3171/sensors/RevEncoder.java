package frc.team3171.sensors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class RevEncoder extends DutyCycleEncoder implements DoubleSupplier {

    public RevEncoder(int channel) {
        super(channel);
    }

    @Override
    public double getAsDouble() {
        return get();
    }

}
