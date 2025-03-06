package frc.team3171.sensors;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Encoder;

public class RevEncoderRelative extends Encoder implements DoubleSupplier {

    public RevEncoderRelative(int channelA, int channelB) {
        super(channelA, channelB);
    }

    @Override
    public double getAsDouble() {
        return get();
    }

}
