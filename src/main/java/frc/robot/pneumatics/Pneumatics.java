package frc.robot.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;

public class Pneumatics {
    private PneumaticHub _pcm;
    private Compressor _compressor;

    public Pneumatics() {
        if (Robot.isReal()) {
            _pcm = new PneumaticHub(PneumaticsMap.PCM_CAN_ID);
            _compressor = new Compressor(PneumaticsMap.PCM_CAN_ID, PneumaticsModuleType.REVPH);

            // TODO: Check these values
            _compressor.enableAnalog(130, 130);
        }
    }

    public PneumaticHub getPneumaticsControlModule() {
        return _pcm;
    }

    public Integer getPneumaticsControlModuleId() {
        return PneumaticsMap.PCM_CAN_ID;
    }

    public PneumaticsModuleType getPneumaticsControlModuleType() {
        return PneumaticsModuleType.REVPH;
    }

    public Compressor getCompressor() {
        return _compressor;
    }

    public double getSystemPressure() {
        return _pcm.getPressure(PneumaticsMap.PRESSURE_SENSOR_CHANNEL);
    }
}
