package frc.robot.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;

public class Pneumatics {
    private PneumaticsControlModule _pcm;
    private Compressor _compressor;

    public Pneumatics() {
        if (Robot.isReal()) {
            _pcm = new PneumaticsControlModule(PneumaticsMap.PCM_CAN_ID);
            _compressor = new Compressor(PneumaticsMap.PCM_CAN_ID, PneumaticsMap.PCM_TYPE);

            _compressor.enableDigital();
        }
    }

    public PneumaticsControlModule getPneumaticsControlModule() {
        return _pcm;
    }

    public Integer getPneumaticsControlModuleId() {
        return PneumaticsMap.PCM_CAN_ID;
    }

    public PneumaticsModuleType getPneumaticsControlModuleType() {
        return PneumaticsMap.PCM_TYPE;
    }

    public Compressor getCompressor() {
        return _compressor;
    }
}
