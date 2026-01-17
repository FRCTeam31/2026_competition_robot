package frc.robot.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pneumatics {
    private PneumaticsControlModule _pcm;
    private Compressor _compressor;

    public Pneumatics(boolean isReal) {
        if (isReal) {
            _pcm = new PneumaticsControlModule(PneumaticsMap.PCMCanID);
            _compressor = new Compressor(PneumaticsMap.PCMCanID, PneumaticsMap.PCMType);

            _compressor.enableDigital();
        }
    }

    public PneumaticsControlModule getPneumaticsControlModule() {
        return _pcm;
    }

    public Integer getPneumaticsControlModuleId() {
        return PneumaticsMap.PCMCanID;
    }

    public PneumaticsModuleType getPneumaticsControlModuleType() {
        return PneumaticsMap.PCMType;
    }

    public Compressor getCompressor() {
        return _compressor;
    }
}
