package frc.robot.climb;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;


public class ClimbSubsystem extends SubsystemBase  {

    Compressor compressor;
    Solenoid solenoid;

    
    public ClimbSubsystem()
    {
        compressor = new Compressor(ClimbConstants.PCM_CANID, PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(ClimbConstants.PCM_CANID, PneumaticsModuleType.CTREPCM, ClimbConstants.SolenoidChannel);    
    }

    @Override
    public void periodic() {
        if(!compressor.getPressureSwitchValue()) compressor.enableDigital();
        else compressor.disable();
    }


    public void SetClimbValve(boolean state)
    {
        solenoid.set(state);
    }

}
