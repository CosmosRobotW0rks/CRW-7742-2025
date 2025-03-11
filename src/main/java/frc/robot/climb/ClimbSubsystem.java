package frc.robot.climb;
import com.pathplanner.lib.events.CancelCommandEvent;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;


public class ClimbSubsystem extends SubsystemBase  {

    Compressor compressor;
    Solenoid solenoid;

    boolean climbState = false;

    
    public ClimbSubsystem()
    {
        compressor = new Compressor(ClimbConstants.PCM_CANID, PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(ClimbConstants.PCM_CANID, PneumaticsModuleType.CTREPCM, ClimbConstants.SolenoidChannel);
        
        compressor.enableDigital();
        SetClimb(true);
    }

    public boolean GetClimb()
    {
        return climbState;
    }


    public void SetClimb(boolean state)
    {
        solenoid.set(!climbState);
        climbState = state;
    }
    
    public void ToggleClimb()
    {
        SetClimb(!GetClimb());
    }

    public Command SetClimbCommand(boolean state)
    {
        return Commands.runOnce(() -> SetClimb(state), this);
    }

    public Command ToggleClimbCommand()
    {
        return Commands.runOnce(() -> ToggleClimb(), this);
    }



}
