package frc.robot.shooter.elevator;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;

public class ElevatorTargetSwitchSubsystem extends SubsystemBase {

    private int target = 0;

    private double lastInteraction = -5;

    public ElevatorTargetSwitchSubsystem() {

        UpdateDashboard(true);
    }


    private boolean Timeout()
    {
        return Timer.getFPGATimestamp() - lastInteraction > 5;
    }


    public void TerminateAlignment()
    {
        target = 0;
        UpdateDashboard(true);
    }

    public void IncreaseTarget()
    {
        target++;

        if(target == 4) target = 0;
        
        lastInteraction = Timer.getFPGATimestamp();
        UpdateDashboard();
    }

    public void DecreaseTarget()
    {
        target--;

        if(target == -1) target = 3;


        lastInteraction = Timer.getFPGATimestamp();
        UpdateDashboard();
    }

    public ElevatorTarget GetTarget(boolean reset)
    {
        if(Timeout()) return null;

        ElevatorTarget t = ElevatorTarget.IDLE;

        switch(target)
        {
            case 0:
            t = ElevatorTarget.L1;
            break;

            case 1:
            t = ElevatorTarget.L2;
            break;

            case 2:
            t = ElevatorTarget.L3;
            break;

            case 3:
            t = ElevatorTarget.L4;
            break;
            
        }
        

        if(reset)
        TerminateAlignment();

        return t;
    }

    private void UpdateDashboard(boolean clear)
    {
        ElevatorTarget target = GetTarget(false);
        SmartDashboard.putString("REEF TARGET", clear || target == null ? "" : target.toString());
    }

    private void UpdateDashboard()
    {
        UpdateDashboard(false);
    }

    @Override
    public void periodic() {

        if(Timeout())
        TerminateAlignment();
        
    }
}
