package frc.robot.shooter.elevator;

public enum ElevatorTarget{
    IDLE,
    CORALSTAT,
    L1,
    L2,
    L3,
    L4;


    public boolean IsReefTarget(){
        return this == L1 || this == L2 || this == L3 || this == L4;
    }
}