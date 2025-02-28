package frc.robot.led.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.led.LEDSubsystem;

public class JoystickTestCommand extends Command {
    CommandXboxController joy;
    LEDSubsystem led;

    public JoystickTestCommand(LEDSubsystem led, CommandXboxController joy)
    {
        this.led = led;
        this.joy = joy;


        System.out.println("CONSTRUCTORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR");


        addRequirements(led);
    }

    @Override
    public void initialize() {
        System.out.println("COMMAND INTINTITNI");
    }

    @Override
    public void execute() {

        System.out.println("CMD EXEC");
        led.Set(joy.getLeftY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("COMMAND ENDDNENDENDE");
    }
}
