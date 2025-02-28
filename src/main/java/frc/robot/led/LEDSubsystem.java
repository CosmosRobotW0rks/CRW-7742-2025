package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{

    AddressableLED led = new AddressableLED(9);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(24);

    public LEDSubsystem()
    {
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }



    @Override
    public void periodic() {
        
    }


    public void Set(double val)
    {
        val = val < 0 ? 0 : val;
        val = val > 1 ? 1 : val;

        int c = (int)(val * 24.0);

        for(int i = 0; i<24; i++)
        {
            if(i < c) buffer.setRGB(i, 255, 255, 255);
            else buffer.setRGB(i, 255,0,0);
        }

        led.setData(buffer);
    }
}
