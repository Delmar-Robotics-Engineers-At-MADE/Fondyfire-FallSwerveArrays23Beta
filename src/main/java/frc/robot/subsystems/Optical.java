package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OpticalConstants;

public class Optical extends SubsystemBase{
    DigitalInput sensor;
    public Optical() {
        sensor = new DigitalInput(OpticalConstants.DIO);
    }

    public boolean getState() {
        return sensor.get();
    }
}
