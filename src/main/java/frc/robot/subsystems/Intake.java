package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final WPI_TalonFX motor;

    public Intake() {
        motor = new WPI_TalonFX(IntakeConstants.ID);
        motor.setNeutralMode(NeutralMode.Brake);
        
    }
    
    public void run(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }
}


