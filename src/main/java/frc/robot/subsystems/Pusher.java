package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {
    
    private final CANSparkMax dart;
    private final PIDController pid;
    private final AnalogInput pot;
    private final RelativeEncoder encoder;
    private ShuffleboardTab comp;
    boolean homed = false;
    int x;

    public Pusher() {
        dart = new CANSparkMax(20, MotorType.kBrushless);
        dart.setIdleMode(IdleMode.kBrake);
        pid = new PIDController(0.02, 0, 0);
        pot = new AnalogInput(0);
        encoder = dart.getEncoder();

        comp = Shuffleboard.getTab("comp");
        comp.add("Pusher Homed", getState());
        comp.add("potentiometer", pot.getValue());
        
    }

    public void checkHome() {
        if (pot.getValue() == PusherConstants.kHomePos) {
            homed = true;
        }
        else {
            homed = false;
        }
    }

    public boolean getState() {
        return homed;
    }

    public void home() {
        while (!homed) {
            checkHome();
            if (!homed) {
                dart.set(pid.calculate(pot.getValue(), PusherConstants.kHomePos));
            }
            else {
                dart.stopMotor();
                encoder.setPosition(0.0);
            }
        }
    }

    // for testing and position finding only. Remove from container for real use.
    public void runClimberManual(boolean reversed) {
        if (pot.getValue() >= PusherConstants.kLimit){
            System.out.println("¡Extension Limit!");
        }
        else if (pot.getValue() == PusherConstants.kHomePos) {
            System.out.println("¡Retraction Limit!");
        }
        else {
            if (!reversed) {
                x = 1;
            }
            else {
                x = -1;
            }
            dart.set((0.5 * x));
        }
    }

    public void runClimberToPos(double pos) {
        dart.set(pid.calculate(pot.getValue(), pos));
    }

    public void stopPusher() {
        dart.set(0);
    }
}
