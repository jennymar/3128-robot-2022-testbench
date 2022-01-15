package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.NAR_EMotor;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;

public class TestBenchSubsystem extends SubsystemBase{
    private NAR_EMotor motor = new NAR_TalonSRX(11);

    public TestBenchSubsystem(){}  

    public void drive(){
        //motor.set(.5);
    }

    public void stop(){
        motor.set(0);
    }
}
