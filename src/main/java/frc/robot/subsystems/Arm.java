package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
public class Arm {

    CANSparkMax jointMotor;
    VictorSPX intakeMotor;

    public Arm(int jointMotorID, int intakeMotorID) {
        
    }
}
