package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm {

    CANSparkMax jointMotor;
    VictorSPX intakeMotor;

    public Arm(int jointMotorID, int intakeMotorID) {
        jointMotor = new CANSparkMax(jointMotorID, MotorType.kBrushless);
        intakeMotor = new VictorSPX(intakeMotorID);
    }
}