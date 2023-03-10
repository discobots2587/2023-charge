package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Arm {

    CANSparkMax jointMotor;
    VictorSPX intakeMotor;
    
    double angle = Constants.Arm.maxAngle;

    public Arm() {
        jointMotor = new CANSparkMax(Constants.Arm.jointMotorID, MotorType.kBrushless);
        intakeMotor = new VictorSPX(Constants.Arm.intakeMotorID);

        jointMotor.setSmartCurrentLimit(Constants.Arm.jointMotorCurrentLimit);
    }
    
}