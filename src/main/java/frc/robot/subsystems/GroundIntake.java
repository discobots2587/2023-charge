package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class GroundIntake {

    CANSparkMax jointMotor;
    VictorSPX rollerMotor;

    public GroundIntake(int jointMotorID, int rollerMotorID) {
        jointMotor = new CANSparkMax(jointMotorID, MotorType.kBrushless);
        rollerMotor = new VictorSPX(rollerMotorID);

        jointMotor.setSmartCurrentLimit(Constants.GroundIntake.gndJointMotorCurrentLimit);
    }

    public CANSparkMax getJointMotor() {
        return jointMotor;
    }

    public VictorSPX getRollerMotor() {
        return rollerMotor;
    }
}
