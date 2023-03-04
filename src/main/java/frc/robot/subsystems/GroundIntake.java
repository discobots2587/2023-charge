package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class GroundIntake {

    CANSparkMax jointMotor;
    CANSparkMax rollerMotor;

    public GroundIntake(int jointMotorID, int rollerMotorID) {
        jointMotor = new CANSparkMax(jointMotorID, MotorType.kBrushless);
        rollerMotor = new CANSparkMax(rollerMotorID, MotorType.kBrushless);

        jointMotor.setSmartCurrentLimit(Constants.GroundIntake.gndJointMotorCurrentLimit);
        rollerMotor.setSmartCurrentLimit(Constants.GroundIntake.gndRollerMotorCurrentLimit);
    }

    public CANSparkMax getJointMotor() {
        return jointMotor;
    }

    public CANSparkMax getRollerMotor() {
        return rollerMotor;
    }
}
