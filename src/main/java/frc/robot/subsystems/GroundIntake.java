package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;

public class GroundIntake {

    TalonSRX jointMotor;
    VictorSPX rollerMotor;

    public GroundIntake(int jointMotorID, int rollerMotorID) {
        jointMotor = new TalonSRX(jointMotorID);
        rollerMotor = new VictorSPX(rollerMotorID);

        jointMotor.configSupplyCurrentLimit(Constants.GroundIntake.gndJointMotorCurrentLimit);
        //rollerMotor.configAllSettings(Constants.GroundIntake.gndRollerMotorCurrentLimit);
    }

    public TalonSRX getJointMotor() {
        return jointMotor;
    }

    public VictorSPX getRollerMotor() {
        return rollerMotor;
    }
}
