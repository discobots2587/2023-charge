package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class GroundIntake {
    CANSparkMax groundJointMotor;
    VictorSPX rollerMotor;

    public GroundIntake() {
        groundJointMotor = new CANSparkMax(Constants.GroundIntake.jointMotorID, MotorType.kBrushless);
        rollerMotor = new VictorSPX(Constants.GroundIntake.rollerMotorID);

        initializeGroundIntake();
    }

    public void spinRoller(int pov) {
        if (pov == 180) {
            //intake
            rollerMotor.set(ControlMode.PercentOutput,1.0);
        }
        else if (pov == 0) {
            //spin out
            rollerMotor.set(ControlMode.PercentOutput, -1.0);
        }
        else {
            rollerMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }
    public void spinGroundIntakeJoint(double trigger, boolean bumper) {
        if (trigger > Constants.Controller.triggerDeadband) {
            //joint down
            groundJointMotor.setVoltage(Constants.GroundIntake.groundMaxVolt);
        }
        else if (bumper) {
            //joint up
            groundJointMotor.setVoltage(-Constants.GroundIntake.groundMaxVolt);
        }
        else {
            groundJointMotor.setVoltage(0);
        }
    }

    public void initializeGroundIntake() {
        groundJointMotor.restoreFactoryDefaults();
        groundJointMotor.setSmartCurrentLimit(Constants.GroundIntake.jointMotorCurrentLimit);
        rollerMotor.configFactoryDefault();
        
    }
}
