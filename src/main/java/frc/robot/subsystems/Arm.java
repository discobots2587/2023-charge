package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Arm {
    CANSparkMax armJointMotor;
    VictorSPX armIntakeMotor;

    public Arm() {
        armJointMotor = new CANSparkMax(Constants.Arm.jointMotorID, MotorType.kBrushless);
        armIntakeMotor = new VictorSPX(Constants.Arm.intakeMotorID);
        
        initializeArm();
    }

    public void spinArmIntake(boolean out, boolean in) {
        if (in) {
            //intake
            armIntakeMotor.set(ControlMode.PercentOutput,1.0);
        }
        else if (out) {
            //spin out
            armIntakeMotor.set(ControlMode.PercentOutput, -1.0);
        }
        else {
            armIntakeMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void spinArmJoint(double trigger, boolean bumper) {
        if (trigger > Constants.Controller.triggerDeadband) {
            //arm down
            armJointMotor.setVoltage(-Constants.Arm.armMaxVolt);
        }
        else if (bumper) {
            //arm up
            armJointMotor.setVoltage(Constants.Arm.armMaxVolt);
        }
        else {
            armJointMotor.setVoltage(0);
        }
    }

    //setting motor/sensor configurations
    public void initializeArm() {
        armJointMotor.restoreFactoryDefaults();
        armJointMotor.setSmartCurrentLimit(Constants.Arm.jointMotorCurrentLimit);
        armIntakeMotor.configFactoryDefault();
        //is there a method for setting victorSPX current limit? // no
    }
}
