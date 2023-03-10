package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class GroundIntake {

    CANSparkMax jointMotor;
    VictorSPX rollerMotor;

    /*
     * Is true if the intake is up and false if down
     */
    boolean up; 
    
    public GroundIntake(int jointMotorID, int rollerMotorID) {
        jointMotor = new CANSparkMax(jointMotorID, MotorType.kBrushless);
        rollerMotor = new VictorSPX(rollerMotorID);
    }
    
    /*
     *
     */
    public GroundIntake(boolean up) 
    {

        jointMotor = new CANSparkMax(Constants.GroundIntake.gndJointMotorID, MotorType.kBrushless);
        rollerMotor = new VictorSPX(Constants.GroundIntake.gndRollerMotorID);
        this.up = up;
        jointMotor.setSmartCurrentLimit(Constants.GroundIntake.gndJointMotorCurrentLimit);
        //rollerMotor.configAllSettings(Constants.GroundIntake.gndRollerMotorCurrentLimit); //Victors cannot have a current limit
    }

    /** 
     * @return whether the intake is in the down position 
     * Method is currently not doing anything
    */
    public boolean isDown(){
        
        return !up;
    }

    public void switchPosition() {
        if(isDown()) {
            moveUp();
        } else {
            moveDown();
        }
    }

    protected void moveUp() {

    }

    protected void moveDown() {

    }

    public CANSparkMax getJointMotor() {
        return jointMotor;
    }

    public VictorSPX getRollerMotor() {
        return rollerMotor;
    }
}
