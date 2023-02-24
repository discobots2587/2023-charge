package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mod0;
import frc.robot.Constants.Mod1;
import frc.robot.Constants.Mod2;
import frc.robot.Constants.Mod3;
import frc.robot.Constants.Swerve;

public class SwerveDrive extends SubsystemBase{
    public SwerveModule[] swerveMods;

    double[] driveSpeed = new double[4];
    double reverseDriveMotor = 1.0, xAxis2;

    public double targetHeading = 0, opTargetHeading = 180;

    public SwerveDrive() {
        swerveMods = new SwerveModule[] {
            new SwerveModule(0,Mod0.angleMotorID,Mod0.driveMotorID, Mod0.threncID),
            new SwerveModule(1,Mod1.angleMotorID,Mod1.driveMotorID,Mod1.threncID),
            new SwerveModule(1,Mod2.angleMotorID,Mod2.driveMotorID, Mod2.threncID),
            new SwerveModule(1,Mod3.angleMotorID,Mod3.driveMotorID, Mod3.threncID),
        };
    }

    public void drive(double xAxis, double yAxis, double xAxis2) {
        findTargetHeading(xAxis, yAxis);
        this.xAxis2 = xAxis2;
        if (Math.abs(this.xAxis2) < Swerve.deadband) {
            this.xAxis2 = 0;
        }

        for (SwerveModule sm : swerveMods) {
            sm.findAngle();
            sm.errorAngle = sm.findError(targetHeading, sm.calculatedAngle);

            if (sm.errorAngle > sm.findError(targetHeading, sm.opAngle)) { //reverse drive motor
                sm.errorAngle = sm.findError(targetHeading, sm.opAngle);
                reverseDriveMotor = -1.0;
            }
            else {
                reverseDriveMotor = 1.0;
            }
            sm.pidController();
        }
        
        driveSpeed[0] = (Math.sqrt(Math.pow(xAxis,2)+Math.pow(yAxis,2)) - this.xAxis2)*Swerve.maxVolt*reverseDriveMotor;
        driveSpeed[1] = (Math.sqrt(Math.pow(xAxis,2)+Math.pow(yAxis,2)) + this.xAxis2)*Swerve.maxVolt*reverseDriveMotor;
        driveSpeed[2] = (Math.sqrt(Math.pow(xAxis,2)+Math.pow(yAxis,2)) + this.xAxis2)*Swerve.maxVolt*reverseDriveMotor;
        driveSpeed[3] = (Math.sqrt(Math.pow(xAxis,2)+Math.pow(yAxis,2)) - this.xAxis2)*Swerve.maxVolt*reverseDriveMotor;

        for (double d : driveSpeed) {
            if (d > Swerve.maxVolt) {
                d = Swerve.maxVolt;
            }
            else if (d < -Swerve.maxVolt) {
                d = -Swerve.maxVolt;
            }
        }

        swerveMods[0].get_driveMotor().setVoltage(driveSpeed[0]);
        swerveMods[1].get_driveMotor().setVoltage(driveSpeed[1]);
        swerveMods[2].get_driveMotor().setVoltage(driveSpeed[2]);
        swerveMods[3].get_driveMotor().setVoltage(driveSpeed[3]);
    }

    public void findTargetHeading(double xAxis, double yAxis) {

        if (Math.abs(xAxis) > Swerve.deadband && Math.abs(yAxis) > Swerve.deadband) {
            targetHeading = Math.atan2(-yAxis, xAxis)*180/Math.PI;
        }
        if (targetHeading < 0) {
            targetHeading+=360;
        }
        
    }

    public void setInitHeading(double init) {
        swerveMods[0].setInitAngle(init);
        swerveMods[1].setInitAngle(init);
        swerveMods[2].setInitAngle(init);
        swerveMods[3].setInitAngle(init);
    }


}
