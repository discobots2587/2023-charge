package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mod0;
import frc.robot.Constants.Mod1;
import frc.robot.Constants.Mod2;
import frc.robot.Constants.Mod3;
import frc.robot.Constants.Swerve;

public class SwerveDrive extends SubsystemBase{
    public SwerveModule[] swerveMods;

    double xAxis1, yAxis1, xAxis2;

    public double targetHeading = 0;

    public SwerveDrive() {
        swerveMods = new SwerveModule[] {
            new SwerveModule(0,Mod0.angleMotorID,Mod0.driveMotorID, Mod0.threncID),
            new SwerveModule(1,Mod1.angleMotorID,Mod1.driveMotorID,Mod1.threncID),
            new SwerveModule(1,Mod2.angleMotorID,Mod2.driveMotorID, Mod2.threncID),
            new SwerveModule(1,Mod3.angleMotorID,Mod3.driveMotorID, Mod3.threncID),
        };
    }

    public void test(boolean button1, boolean button2) {
        if (button1) {
            swerveMods[0].get_driveMotor().setVoltage(Swerve.driveMaxVolt);
            swerveMods[1].get_driveMotor().setVoltage(Swerve.driveMaxVolt);
            swerveMods[2].get_driveMotor().setVoltage(Swerve.driveMaxVolt);
            swerveMods[3].get_driveMotor().setVoltage(Swerve.driveMaxVolt);
        }
        else if (button2) {
            swerveMods[0].get_driveMotor().setVoltage(-Swerve.driveMaxVolt);
            swerveMods[1].get_driveMotor().setVoltage(-Swerve.driveMaxVolt);
            swerveMods[2].get_driveMotor().setVoltage(-Swerve.driveMaxVolt);
            swerveMods[3].get_driveMotor().setVoltage(-Swerve.driveMaxVolt);
        }
    }

    public void drive(double x1, double y1, double x2) {
        if (Math.abs(x1) < Swerve.deadband) {
            xAxis1 = 0;
        }
        else {
            xAxis1 = x1;
        }
        if (Math.abs(y1) < Swerve.deadband) {
            yAxis1 = 0;
        }
        else {
            yAxis1 = x1;
        }
        findTargetHeading(x1, y1);
        xAxis2 = x2;
        if (Math.abs(xAxis2) < Swerve.deadband) {
            xAxis2 = 0;
        }

        for (SwerveModule sm : swerveMods) {
            sm.findAngle();
            sm.errorAngle = sm.findError(targetHeading, sm.calculatedAngle);
            sm.pidController();
            
            //sm.driveSpeed = sm.reverseDriveMotor;
            
        }
        
        swerveMods[0].driveSpeed *= (Math.sqrt(Math.pow(xAxis1,2)+Math.pow(yAxis1,2)) - xAxis2)*Swerve.driveMaxVolt*swerveMods[0].reverseDriveMotor;
        swerveMods[1].driveSpeed *= (Math.sqrt(Math.pow(xAxis1,2)+Math.pow(yAxis1,2)) + xAxis2)*Swerve.driveMaxVolt*swerveMods[1].reverseDriveMotor;
        swerveMods[2].driveSpeed *= (Math.sqrt(Math.pow(xAxis1,2)+Math.pow(yAxis1,2)) + xAxis2)*Swerve.driveMaxVolt*swerveMods[2].reverseDriveMotor;
        swerveMods[3].driveSpeed *= (Math.sqrt(Math.pow(xAxis1,2)+Math.pow(yAxis1,2)) - xAxis2)*Swerve.driveMaxVolt*swerveMods[3].reverseDriveMotor;

        for (SwerveModule sm : swerveMods) {
            if (sm.driveSpeed > Swerve.driveMaxVolt) {
                sm.driveSpeed = Swerve.driveMaxVolt;
            }
            else if (sm.driveSpeed < -Swerve.driveMaxVolt) {
                sm.driveSpeed = -Swerve.driveMaxVolt;
            }
            if (swerveMods[0].correctAngle && swerveMods[1].correctAngle && swerveMods[2].correctAngle && swerveMods[3].correctAngle) {
              sm.get_driveMotor().setVoltage(sm.driveSpeed);
            }
        }
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

    public void initEncoders() {
        for (SwerveModule sm : swerveMods) {
            sm.get_driveMotor().getEncoder().setPosition(0.0);
        }
    }

}