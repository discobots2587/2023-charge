package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mod0;
import frc.robot.Constants.Mod1;
import frc.robot.Constants.Mod2;
import frc.robot.Constants.Mod3;
import frc.robot.Constants.Swerve;
public class SwerveDrive extends SubsystemBase{
    public SwerveModule[] swerveMods;

    public double targetHeading = 0;
    public double turnAngle; // Wheel angle while turning
    public double moduleAngle; // Wheel angle with offset

    public SwerveDrive() {
        swerveMods = new SwerveModule[] {
            new SwerveModule(0,Mod0.angleMotorID,Mod0.driveMotorID, Mod0.threncID),
            new SwerveModule(1,Mod1.angleMotorID,Mod1.driveMotorID, Mod1.threncID),
            new SwerveModule(2,Mod2.angleMotorID,Mod2.driveMotorID, Mod2.threncID),
            new SwerveModule(3,Mod3.angleMotorID,Mod3.driveMotorID, Mod3.threncID),
        };
    }

    public void drive(double x1, double y1) {
        //applying deadband
        if (Math.abs(x1) < Swerve.deadband) {
            x1=0;
        }
        if (Math.abs(y1) < Swerve.deadband) {
            y1=0;
        }
        //convert joystick values to target heading
        findTargetHeading(x1, y1);
        //setting speed for each angle motors
        for (SwerveModule sm : swerveMods) {
            sm.findAngle();
            sm.errorAngle = sm.findErrorOptimize(targetHeading, sm.calculatedAngle);
            sm.turnPID();
        }
        //setting speed for each drive motors

        y1*=-1;

        double frSpeed = Math.sqrt(x1*x1+y1*y1);
        double flSpeed = Math.sqrt(x1*x1+y1*y1);
        double blSpeed = Math.sqrt(x1*x1+y1*y1);
        double brSpeed = Math.sqrt(x1*x1+y1*y1);

        swerveMods[0].get_driveMotor().setVoltage(frSpeed*2.0*swerveMods[0].reverseDriveMotor);
        swerveMods[1].get_driveMotor().setVoltage(flSpeed*2.0*swerveMods[1].reverseDriveMotor);
        swerveMods[2].get_driveMotor().setVoltage(blSpeed*2.0*swerveMods[2].reverseDriveMotor);
        swerveMods[3].get_driveMotor().setVoltage(brSpeed*2.0*swerveMods[3].reverseDriveMotor);
    }
    
    /**
     * Turn Method using triggers
     * @param leftTrigger axis of left trigger [0,1]
     * @param rightTrigger axis of right trigger [0,1]
     * @param turnPowerConstant constant for turning power [0,1]
     */
    public void turnTrigger(double leftTrigger, double rightTrigger, double turnPowerConstant) {
        turnAngle(Swerve.chassisLengthInches, Swerve.chassisWidthInches); // sets wheel angle for turn
        moduleAngle = turnAngle;
        if (leftTrigger > Swerve.deadband) { // left trigger is pressed
            // Set Angle for each swerve wheel
            for (SwerveModule sm : swerveMods) {
                switch (sm.moduleNumber) {
                    case 0:
                        moduleAngle = turnAngle*-1;
                        break;
                    case 1:
                        moduleAngle = turnAngle;
                        break;
                    case 2:
                        moduleAngle = turnAngle*-1;
                        break;
                    case 3:
                        moduleAngle = turnAngle;
                        break;
                    default:
                        moduleAngle = turnAngle;
                }

                sm.findAngle();
                sm.errorAngle = sm.findErrorOptimize(moduleAngle, sm.calculatedAngle);
                sm.turnPID();
                sm.get_driveMotor().setVoltage(leftTrigger*turnPowerConstant*sm.reverseDriveMotor);
            }
        } else if (rightTrigger > Swerve.deadband) { // right trigger is pressed
            // Set Angle for each swerve wheel
            for (SwerveModule sm : swerveMods) {
                switch (sm.moduleNumber) {
                    case 0:
                        moduleAngle = turnAngle*-1;
                        break;
                    case 1:
                        moduleAngle = turnAngle;
                        break;
                    case 2:
                        moduleAngle = turnAngle*-1;
                        break;
                    case 3:
                        moduleAngle = turnAngle;
                        break;
                    default:
                        moduleAngle = turnAngle;
                }

                sm.findAngle();
                sm.errorAngle = sm.findErrorOptimize(moduleAngle, sm.calculatedAngle);
                sm.turnPID();
                sm.get_driveMotor().setVoltage(rightTrigger*turnPowerConstant*sm.reverseDriveMotor);
            }
        }
    }

    /**
     * Turn method when using IMU
     * @param targetAngle desired angle from joystick
     * @param currentAngle current angle from IMU
     */
    public void turnIMU(double targetAngle, double currentAngle) {

    }

    /**
     * Finds the angle of the wheels during turn functionality
     * @param chassisLengthInches long side of chassis
     * @param chassisWidthInches short side of chassis
     */
    public void turnAngle(double chassisLengthInches, double chassisWidthInches) {
        // calculates turn angle needed for specific chassis side
        turnAngle = Math.abs(
            Math.acos(
                (chassisLengthInches/2-Swerve.moduleOffsetInches)
                /
                Math.sqrt(Math.pow((Swerve.chassisLengthInches/2),2)+Math.pow((Swerve.chassisWidthInches/2),2))
                ));
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
            sm.get_angleMotor().restoreFactoryDefaults();
            sm.get_driveMotor().restoreFactoryDefaults();
            sm.get_angleMotor().setIdleMode(IdleMode.kBrake);

            sm.get_driveMotor().getEncoder().setPosition(0.0);
            sm.get_angleMotor().setSmartCurrentLimit(30);
            sm.get_driveMotor().setSmartCurrentLimit(40);
        }
    }
}