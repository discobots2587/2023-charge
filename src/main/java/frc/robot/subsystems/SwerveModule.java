package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.Swerve;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule {
    public int moduleNumber;
    
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder integratedDriveEncoder;
    private RelativeEncoder integratedAngleEncoder;

    private AnalogInput input;
    private AnalogEncoder angleEncoder;

    private double initAngle, offsetAngle, oppoTarget;
    public double calculatedAngle, reverseDriveMotor, errorAngle, lastErrorAngle;
    public double inputAngleVoltage, integral, derivative;
    public boolean correctAngle;
    public double driveSpeed;

    private double driveError, driveIntegral, driveDerivative, lastErrorDrive;
    public double inputDriveVoltage;

    public SwerveModule(int moduleNumber, int angleMotorID, int driveMotorID, int threncID) {
        this.moduleNumber = moduleNumber;
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        angleMotor.setSmartCurrentLimit(Swerve.angleMotorCurrentLimit);
        driveMotor.setSmartCurrentLimit(Swerve.driveMotorCurrentLimit);

        driveMotor.setIdleMode(IdleMode.kCoast);

        integratedDriveEncoder = driveMotor.getEncoder();
        integratedAngleEncoder = angleMotor.getEncoder();

        input = new AnalogInput(threncID);
        angleEncoder = new AnalogEncoder(input);
    }

    public CANSparkMax get_angleMotor() {
        return angleMotor;
    }

    public CANSparkMax get_driveMotor() {
        return driveMotor;
    }

    public RelativeEncoder get_integratedDriveEncoder() {
        return integratedDriveEncoder;
    }

    public RelativeEncoder get_integratedAngleEncoder() {
        return integratedAngleEncoder;
    }

    public AnalogEncoder get_angleEncoder() {
        return angleEncoder;
    }

    public void setInitAngle(double initangle) {
        initAngle = initangle;
        offsetAngle = angleEncoder.getAbsolutePosition()*360 - initAngle;
    }

    public void findAngle() { //find angleMotor angle
        calculatedAngle = angleEncoder.getAbsolutePosition()*360 - offsetAngle;
        if (calculatedAngle >= 360) {
          calculatedAngle -= 360;
        }
        else if (calculatedAngle < 0) {
          calculatedAngle += 360;
        }
    }

    public double findError(double target, double current) {
        double error = 0.0;

        if (target >= current) {
            //normal
            error = target - current; 
            //optimal
            if (error >= 90) {
                reverseDriveMotor = -1.0;
                if (error >= 180) {
                    error = target - 180 - current;
                }
                else if (error >= 90) {
                    error = -1*(180 - target + current);
                }
            }
            else {
                reverseDriveMotor = 1.0;
            }
        }
        else {
            //normal
            error = target + 360 - current;
            //optimal
            if (error >= 90) {
                reverseDriveMotor = -1.0;
                if (error >= 180) {
                    error = -1*(current - 180 - target);
                }
                else {
                    error = 180 + target - current;
                }
            }
            else {
                reverseDriveMotor = 1.0;
            }
        }
        return error;
    }

    public double findErrorOptimize(double target, double current) {
        double error = target - current; 
        if (Math.abs(error) <= 90) {
            reverseDriveMotor = 1.0;
        }
        else if (error >= 270 && error <= 360) {
            error = 360 - target + current;
            reverseDriveMotor = 1.0;
        }
        else if (error >= -360 && error <= -270) {
            error = -1*(360+error);
            reverseDriveMotor = 1.0;
        }
        else {
            oppoTarget = (target + 180) % 360; 
            error = findErrorOptimize(oppoTarget, current); 
            reverseDriveMotor = -1.0;
        }
        
        return error;
    }

    public void pidControllerAngle() {
        inputAngleVoltage = errorAngle * Swerve.angleKP; //p

        if (Math.abs(errorAngle) >= Swerve.errorTolerance) {
            integral += Math.abs(errorAngle); //i
        }
        derivative = errorAngle - lastErrorAngle; //d
        lastErrorAngle = errorAngle;
        
        inputAngleVoltage = -1.0 * (inputAngleVoltage + integral*Swerve.angleKI + derivative*Swerve.angleKD);

        if (Math.abs(inputAngleVoltage) > Swerve.maxVolt) {
            if (errorAngle < 0) {
                inputAngleVoltage = Swerve.maxVolt;
            }
            else {
                inputAngleVoltage = Swerve.maxVolt*-1;
            }
        }

        if (Math.abs(errorAngle) <= Swerve.errorTolerance) {
            inputAngleVoltage = 0;
            correctAngle = true;
        }
        else {
            correctAngle = false;
            if (Math.abs(inputAngleVoltage) < Swerve.minVolt) {
                if (errorAngle < 0) {
                    inputAngleVoltage = Swerve.minVolt;
                }
                else {
                    inputAngleVoltage = Swerve.minVolt*-1;
                }
            }
        }
        angleMotor.setVoltage(inputAngleVoltage);
    }
    
    public void testPID() {
        if (Math.abs(errorAngle) > Swerve.errorTolerance + 5) {
            if (errorAngle < 0) {
                angleMotor.setVoltage(1.0);
            }
            else if (errorAngle > 0){
                angleMotor.setVoltage(-1.0);
            }
        }
        else {
            angleMotor.setVoltage(0);
        }
    }

    public void pidControllerDrive(double target) {
        driveError = target - driveMotor.getEncoder().getVelocity();
        driveIntegral += driveError;
        driveDerivative = driveError - lastErrorDrive;
        
        inputDriveVoltage = driveError*Swerve.driveKP + driveIntegral*Swerve.driveKI + driveDerivative*Swerve.driveKD;
        
        if (inputDriveVoltage > Swerve.driveMaxVolt) {
            inputDriveVoltage = Swerve.driveMaxVolt;
        }
        else if (inputDriveVoltage < -Swerve.driveMaxVolt) {
            inputDriveVoltage = -Swerve.driveMaxVolt;
        }

        driveMotor.setVoltage(inputDriveVoltage);
    }
    
}
