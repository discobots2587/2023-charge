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

    // below are methods that get critical electronics of each module
    
    /**
     * Returns motor controller object that is controlling the the angle motor
     * @return <B>SparkMax</B> that interfaces with the motor controller for angle motor
     */
    public CANSparkMax get_angleMotor() {
        return angleMotor;
    }

    /**
     * Returns motor controller object that is controlling the the drive motor
     * @return <B>SparkMax</B> that interfaces with the motor controller for drive motor
     */
    public CANSparkMax get_driveMotor() {
        return driveMotor;
    }

    /**
     * Returns integrated encoder object for the angle motor
     * @return <B>RelativeEncoder</B> that interfaces the integrated encoder inside the angle motor
     */
    public RelativeEncoder get_integratedDriveEncoder() {
        return integratedDriveEncoder;
    }

    /**
     * Returns integrated encoder object for the drive motor
     * @return <B>RelativeEncoder</B> that interfaces the integrated encoder inside the drive motor
     */
    public RelativeEncoder get_integratedAngleEncoder() {
        return integratedAngleEncoder;
    }

    /**
     * Returns analog encoder object for wheel direction
     * @return <B>AnalogEncoder</B> that interfaces with the analog encoder on the module
     */
    public AnalogEncoder get_angleEncoder() {
        return angleEncoder;
    }

    /**
     * Sets starting angle of the swerve module to define offsets for the angle encoder
     * @param initangle angle of drivetrain when initialized
     */
    public void setInitAngle(double initangle) {
        initAngle = initangle;
        offsetAngle = angleEncoder.getAbsolutePosition()*360 - initAngle;
    }

    /**
     * Finds the angle of the angleMotor when it is called
     * 
     * This method uses the absolute position of the angle encoder as the measurement device
     * <p>
     * 0 <= <B>calculatedAngle</B> < 360
     */
    public void findAngle() {
        calculatedAngle = angleEncoder.getAbsolutePosition()*360 - offsetAngle;

        // uses unit circle to find equivalent angle in the interval [0,360)
        if (calculatedAngle >= 360) {
          calculatedAngle -= 360;
        }
        else if (calculatedAngle < 0) {
          calculatedAngle += 360;
        }
    }

    public double findError(double target, double current) {
        double error = 0.0;

        if (target >= current) { // when current value is greater than that of the setpoint 
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
        else { // when current value is less than that of the setpoint 
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
            error = findError(oppoTarget, current);
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
