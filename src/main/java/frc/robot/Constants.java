package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class Swerve {
        public static final double deadband = 0.1;
        public static final double trackWidth = Units.inchesToMeters(20);
        public static final double wheelBase = Units.inchesToMeters(20);
        public static final double maxVolt = 12.0;
        public static final double driveMaxVolt = 3.0;

        public static final int angleMotorCurrentLimit = 30;
        public static final int driveMotorCurrentLimit = 40;

        public static final double initHeading = 0.0;
        
        /* Angle Motor PID Values */
        public static final double angleKP = 0;
        public static final double angleKI = 0;
        public static final double angleKD = 0; //0.03
        public static final double angleKFF = 0.0;
        public static final double minVolt = 3.0; //for correcting the error of a angle motors
        public static final double errorTolerance = 2.5; //in degrees

        /* Drive Motor PID Values */
        public static final double driveKP = 0.005;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0005; //0.0005
        public static final double driveKFF = 0.0;
        }

    public static final class Mod0 { //front right module
        public static final int driveMotorID = 20;
        public static final int angleMotorID = 21;
        public static final int threncID = 1; //0
    }

    public static final class Mod1 { //front left module
        public static final int driveMotorID = 10;
        public static final int angleMotorID = 11;
        public static final int threncID = 0; //1
    }

    public static final class Mod2 { //back left module
        public static final int driveMotorID = 30;
        public static final int angleMotorID = 31;
        public static final int threncID = 2;
    }

    public static final class Mod3 { //back right module
        public static final int driveMotorID = 40;
        public static final int angleMotorID = 41;
        public static final int threncID = 3;
    }

    public static final class Arm {
        public static final int jointMotorID = 50;
        public static final int intakeMotorID = 51;

        public static final int jointMotorCurrentLimit = 40;
        //public static final int intakeMotorCurrentLimit = 25;

        public static final double maxAngle = 90;
    }

    public static final class GroundIntake {
        public static final int gndJointMotorID = 60;
        public static final int gndRollerMotorID = 61;

        public static final int gndJointMotorCurrentLimit = 40;
        //public static final SupplyCurrentLimitConfiguration gndJointMotorCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 0); // was for TalonSRX
        //public static final VictorSPXConfiguration gndRollerMotorCurrentLimit = new VictorSPXConfiguration(); //Victors can not apply a current limit
    }
}