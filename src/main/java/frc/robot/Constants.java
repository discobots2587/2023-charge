package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class Swerve {
        public static final double deadband = 0.1;
        public static final double trackWidth = Units.inchesToMeters(20);
        public static final double wheelBase = Units.inchesToMeters(20);
        public static final double maxVolt = 12.0;

        public static final int angleMotorCurrentLimit = 30;
        public static final int driveMotorCurrentLimit = 40;

        public static final double initHeading = 90.0;
        
        /* Angle Motor PID Values */
        public static final double angleKP = 0.5;
        public static final double angleKI = 0;
        public static final double angleKD = 0.005;
        public static final double angleKFF = 0.0;
        public static final double minVolt = 3.0; //for correcting the eror of a angle motors
        public static final double errorTolerance = 2.5; //in degrees

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;
        }

    public static final class Mod0 { //front right module
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int threncID = 0;
    }

    public static final class Mod1 { //front left module
        public static final int driveMotorID = 11;
        public static final int angleMotorID = 10;
        public static final int threncID = 1;
    }

    public static final class Mod2 { //back left module
        public static final int driveMotorID = 20;
        public static final int angleMotorID = 21;
        public static final int threncID = 2;
    }

    public static final class Mod3 { //front right module
        public static final int driveMotorID = 31;
        public static final int angleMotorID = 30;
        public static final int threncID = 3;
    }
}
