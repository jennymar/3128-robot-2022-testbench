package frc.team3128;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class Constants {

    public static class ConversionConstants {
        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final double SPARK_ENCODER_RESOLUTION = 42;
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // (sensor units per 100 ms to rpm)
        public static final double ENCODER_TO_RPM = 10*60/FALCON_ENCODER_RESOLUTION; // (sensor units per 100 ms to rpm)
    }

    public static class DriveConstants {

        public static final int DRIVE_MOTOR_LEFT_LEADER_ID = 0;
        public static final int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 1;
        public static final int DRIVE_MOTOR_RIGHT_LEADER_ID = 2;
        public static final int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 3;

        // Sim constants, TODO: move to new class

        // TODO: Get actual kv, ka
        public static final DCMotor GEARBOX = DCMotor.getFalcon500(4); 
        public static final LinearSystem<N2, N2, N2> DRIVE_CHAR = 
        LinearSystemId.identifyDrivetrainSystem(
            5,              // kvVoltSecondsPerMeter
            0.5,            // kaVoltSecondsSquaredPerMeter
            5,              // kvVoltSecondsPerRadian
            0.5             // kaVoltSecondsSquaredPerRadian
        );
        public static final double DRIVE_GEARING = 8;
        public static final double WHEEL_RADIUS_METERS = 0.0508; 
        public static final double TRACK_WIDTH_METERS = 0.66;        
        public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS_METERS * 2 / ConversionConstants.FALCON_ENCODER_RESOLUTION;

        public static final Boolean GYRO_REVERSED = false;
    }

    public static class VisionContants {

        public static final String TOP_HOSTNAME = "limelight-sog";

        public static final int SAMPLE_RATE = 3;

        public static final double TOP_CAMERA_ANGLE = -26.0; //degrees
        public static final double TOP_CAMERA_HEIGHT = 0.0; // Daniel - We had this at 0.0 previously, if we want to do more advanced math using vision this value should be measured - also determine units
        public static final double TOP_FRONT_DIST = 0.0; // Daniel - We had this at 0.0 previously, if we want to do more advanced math using vision this value should be measured.
        public static final double TARGET_WIDTH = 30.0; //inches

        public static final double VISION_PID_kP = 0.01;
        public static final double VISION_PID_kI = 0.02;
        public static final double VISION_PID_kD = 0.00006;

        public static final double TX_OFFSET = 0.0; // to offset alignment in either direction

        public static final double TX_THRESHOLD = 1; //degrees
        public static final double TX_THRESHOLD_MAX = 2; //degrees
        public static final double TIME_TO_MAX_THRESHOLD = 5; //seconds
        public static final double TX_THRESHOLD_INCREMENT = (TX_THRESHOLD_MAX - TX_THRESHOLD) / TIME_TO_MAX_THRESHOLD; //degrees per second

        public static final int ALIGN_PLATEAU_COUNT = 10; //Number of checks at correct RPM to shoot
        
    }

    public static class ShooterConstants {
        public static final int LEFT_SHOOTER_ID = 6; 
        public static final double SHOOTER_PID_kP = 1.24e-3;
        public static final double SHOOTER_PID_kI = 0;
        public static final double SHOOTER_PID_kD = 0;

        public static final double SHOOTER_KS = 0.711; //Static gain in PID Feed Forward
        public static final double SHOOTER_KV = 0.00163; //Velocity gain in PID Feed Forward
        public static final double SHOOTER_KA = 0.0349; //Acceleration gain PID Feed Forward

        public static final int PLATEAU_COUNT = 25;

        public static final double RPM_THRESHOLD_PERCENT = 0.05;
        public static final double RPM_THRESHOLD_PERCENT_MAX = 0.06;
        public static final double TIME_TO_MAX_THRESHOLD = 8;

        public static final LinearSystem<N1, N1, N1> SHOOTER_CHAR = 
        LinearSystemId.identifyVelocitySystem(
            SHOOTER_KV, 
            SHOOTER_KA
        );
        public static final double SHOOTER_RADIUS_METERS = 0.0508;
        public static final DCMotor SHOOTER_GEARBOX = DCMotor.getCIM(2);
        public static final double SHOOTER_GEARING = 1.5;
    }
}
