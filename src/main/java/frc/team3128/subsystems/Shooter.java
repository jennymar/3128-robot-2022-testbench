package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.team3128.Constants;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import frc.team3128.common.NAR_PIDSubsystem;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;

public class Shooter extends NAR_PIDSubsystem{
    
    public enum ShooterState {

        OFF(0),
        LAUNCHPAD(0),
        UPPERHUB(0),
        LOWERHUB(0);

        public double shooterRPM;

        private ShooterState(double RPM) {
            this.shooterRPM = RPM;
        }
    }

    private static Shooter instance;


    //Motors
    private NAR_TalonSRX m_leftShooter = new NAR_TalonSRX(Constants.ShooterConstants.LEFT_SHOOTER_ID);
    //private NAR_TalonSRX m_rightShooter = new NAR_TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_ID);

    private ShooterState shooterState = ShooterState.OFF;
    private double time = 0, preTime = 0;
    private double thresholdPercent = Constants.ShooterConstants.RPM_THRESHOLD_PERCENT;

    public Shooter() {
        super(new PIDController(Constants.ShooterConstants.SHOOTER_PID_kP, Constants.ShooterConstants.SHOOTER_PID_kI, Constants.ShooterConstants.SHOOTER_PID_kD), Constants.ShooterConstants.PLATEAU_COUNT);
    }

    public static synchronized Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }


     /**
     * @return If the shooter is at the setpoint RPM
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }


    /**
     * Sets the Shooter State for the shooter
     * @param state Desired Shooter State
     */
    public void setState(ShooterState state) {
        this.shooterState = state;
    }

    /**
     * Begins the PID loop to achieve the desired RPM with the currently set Shooter State
     */
    public void startPID() {
        thresholdPercent = Constants.ShooterConstants.RPM_THRESHOLD_PERCENT;
        super.startPID();
        super.setSetpoint(shooterState.shooterRPM);
        getController().setTolerance(Constants.ShooterConstants.RPM_THRESHOLD_PERCENT * shooterState.shooterRPM);
    }

    /**
     * Sets the state using {@link #setState(ShooterState)} and begins the PID loop to achieve it using {@link #startPID()}
     * @param state Desired Shooter State
     */
    public void beginShoot(ShooterState state) {
        setState(state);
        startPID();
    }

    public void stopShoot() {
        beginShoot(ShooterState.OFF);
    }

    /**
     * @return The Shooter State
     */
    public ShooterState getState() {
        return shooterState;
    }

    @Override
    protected double getMeasurement() {
        return m_leftShooter.getSelectedSensorVelocity() * Constants.ConversionConstants.ENCODER_TO_RPM;
    }

    /**
     * Use the raw voltage output from the PID loop, add a feed forward component, and convert it to a percentage of total
     * possible voltage to apply to the motors.
     * 
     * @param output Output from the PID Loop (RPM)
     * @param setpoint The desired setpoint RPM for the PID Loop (RPM)
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        double voltageOutput = output + 0.0019;
        double voltage = RobotController.getBatteryVoltage();
        double percentOutput = voltageOutput/voltage;

        time = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX) {
            thresholdPercent += ((time - preTime) * ((Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX - Constants.ShooterConstants.RPM_THRESHOLD_PERCENT)) / Constants.ShooterConstants.TIME_TO_MAX_THRESHOLD);
            getController().setTolerance(thresholdPercent * setpoint);
        }

        super.useOutput(setpoint);

        preTime = time;


        percentOutput = (percentOutput > 1) ? 1 : ((percentOutput < -1) ? -1 : percentOutput);
        percentOutput = (setpoint == 0) ? 0 : percentOutput;

        m_leftShooter.set(ControlMode.PercentOutput, percentOutput);
        //m_rightShooter.set(ControlMode.PercentOutput, -percentOutput);


    }
}

