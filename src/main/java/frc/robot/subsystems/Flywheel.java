package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.Constants;
//import frc.robot.RobotState;
//import frc.robot.ShooterAimingParameters;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.CircularBuffer;
import frc.lib.util.ReflectingCSVWriter;
import frc.lib.util.Util;
import frc.lib.util.drivers.Talon.CANTalonFactory;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The shooter subsystem consists of 4 775 Pro motors driving twin backspin flywheels. When run in reverse, these motors
 * power the robot's climber through a 1 way bearing. The shooter subsystem goes through 3 stages when shooting. 
 * 1. Spin Up 
 *  Use a PIDF controller to spin up to the desired RPM. We acquire this desired RPM by converting the camera's range
 *  value into an RPM value using the range map in the {@link Constants} class. 
 * 2. Hold When Ready
 *  Once the flywheel's
 *  RPM stabilizes (remains within a certain bandwidth for certain amount of time), the shooter switches to the hold when
 *  ready stage. In this stage, we collect kF samples. The idea is that we want to run the shooter in open loop when we
 *  start firing, so in this stage we calculate the voltage we need to supply to spin the flywheel at the desired RPM. 
 * 3. Hold 
 *  Once we collect enough kF samples, the shooter switches to the hold stage. This is the stage that we begin
 *  firing balls. We set kP, kI, and kD all to 0 and use the kF value we calculated in the previous stage for essentially
 *  open loop control. The reason we fire in open loop is that we found it creates a much narrower stream and leads to
 *  smaller RPM drops between fuel shots.
 * 
 * @see Subsystem.java
 */
public class Flywheel extends Subsystem {
    private static Flywheel mInstance = null;

    public static class ShooterDebugOutput {
        public double timestamp;
        public double setpoint;
        public double rpm;
        public double voltage;
        public ControlMethod control_method;
        public double kF;
        public double range;
    }

    public static int kSpinUpProfile = 0;
    public static int kHoldProfile = 1;

    public static Flywheel getInstance() {
        if (mInstance == null) {
            mInstance = new Flywheel();
        }
        return mInstance;
    }

    public enum ControlMethod {
        OPEN_LOOP, // open loop voltage control for running the climber
        SPIN_UP, // PIDF to desired RPM
        HOLD_WHEN_READY, // calculate average kF
        HOLD, // switch to pure kF control
    }

    private final TalonSRX mMasterSrx;

    private ControlMethod mControlMethod;
    private double mSetpointRpm;
    private double mLastRpmSpeed;

    private CircularBuffer mKfEstimator = new CircularBuffer(Constants.kShooterKfBufferSize);

    // Used for transitioning from spin-up to hold loop.
    private boolean mOnTarget = false;
    private double mOnTargetStartTime = Double.POSITIVE_INFINITY;

    private ShooterDebugOutput mDebug = new ShooterDebugOutput();

    private final ReflectingCSVWriter<ShooterDebugOutput> mCSVWriter;

    

    private Flywheel() {
        mMasterSrx = CANTalonFactory.createTalon(Constants.kShooterMasterId, false, NeutralMode.Coast, FeedbackDevice.CTRE_MagEncoder_Relative, 0, true);
        
        //mMasterSrx.changeControlMode(TalonControlMode.Voltage);
     
        
        
        mMasterSrx.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        mMasterSrx.configVelocityMeasurementWindow(32);
        mMasterSrx.configVoltageCompSaturation(12);
        //mMasterSrx.configNominalClosedLoopVoltage(12);
        mMasterSrx.enableVoltageCompensation(true);//TODO: new thing

        mMasterSrx.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 2);
        mMasterSrx.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 2);


        ErrorCode e = mMasterSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        if (e== ErrorCode.SensorNotPresent) {
            DriverStation.reportError("Could not detect shooter encoder: " + e, false);//TODO: test encoder not being there maybe
        }

       

        refreshControllerConsts();

        mControlMethod = ControlMethod.OPEN_LOOP;

        System.out.println("RPM Polynomial: " + Constants.kFlywheelAutoAimPolynomial);

        mCSVWriter = new ReflectingCSVWriter<ShooterDebugOutput>("/home/lvuser/SHOOTER-LOGS.csv",
                ShooterDebugOutput.class);
    }

    /**
     * Load PIDF profiles onto the master talon
     */
    public void refreshControllerConsts() {
        
        mMasterSrx.config_kP(kSpinUpProfile, Constants.kShooterTalonKP);
        mMasterSrx.config_kI(kSpinUpProfile, Constants.kShooterTalonKI);
        mMasterSrx.config_kD(kSpinUpProfile, Constants.kShooterTalonKD);
        mMasterSrx.config_kF(kSpinUpProfile, Constants.kShooterTalonKF);
        mMasterSrx.config_IntegralZone(kSpinUpProfile, Constants.kShooterTalonIZone);

        
        mMasterSrx.config_kP(kHoldProfile, 0.0);
        mMasterSrx.config_kI(kHoldProfile, 0.0);
        mMasterSrx.config_kD(kHoldProfile, 0.0);
        mMasterSrx.config_kF(kHoldProfile, Constants.kShooterTalonKF);
        mMasterSrx.config_IntegralZone(kHoldProfile, 0);

        mMasterSrx.configClosedloopRamp(Constants.kShooterRampRate);
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        double current_rpm = getSpeedRpm();
        SmartDashboard.putNumber("shooter_speed_talon", current_rpm);
        SmartDashboard.putNumber("shooter_speed_error", mSetpointRpm - current_rpm);
        SmartDashboard.putNumber("shooter_output_voltage", mMasterSrx.getMotorOutputVoltage());
        SmartDashboard.putNumber("shooter_setpoint", mSetpointRpm);

        SmartDashboard.putBoolean("shooter on target", isOnTarget());
        // SmartDashboard.putNumber("shooter_talon_position", mRightMaster.getPosition());
        // SmartDashboard.putNumber("shooter_talon_enc_position", mRightMaster.getEncPosition());
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
        mSetpointRpm = 0.0;
    }

    @Override
    public void zeroSensors() {
        // Don't zero the flywheel, it'll make deltas screwy
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Flywheel.this) {
                    mControlMethod = ControlMethod.OPEN_LOOP;
                    mKfEstimator.clear();
                    mOnTarget = false;
                    mOnTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Flywheel.this) {
                    if (mControlMethod != ControlMethod.OPEN_LOOP) {
                        handleClosedLoop(timestamp);
                        mCSVWriter.add(mDebug);
                    } else {
                        // Reset all state.
                        mKfEstimator.clear();
                        mOnTarget = false;
                        mOnTargetStartTime = Double.POSITIVE_INFINITY;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mCSVWriter.flush();
            }
        });
    }

    /**
     * Run the shooter in open loop, used for climbing
     */
    public synchronized void setOpenLoop(double voltage) {//TODO: change open loop to percent output
        if (mControlMethod != ControlMethod.OPEN_LOOP) {
            mControlMethod = ControlMethod.OPEN_LOOP;
            //mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
            mMasterSrx.configPeakCurrentLimit(Constants.kShooterOpenLoopCurrentLimit);
            mMasterSrx.enableCurrentLimit(true);
        }
        mMasterSrx.set(ControlMode.PercentOutput,voltage);
    }

    /**
     * Put the shooter in spinup mode
     */
    public synchronized void setSpinUp(double setpointRpm) {
        if (mControlMethod != ControlMethod.SPIN_UP) {
            configureForSpinUp();
        }
        mSetpointRpm = setpointRpm;
    }

    /**
     * Put the shooter in hold when ready mode
     */
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if (mControlMethod == ControlMethod.OPEN_LOOP || mControlMethod == ControlMethod.SPIN_UP) {
            configureForHoldWhenReady();
        }
        mSetpointRpm = setpointRpm;
    }

    /**
     * Configure talons for spin up
     */
    private void configureForSpinUp() {
        mControlMethod = ControlMethod.SPIN_UP;
       // mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mMasterSrx.selectProfileSlot(kSpinUpProfile, 0);
        mMasterSrx.enableCurrentLimit(false);
        mMasterSrx.enableVoltageCompensation(false);
        mMasterSrx.configClosedloopRamp(Constants.kShooterRampRate);
    }

    /**
     * Configure talons for hold when ready
     */
    private void configureForHoldWhenReady() {
        mControlMethod = ControlMethod.HOLD_WHEN_READY;
        //mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mMasterSrx.selectProfileSlot(kSpinUpProfile, 0);
        mMasterSrx.enableCurrentLimit(false);
        mMasterSrx.enableVoltageCompensation(false);
        mMasterSrx.configClosedloopRamp(Constants.kShooterRampRate);
    }

    /**
     * Configure talons for hold
     */
    private void configureForHold() {
        mControlMethod = ControlMethod.HOLD;
        //mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mMasterSrx.selectProfileSlot(kHoldProfile, 0);
        mMasterSrx.enableCurrentLimit(false);
        mMasterSrx.configVoltageCompSaturation(12.0);
        mMasterSrx.config_kF(kHoldProfile, mKfEstimator.getAverage());
        mMasterSrx.configClosedloopRamp(Constants.kShooterHoldRampRate);
    }

    private void resetHold() {
        mKfEstimator.clear();
        mOnTarget = false;
    }

    /**
     * Estimate the kF value from current RPM and voltage
     */
    private double estimateKf(double rpm, double voltage) {
        final double speed_in_ticks_per_100ms = 4096.0 / 600.0 * rpm;
        final double output = 1023.0 / 12.0 * voltage;
        return output / speed_in_ticks_per_100ms;
    }

    /**
     * Main control loop of the shooter. This method will progress the shooter through the spin up, hold when ready, and
     * hold stages.
     */
    private void handleClosedLoop(double timestamp) {
        final double speed = getSpeedRpm();
        final double voltage = mMasterSrx.getMotorOutputVoltage();
        mLastRpmSpeed = speed;

        // See if we should be spinning up or holding.
        if (mControlMethod == ControlMethod.SPIN_UP) {
            mMasterSrx.set(ControlMode.Velocity, mSetpointRpm);
            resetHold();
        } else if (mControlMethod == ControlMethod.HOLD_WHEN_READY) {
            final double abs_error = Math.abs(speed - mSetpointRpm);
            final boolean on_target_now = mOnTarget ? abs_error < Constants.kShooterStopOnTargetRpm
                    : abs_error < Constants.kShooterStartOnTargetRpm;
            if (on_target_now && !mOnTarget) {
                // First cycle on target.
                mOnTargetStartTime = timestamp;
                mOnTarget = true;
            } else if (!on_target_now) {
                resetHold();
            }

            if (mOnTarget) {
                // Update Kv.
                mKfEstimator.addValue(estimateKf(speed, voltage));
            }
            if (mKfEstimator.getNumValues() >= Constants.kShooterMinOnTargetSamples) {
                configureForHold();
            } else {
                mMasterSrx.set(ControlMode.Velocity, mSetpointRpm);
            }
        }
        // No else because we may have changed control methods above.
        if (mControlMethod == ControlMethod.HOLD) {
            // Update Kv if we exceed our target velocity. As the system heats up, drag is reduced.
            if (speed > mSetpointRpm) {
                mKfEstimator.addValue(estimateKf(speed, voltage));
                mMasterSrx.config_kF(kHoldProfile, mKfEstimator.getAverage());
            }
        }
        mDebug.timestamp = timestamp;
        mDebug.rpm = speed;
        mDebug.setpoint = mSetpointRpm;
        mDebug.voltage = voltage;
        mDebug.control_method = mControlMethod;
        mDebug.kF = mKfEstimator.getAverage();
       // Optional<ShooterAimingParameters> params = RobotState.getInstance().getAimingParameters();
      //  if (params.isPresent()) {
            mDebug.range = 0.0;//params.get().getRange();
      //  } else {
       //     mDebug.range = 0;
       // }
    }

    public synchronized double getSetpointRpm() {
        return mSetpointRpm;
    }

    private double getSpeedRpm() {
        return mMasterSrx.getSelectedSensorVelocity();//TODO: could have to convert everything to rpm's
    }                                                      //thing is it was previously just getSpeed()

   

    public synchronized boolean isOnTarget() {
        return mControlMethod == ControlMethod.HOLD;
    }

    public synchronized double getLastSpeedRpm() {
        return mLastRpmSpeed;
    }

    @Override
    public void writeToLog() {
        mCSVWriter.write();
    }

    public boolean checkSystem() {
        System.out.println("Testing SHOOTER.----------------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThres = 1200;

       // mMasterSrx.changeControlMode(CANTalon.TalonControlMode.Voltage);
       
        mMasterSrx.enableVoltageCompensation(true);
       mMasterSrx.set(ControlMode.PercentOutput, 1.0f);
        Timer.delay(4.0);
        final double currentRightMaster = mMasterSrx.getOutputCurrent();
        final double rpmMaster = mMasterSrx.getSelectedSensorVelocity();
        mMasterSrx.set(ControlMode.PercentOutput, 0.0f);

        Timer.delay(2.0);

       
        System.out.println("Shooter Right Master Current: " + currentRightMaster);
       
        System.out.println("Shooter RPM Master: " + rpmMaster);

        boolean failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Right Master Current Low !!!!!!!!!!");
        }

       
        if (rpmMaster < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Master RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }
}