package frc.robot.state_machines;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.Robot;
//import frc.robot.ShooterAimingParameters;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.InterpolatingDouble;


import java.util.Optional;

/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * 
 *
 * HA HA HA HA HA HA HA
 *
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * 
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 * 
 */
/*
public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

  //  private final Feeder mFeeder = Feeder.getInstance();
  //  private final Intake mIntake = Intake.getInstance();
 // private final Hopper mHopper = Hopper.getInstance();
 // private final Flywheel mShooter = Flywheel.getInstance();
 // private final LED mLED = LED.getInstance();

    
    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,       
        WAITING_FOR_FLYWHEEL, // waiting for the shooter to spin up
        SHOOTING, // shooting
        SHOOTING_SPIN_DOWN, // short period after the driver releases the shoot button where the flywheel
                            // continues to spin so the last couple of shots don't go short
        UNJAMMING, // unjamming the feeder and hopper        
        JUST_INTAKE, // run hopper and feeder but not the shooter
        MANUAL_FEED
        };

    // Desired function from user
    public enum WantedState {
        IDLE, SHOOT, UNJAM, MANUAL_FEED, INTAKE
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

  
    private double mCurrentStateStartTime;
    private boolean mStateChanged;


    public synchronized boolean isShooting() {
        return (mSystemState == SystemState.SHOOTING) || (mSystemState == SystemState.SHOOTING_SPIN_DOWN);
    }

    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
       

        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle(mStateChanged);
                    break;
                case WAITING_FOR_FLYWHEEL:
                    newState = handleWaitingForFlywheel();
                    break;
                case SHOOTING:
                    newState = handleShooting(timestamp);
                    break;
                case UNJAMMING:
                    newState = handleUnjamming();
                    break;
                case JUST_INTAKE:
                    newState = handleJustIntake();
                    break;
                case SHOOTING_SPIN_DOWN:
                    newState = handleShootingSpinDown(timestamp);
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
        }

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case SHOOT:
            return SystemState.WAITING_FOR_FLYWHEEL;
        case MANUAL_FEED:
            return SystemState.MANUAL_FEED;
        case INTAKE:
            return SystemState.JUST_INTAKE;
        default:
            return SystemState.IDLE;
        }
    }

    

    private SystemState handleWaitingForFlywheel() {
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mIntake.setWantedState(Intake.WantedState.INTAKE);

       if (autoSpinShooter(true)) {
            System.out.println(Timer.getFPGATimestamp() + ": making shot: Range: " + mLastGoalRange + " setpoint: "
                    + mShooter.getSetpointRpm());

            return SystemState.SHOOTING;
        }
        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case SHOOT:
            return SystemState.WAITING_FOR_FLYWHEEL;
        case MANUAL_FEED:
            return SystemState.MANUAL_FEED;
        case INTAKE:
            return SystemState.JUST_INTAKE;
            break;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleShooting(double timestamp) {
        // Don't auto spin anymore - just hold the last setpoint
        mFeeder.setWantedState(Feeder.WantedState.CONTINUOUS_FEED);

        setWantIntakeOnForShooting();

        // Pump circular buffer with last rpm from talon.
        final double rpm = mShooter.getLastSpeedRpm();

        if (mStateChanged) {
            mShooterRpmBuffer.clear();
        }

        // Find time of last shooter disturbance.
        if ((timestamp - mCurrentStateStartTime < Constants.kShooterMinShootingTime) ||
                !mShooterRpmBuffer.isFull() ||
                (Math.abs(mShooterRpmBuffer.getAverage() - rpm) > Constants.kShooterDisturbanceThreshold)) {
            mLastDisturbanceShooterTime = timestamp;
        }

        mShooterRpmBuffer.addValue(rpm);

        switch (mWantedState) {
        case SHOOT:


            boolean jam_detected = false;
            if (timestamp - mLastDisturbanceShooterTime > Constants.kShooterJamTimeout) {
                // We have jammed, move to unjamming.
                jam_detected = true;
            }
            SmartDashboard.putBoolean("Jam Detected", jam_detected);

            if (jam_detected) {
                return SystemState.UNJAMMING_WITH_SHOOT;
            } else {
                return SystemState.SHOOTING;
            }
        case MANUAL_FEED:
            return SystemState.MANUAL_FEED;
            break;
        default:
            return SystemState.SHOOTING_SPIN_DOWN;
        }
    }


    private SystemState handleShootingSpinDown(double timestamp) {
        // Don't auto spin anymore - just hold the last setpoint
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.FEED);

        // Turn off the floor.
        mHopper.setWantedState(Hopper.WantedState.IDLE);

        mLED.setWantedState(LED.WantedState.FIND_RANGE);
        setWantIntakeOnForShooting();

        if (timestamp - mCurrentStateStartTime > Constants.kShooterSpinDownTime) {
            switch (mWantedState) {
            case UNJAM:
                return SystemState.UNJAMMING;
           case SHOOT:
                return SystemState.WAITING_FOR_FLYWHEEL;
            case MANUAL_FEED:
                return SystemState.MANUAL_FEED;
            case INTAKE:
                return SystemState.JUST_INTAKE;
            default:
                return SystemState.IDLE;
            }
        }
        return SystemState.SHOOTING_SPIN_DOWN;
    }

    private SystemState handleUnjamming() {
       

       mFeeder.setWantedState(Feeder.WantedState.UNJAM);
       mIntake.setWantedState(Intake.WantedState.UNJAM);

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case SHOOT:
            return SystemState.WAITING_FOR_FLYWHEEL;
         case MANUAL_FEED:
             return SystemState.MANUAL_FEED;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleJustIntake() {
       //TODO make shooter stop
        mFeeder.setWantedState(Feeder.WantedState.CONTINUOUS_FEED);
        mIntake.setWantedState(Intake.WantedState.INTAKE);

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case SHOOT:
            return SystemState.WAITING_FOR_FLYWHEEL;
        case MANUAL_FEED:
            return SystemState.MANUAL_FEED;
        default:
            return SystemState.IDLE;
        }
    }

    private double handleManualFeed(){
       mFeeder.setWantedState(Feeder.WantedState.INCREMENT_FEED);
       mIntake.setWantedState(Intake.WantedState.INTAKE);

        //TODO fix how this is setup (in the FEEDER class)


    }





   

  

   
    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }
   

    @Override
    public void outputToSmartDashboard() {
       
    }

    @Override
    public void stop() {
        //TODO: set all subsystems to idle
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

}*/
