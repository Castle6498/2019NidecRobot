package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.RobotState;
import frc.robot.ShooterAimingParameters;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.subsystems.LED.LedMode;



public class Shooter extends Subsystem {

   
    static Shooter mInstance = null;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private final Feeder mFeeder = Feeder.getInstance();
    private final Hood mHood = Hood.getInstance();
    
    private final Flywheel mFlywheel = Flywheel.getInstance();
    private final LED mLED = LED.getInstance();

   

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        SPINNING_UP,
        SHOOTING
    };

    // Desired function from user
    public enum WantedState {
        IDLE,
        SHOOT
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

  

   
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

  

    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mWantStateChangeStartTime;

        @Override
        public void onStart(double timestamp) {
            synchronized (Shooter.this) {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mWantStateChangeStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Shooter.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case SPINNING_UP:
                    newState = handleSpinningUp();
                    break;
                case SHOOTING:
                    newState = handleShooting();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Shooter state " + mSystemState + " to " + newState + " Timestamp: "
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

    

    private SystemState handleIdle() {
        if (mStateChanged) {
            mLED.setLED(LedMode.IDLE); 
            mFeeder.setSystemState(Feeder.SystemState.IDLE);
           mFlywheel.setWantedState(Flywheel.WantedState.IDLE);
        }

        switch (mWantedState) {
        case SHOOT:
            return SystemState.SPINNING_UP;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleSpinningUp() {
        if (mStateChanged) {
            mLED.setLED(LedMode.IDLE); 
            mFeeder.setSystemState(Feeder.SystemState.IDLE);
           mFlywheel.setWantedState(Flywheel.WantedState.SHOOT);
        }

        switch (mWantedState) {
        case SHOOT:
            if(mFlywheel.getSystemState()==Flywheel.SystemState.SHOOTING){
            return SystemState.SHOOTING;
            }
        default:
            return SystemState.SPINNING_UP;
        }
    }



    private SystemState handleShooting() {
        if (mStateChanged) {
            mLED.setLED(LedMode.SHOOT); 
            mFeeder.setSystemState(Feeder.SystemState.FEEDING);
           mFlywheel.setWantedState(Flywheel.WantedState.SHOOT);
        }

        if(!mHood.onPosition()){ //Stops feeding balls when hood moving
            mFeeder.setSystemState(Feeder.SystemState.IDLE);
        } else{
            mFeeder.setSystemState(Feeder.SystemState.FEEDING);
        }

        switch (mWantedState) {
        case IDLE:
            return SystemState.IDLE;
        default:
            return SystemState.SHOOTING;
        }
    }

    public void setHoodAngle(double a){
        mHood.setAngle(a);
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

   

    @Override
    public void outputToSmartDashboard() {
        
    }

    @Override
    public void stop() {
        mSystemState=SystemState.IDLE;
        mLED.setLED(LedMode.IDLE); 
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

   
}
