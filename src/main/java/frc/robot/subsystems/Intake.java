package frc.robot.subsystems;

import frc.lib.util.drivers.Talon.CANTalonFactory;
import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The suspension subsystem consists of dynamo motors that are meant to send one ball at a time into the shooter.
 * There are ir sensors placed before and after the suspension to sense blockage or emptiness of the hopper.
 * The main things this subsystem has to are feed fuel and unjam
 * 
 *
 */
public class Intake extends Subsystem {
    
   

    private static Intake sInstance = null;

    public static Intake getInstance() {
        if (sInstance == null) {
            sInstance = new Intake();
        }
        return sInstance;
    }

    private TalonSRX mTalon; 
    private DigitalInput mPhotoeye;

    public Intake() {
        
         //Talon Initialization 
         mTalon = CANTalonFactory.createTalon(Constants.kIntakeTalonID, 
         true, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);
 
 
         mTalon = CANTalonFactory.setupHardLimits(mTalon,  LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,false,
         LimitSwitchSource.Deactivated,LimitSwitchNormal.Disabled, false);
         
         mTalon = CANTalonFactory.setupSoftLimits(mTalon, false, 0,false, 0);
        
       //Photoeye Initialization
       mPhotoeye=new DigitalInput(Constants.kIntakeSensorPort);
        mTalon.setNeutralMode(NeutralMode.Brake);
       System.out.println("intake on start");
    }

    public enum SystemState {
        IDLE,
        PICKINGUP,
        SHOOTING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private SystemState mWantedState = SystemState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Intake.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
                System.out.println("Intake started");
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Intake.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case PICKINGUP:
                    newState = handlePickingUp(timestamp);
                    break;
                case SHOOTING:
                    newState = handleShooting(timestamp);
                    break;                
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Intake state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }

            ballUpdate(timestamp);
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };


    private SystemState defaultIdleTest(){
        if(mSystemState == mWantedState){
            mWantedState=SystemState.IDLE;
            return SystemState.IDLE; 
        }
        else return mWantedState;
    }

    private SystemState handleIdle() {
        if(mStateChanged){  
            stopMotor();
        }
        //System.out.println("ipdate");
        return defaultIdleTest();
    }

    private SystemState handlePickingUp(double now){
        if(mStateChanged){
            setMotor(Constants.kIntakePickUpSpeed);
        }

        if(hasBall()){
            stopMotor();
            return defaultIdleTest();
        }

        return mWantedState;
    }




    private double shootStartTime=0;
    private SystemState handleShooting(double now){
        if(mStateChanged){
            setMotor(Constants.kIntakeShootSpeed);
            shootStartTime=0;
        }

        if(hasBall()&&shootStartTime==0){
            shootStartTime=now;
        }

        if(now-shootStartTime>=Constants.kIntakeShootPause){
            stopMotor();
            return defaultIdleTest();
        }


        return mWantedState;
    }


    public boolean seesBall(){
        return mPhotoeye.get();
    }


    private boolean mHasBall=false;
    public boolean hasBall(){
        return mHasBall;
    }

    private double ballStartSeenTime=0;
    private double ballSeenTime=0;

    private void ballUpdate(double time){
        boolean seen = seesBall();
        if(!seen){
            ballSeenTime=0;
            ballStartSeenTime=0;
            mHasBall=false;
        }else{
            if(ballStartSeenTime==0)ballStartSeenTime=time;
            ballSeenTime=time;
        } 

        if(ballSeenTime-ballStartSeenTime>=Constants.kIntakeBallRequiredTime){
            mHasBall=true;
        }
    }

   
    
    public synchronized void setMotor(double s){
        mTalon.set(ControlMode.PercentOutput,s);
        
    }

    
    public double getCurrent(){
        double current = mTalon.getOutputCurrent();
        if(true) System.out.println("Intake current: "+current);
        return current;
    }

    public boolean getStalled(){
        return getCurrent()>=Constants.kIntakeCurrentThreshold;
    }

    //Boring Stuff

        private void stopMotor(){
            mTalon.set(ControlMode.PercentOutput,.1);
            //mTalon.set(ControlMode.PercentOutput, .1);
        }


        public synchronized void setWantedState(SystemState state) {
            mWantedState = state;
        }

        @Override
        public void outputToSmartDashboard() {
            // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kIntakeSensorGearReduction);
        }

        @Override
        public void stop() {
            setWantedState(SystemState.IDLE);
            stopMotor();
        }

        @Override
        public void zeroSensors() {
        }


        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

        public boolean checkSystem() {
            System.out.println("Testing Intake.-----------------------------------");
            boolean failure=false;       
            return !failure;
        }

}