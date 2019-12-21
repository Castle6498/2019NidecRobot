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
import edu.wpi.first.wpilibj.Spark;

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

    
    private Spark mIntakeMotor, mIntakeActuatorMotor;
    private DigitalInput mMinLimit, mMaxLimit;

    public Intake() {
        
         //Spark Initialization 
         
        //EJ initialize ur cwappy sparks
        mIntakeMotor = new Spark(Constants.kIntakeSparkPort);
        mIntakeActuatorMotor = new Spark(Constants.kLinearActuatorSparkPort);

        mMinLimit = new DigitalInput(Constants.kIntakeMinLimitPort);
        mMaxLimit = new DigitalInput(Constants.kIntakeMaxLimitPort);
       
    }

    public enum WantedState {
        IDLE,
        PICKUP, //only able to go to this when unfolded
        UNFOLD
    }

    public enum SystemState {
        IDLE,
        PICKINGUP,
        UNFOLDED,
        LOWERING,
        RAISING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE; //EJ notice how there is a wanted state

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
                case UNFOLDED:
                    newState = handleUnfolded();
                    break;  
                case LOWERING:
                    newState = handleLowering();
                    break;
                case RAISING:
                    newState = handleRaising();              
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

           
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };


  

    private SystemState handleIdle() {
        if(mStateChanged){  
            stopMotors();
        }
        //System.out.println("ipdate");
        switch(mWantedState){
            case PICKUP:
            case UNFOLD:
                return SystemState.LOWERING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handlePickingUp(double now){
        if(mStateChanged){
            setRollers(Constants.kIntakePickUpSpeed);
        }

        if(mWantedState!=WantedState.PICKUP){
            stopMotors();
        }


       switch(mWantedState){
            case IDLE:
                return SystemState.RAISING;
            case UNFOLD:
                return SystemState.UNFOLDED;
            default:
                return SystemState.PICKINGUP;
        }
    }
    
    private SystemState handleUnfolded(){
        if(mStateChanged){
           
        }

        switch(mWantedState){
            case IDLE:
                return SystemState.RAISING;
            case PICKUP:
                return SystemState.PICKINGUP;
            default:
                return SystemState.UNFOLDED;
        }
    }

    private SystemState handleLowering(){
        if(mStateChanged){
           setActuator(Constants.kIntakeActuationSpeed);
        }

        if(getMaxLimit()){
            setActuator(0);
            return SystemState.UNFOLDED;
        }
        
        switch(mWantedState){
            case IDLE:
                return SystemState.RAISING;
            default:
                return SystemState.LOWERING;
        }
    }

    private SystemState handleRaising(){
        if(mStateChanged){
           setActuator(-Constants.kIntakeActuationSpeed);
        }

        if(getMinLimit()){
            setActuator(0);
            return SystemState.IDLE;
        }
        
        switch(mWantedState){
            case UNFOLD:
            case PICKUP:
                return SystemState.LOWERING;
            default:
                return SystemState.RAISING;
        }
    }
    
    private void setRollers(double s){
        mIntakeMotor.set(s);
        
    }

    public void setActuator(double s){
        mIntakeActuatorMotor.set(s);
    }

    private boolean getMinLimit(){
        return mMinLimit.get();
    }
   
    private boolean getMaxLimit(){
        return mMaxLimit.get();
    }

    

    //Boring Stuff

        private void stopMotors(){
            mIntakeMotor.set(0);
            mIntakeActuatorMotor.set(0);
        }


        public synchronized void setWantedState(WantedState state) {
            mWantedState = state;
        }

        public synchronized SystemState getSystemState(){
            return mSystemState;
        }

        @Override
        public void outputToSmartDashboard() {
            // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kIntakeSensorGearReduction);
        }

        @Override
        public void stop() {
            setWantedState(WantedState.IDLE);
            stopMotors();
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

            //EJ make a cool thing to make it test things, use print outs to verify status, see other subsytems for using timer delays
            return !failure;
        }

}
