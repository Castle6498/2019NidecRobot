package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.drivers.Talon.CANTalonFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The suspension subsystem consists of dynamo motors that are meant to send one ball at a time into the shooter.
 * There are ir sensors placed before and after the suspension to sense blockage or emptiness of the hopper.
 * The mail
 * 
 *
 */
public class Lift extends Subsystem {
    
   

    private static Lift sInstance = null;

    public static Lift getInstance() {
        if (sInstance == null) {
            sInstance = new Lift();
        }
        return sInstance;
    }

    private TalonSRX mTalon;
   
    public Lift() {
        
        //Talon Initialization 
        mTalon = CANTalonFactory.createTalon(Constants.kLiftTalonID, 
        true, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

        mTalon = CANTalonFactory.setupHardLimits(mTalon, LimitSwitchSource.Deactivated,
        LimitSwitchNormal.Disabled, false, LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,true);
        
        mTalon = CANTalonFactory.setupSoftLimits(mTalon, true, (int) Math.round(Constants.kLiftSoftLimit*Constants.kLiftTicksPerInch),
        false, 0);
        

        mTalon = CANTalonFactory.tuneLoops(mTalon, 0, Constants.kLiftTalonP,
        Constants.kLiftTalonI, Constants.kLiftTalonD, Constants.kLiftTalonF);


        setClimbTuning(false);

        mTalon = CANTalonFactory.tuneLoops(mTalon, 1, Constants.kLiftClimbP,
        Constants.kLiftClimbI, Constants.kLiftClimbD, Constants.kLiftClimbF);
        
        //setMaxOuput(.75);
     
    }

    public void setClimbTuning(boolean t){
        if(t){
            mTalon.selectProfileSlot(1, 0);
            mTalon.configMotionCruiseVelocity((int) Math.round((Constants.kLiftClimbVelocity*Constants.kLiftTicksPerInch)/(10)));
            mTalon.configMotionAcceleration((int) Math.round((Constants.kLiftClimbAcceleration*Constants.kLiftTicksPerInch)/(10)));
        }else{
            mTalon.selectProfileSlot(0, 0);
            mTalon.configMotionCruiseVelocity((int) Math.round((Constants.kLiftVelocity*Constants.kLiftTicksPerInch)/(10)));
            mTalon.configMotionAcceleration((int) Math.round((Constants.kLiftAcceleration*Constants.kLiftTicksPerInch)/(10)));
        }
    }

    void setMaxOuput(double output){
        mTalon.configClosedLoopPeakOutput(0, output);
    }

    private void setLimitClear(boolean e){
           mTalon.configClearPositionOnLimitR(e,0);    
    }

    public enum ControlState {
        IDLE,
        HOMING,
    }

    private ControlState mControlState = ControlState.IDLE;
    private ControlState mWantedState = ControlState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Lift.this) {
                mControlState = ControlState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Lift.this) {
                ControlState newState;
                switch (mControlState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case HOMING:
                    newState = handleHoming();
                    break;   
                default:
                    newState = ControlState.IDLE;
                }
                if (newState != mControlState) {
                    System.out.println("Lift state " + mControlState + " to " + newState);
                    mControlState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                positionUpdater();
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };


    private ControlState defaultIdleTest(){
        if(mControlState == mWantedState){
            mWantedState=ControlState.IDLE;
            return ControlState.IDLE; 
        }
        else return mWantedState;
    }

    private ControlState handleIdle() {
        if(mStateChanged){
            stopMotor();
        }
        
       return defaultIdleTest();
    }

    private boolean hasHomed = false;

    private ControlState handleHoming(){
        if(mStateChanged){
            hasHomed=false;
            setLimitClear(true);
            mTalon.set(ControlMode.PercentOutput,-.2);
            mTalon.setSelectedSensorPosition(-1);
        }

        if(!hasHomed&&mTalon.getSelectedSensorPosition()==0){
            hasHomed=true;
            mTravelingPosition=.1;
            setPosition(0);
        }

        ControlState newState;

        if(hasHomed){
            if(atPosition()){
            newState= defaultIdleTest();
            }else{
                newState= mWantedState;
            }
        }else{
            newState= ControlState.HOMING;
        }

        if(newState!=ControlState.HOMING)setLimitClear(false);
        return newState;
    }

   
   

    //CLOSED LOOP CONTROL
    private double mWantedPosition = .1;
    private double mTravelingPosition = 0;
    
    public synchronized void setPosition(double pos){
        if(pos>=Constants.kLiftSoftLimit)pos=Constants.kLiftSoftLimit;
        else if(pos<0)pos=0;
        mWantedPosition=pos;
        
       // System.out.println("Set wanted pos to "+pos);
    }
private boolean jog=false;
    public void jog(double amount){
        setPosition(mWantedPosition+=amount);
        jog=true;
    }


   public boolean atPosition(){
      if(Math.abs(mWantedPosition-getPosition())<=Constants.kLiftTolerance){
          return true;
      }else{
          return false;
      }
       
   }

   public double getPosition(){
       return mTalon.getSelectedSensorPosition()/Constants.kLiftTicksPerInch;
   }

   private void positionUpdater(){
           
    if(hasHomed&&mWantedPosition!=mTravelingPosition){

        mTravelingPosition=mWantedPosition;
        if(!jog)System.out.println("Lift to "+mTravelingPosition);
        jog=false;
        mTalon.set(ControlMode.MotionMagic, mTravelingPosition*Constants.kLiftTicksPerInch);
    } 
}
  
   private synchronized void stopMotor(){
        mTalon.set(ControlMode.Disabled,0);
    }

    public synchronized void setWantedState(ControlState state) {
        mWantedState = state;
    }

    @Override
    public void outputToSmartDashboard() {
        // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kLiftSensorGearReduction);
    }

    @Override
    public void stop() {
        setWantedState(ControlState.IDLE);
    }

    @Override
    public void zeroSensors() {
    }


    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing Lift.-----------------------------------");
        boolean failure=false;       
        return !failure;
    }

}