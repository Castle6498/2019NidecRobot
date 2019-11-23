package frc.robot.state_machines;


import java.awt.SystemColor;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;


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
public class ClimbingHelper extends Subsystem {

    static ClimbingHelper mInstance = null;

    public static ClimbingHelper getInstance() {
        if (mInstance == null) {
            mInstance = new ClimbingHelper();
        }
        return mInstance;
    }

    private final Lift mLift = Lift.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Wrist mWrist = Wrist.getInstance();
    public final Suspension mSuspension = Suspension.getInstance();
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,       
        PRECLIMB,
        LIFT,
        STOW,
        HOME
        };

    private SystemState mSystemState = SystemState.IDLE;
    private SystemState mWantedState = SystemState.IDLE;

  
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
       

        @Override
        public void onStart(double timestamp) {
            synchronized (ClimbingHelper.this) {
                mWantedState = SystemState.IDLE;
                mCurrentStateStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (ClimbingHelper.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case PRECLIMB:
                    newState = handlePreClimb();
                    break;
                case LIFT:
                    newState = handleLifting();
                    break;
                case STOW:
                    newState = handleStowing();
                    break;
                case HOME:
                    newState = handleHome();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Climbing state " + mSystemState + " to " + newState + " Timestamp: "
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

    private SystemState defaultIdleTest(){
        if(mSystemState == mWantedState){
            mWantedState=SystemState.IDLE;
            return SystemState.IDLE; 
        }
        else return mWantedState;
    }

    private SystemState handleIdle() {
        if (mStateChanged) {
            stop();
            setClimbTuning(false);
            setVerticalJogMode(VerticalJogMode.DISABLED);
            mIntake.stop();
        }

        return defaultIdleTest();
    }

    private PreClimbHeight mWantedPreClimbHeight = PreClimbHeight.MIDDLE;
    private PreClimbHeight mCurrentPreClimbHeight = PreClimbHeight.MIDDLE;
    boolean preClimbUpdate = false;

    boolean liftReady = false;

    private SystemState handlePreClimb() {
        if(mStateChanged){
           liftReady=false;
           mIntake.stop();
           setVerticalJogMode(VerticalJogMode.LIFT);
        }
 
        if(preClimbUpdate||mStateChanged){
            preClimbUpdate=false;
            mCurrentPreClimbHeight=mWantedPreClimbHeight;
            liftReady=false;
            switch(mCurrentPreClimbHeight){
                case MIDDLE:
                    mLift.setPosition(Constants.preClimbMiddleHeight);
                break;
                case HIGH:
                    mLift.setPosition(Constants.preClimbHighHeight);
                break;
            }
        }

        if(mLift.atPosition()){
            if(!liftReady){
            liftReady=true;
            mWrist.setPosition(-Constants.climbWristAngle);
            System.out.println("pre climb lift ready");
            }
        }else{
            mWrist.setPosition(0);
            liftReady=false;
        }

        switch(mWantedState){
            case LIFT:
                if(liftReady) return mWantedState;
                else return SystemState.PRECLIMB;
            case IDLE:
            case HOME:
            case STOW: //Its fine to go to stow from pre climb
                return mWantedState;
            case PRECLIMB:
            default:
                return SystemState.PRECLIMB;
        }

    }


    boolean  fullBlastReady = false;

    private SystemState handleLifting(){
        if(mStateChanged){
            mIntake.stop();
            setClimbTuning(true);
            fullBlastReady=false;
            mSuspension.setPosition(Constants.suspFloorPos);
            setVerticalJogMode(VerticalJogMode.DISABLED);
        }

        if(mSuspension.atPosition()){
            if(!fullBlastReady){
            fullBlastReady=true;
            System.out.println("Climb Lifting Full Blast Ready");
            }
        }

        if(fullBlastReady){
            setVerticalJogMode(VerticalJogMode.BOTH);



            if(getFullBlast()){
                System.out.println("full blast BOI");
                mSuspension.setPosition(mLift.getPosition()-Constants.liftOffset+Constants.suspClimbOffset);
                mLift.setPosition(Constants.liftClimbSetPoint);
            }


        }

        if(mWantedState!=SystemState.LIFT){
            setClimbTuning(false);
            setVerticalJogMode(VerticalJogMode.DISABLED);
        }

        switch(mWantedState){
            case LIFT:
            case STOW:
                return mWantedState;
            case IDLE:
            case PRECLIMB:
            case HOME:
            default:
                if(mSuspension.getPosition()<.1) return mWantedState;
                else return SystemState.LIFT;
        }
    }

    private SystemState handleStowing(){
        if(mStateChanged){
            mIntake.stop();
            setClimbTuning(false);
            fullBlastReady=false;
            setVerticalJogMode(VerticalJogMode.SUSPENSION);
            mWrist.setPosition(0);
        }

            if(getFullBlast()){
                mSuspension.setPosition(0);
            }

        if(mWantedState!=SystemState.STOW){
            setVerticalJogMode(VerticalJogMode.DISABLED);
        }

        switch(mWantedState){
            case LIFT:
            case STOW:
                return mWantedState;
            case IDLE:
            case PRECLIMB:
            case HOME:
            default:
                if(mSuspension.getPosition()<.1) return mWantedState;
                else return SystemState.LIFT;
        }
    }

    private SystemState handleHome() {
       if(mStateChanged){
           mSuspension.setWantedState(Suspension.ControlState.HOMING);
       }

        mWantedState= SystemState.IDLE;
        return SystemState.IDLE;
    }

    
    //Pre Climb
        public enum PreClimbHeight{
            HIGH,
            MIDDLE
        }

        public void preClimb(PreClimbHeight mode){
            mWantedState=SystemState.PRECLIMB;
            mWantedPreClimbHeight=mode;
            preClimbUpdate=true;
        }
        
    public boolean getClimbing(){
        return mSystemState!=SystemState.IDLE&&mSystemState!=SystemState.HOME;
    }
    

    public void setHorizontalJog(double s){
        if(mSystemState==SystemState.LIFT||mSystemState==SystemState.STOW){
            mIntake.setMotor(s);
            mDrive.setOpenLoop(new DriveSignal(s,-s,true));
        }
    }

    public enum VerticalJogMode{
        LIFT, SUSPENSION, BOTH, DISABLED
    }
    private VerticalJogMode verticalJogMode = VerticalJogMode.DISABLED;

    private void setVerticalJogMode(VerticalJogMode s){
        verticalJogMode=s;
    }

    public void setVerticalJog(double s){
        switch(verticalJogMode){
            case LIFT:
                mLift.jog(s);
            break;
            case SUSPENSION:
                mSuspension.jog(-s);
            case BOTH:
                mLift.jog(s);
                mSuspension.jog(-s);
            break;
        }
    }


    void setClimbTuning(boolean s){
        mLift.setClimbTuning(s);
        mWrist.setClimbTuning(s);
    }
       
    boolean fullBlast=false;

    public void setFullBlast(boolean s){ fullBlast = s;}

    public boolean getFullBlast(){ return fullBlast;}
   

  
    //BORRING Stupid stuff needed :(
   
        public synchronized void setWantedState(SystemState wantedState) {
            mWantedState = wantedState;
        }
    

        @Override
        public void outputToSmartDashboard() {
        
        }

        @Override
        public void stop() {
            mSuspension.setWantedState(Suspension.ControlState.IDLE);
            setClimbTuning(false);
            setVerticalJogMode(VerticalJogMode.DISABLED);
        }

        @Override
        public void zeroSensors() {

        }

        @Override
        public void registerEnabledLoops(Looper enabledLooper) {
            enabledLooper.register(mLoop);
        }

}
*/
