package frc.robot.state_machines;


import java.awt.SystemColor;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;

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
public class BallControlHelper extends Subsystem {

    static BallControlHelper mInstance = null;

    public static BallControlHelper getInstance() {
        if (mInstance == null) {
            mInstance = new BallControlHelper();
        }
        return mInstance;
    }

    private final Lift mLift = Lift.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Wrist mWrist = Wrist.getInstance();
   // private final ClimbingHelper mClimbingHelper = ClimbingHelper.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,       
        PICKUPBALL,
        SHOOTBALLPOSITION,
        SHOOT,
        CARRYBALL,
       // CLIMB,
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
            synchronized (BallControlHelper.this) {
                mWantedState = SystemState.IDLE;
                mCurrentStateStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (BallControlHelper.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case PICKUPBALL:
                    newState = handlePickUpBall();
                    break;
                case SHOOTBALLPOSITION:
                    newState = handleShootBallPosition();
                    break;
                case SHOOT:
                    newState = handleShoot(timestamp);
                    break;
                case CARRYBALL:
                    newState = handleCarryBall();
                    break;
              //  case CLIMB: 
              //      newState = handleClimb();
              //      break;
                case HOME:
                    newState = handleHome();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("BallMachine state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            //    climbUpdater();
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
        }

        return defaultIdleTest();
    }


    //Only sets the position of the motors once per setpoint change, 
    //This allows for any jogging after a setpoint is set

    private PickUpHeight mWantedPickUpHeight = PickUpHeight.FLOOR;
    private PickUpHeight mCurrentPickUpHeight = PickUpHeight.FLOOR;
    boolean pickUpUpdate=false;

    private SystemState handlePickUpBall() {
       if(mStateChanged){
           mIntake.setWantedState(Intake.SystemState.PICKINGUP);
       }
       
       if(pickUpUpdate || mStateChanged){ // || mCurrentPickUpHeight != mWantedPickUpHeight){
           pickUpUpdate=false;
            mCurrentPickUpHeight=mWantedPickUpHeight;
            switch(mCurrentPickUpHeight){
                case FLOOR:
                    System.out.println("Ball lift and wrist pick up floor");
                        mLift.setPosition(Constants.kLiftPickUpFloor);
                        mWrist.setPosition(Constants.kWristPickUpFloor);
                        
                    break;
                case LOADING_STATION:
                        mLift.setPosition(Constants.kLiftPickUpLoadingStation);
                        mWrist.setPosition(Constants.kWristPickUpLoadingStation);
                break;
            }
       }
       
       if(mIntake.hasBall()){
           if(Constants.kCarryAfterPickUp){
               mWantedState=SystemState.CARRYBALL;
                return SystemState.CARRYBALL;
           }
           else return defaultIdleTest();
       }
       else return mWantedState;
    }
    

    private ShootHeight mWantedShootHeight = ShootHeight.CARGO_SHIP;
    private ShootHeight mCurrentShootHeight = ShootHeight.CARGO_SHIP;
    boolean shootPositionUpdate=false;

    private SystemState handleShootBallPosition() {
        if(mStateChanged){
            mIntake.setWantedState(Intake.SystemState.IDLE);
        }
        
        if(shootPositionUpdate || mStateChanged){  //} || mCurrentShootHeight != mWantedShootHeight){
            shootPositionUpdate=false;
            mCurrentShootHeight=mWantedShootHeight;
            switch(mCurrentShootHeight){
                case CARGO_SHIP:
                    mLift.setPosition(Constants.kLiftShootCargoShip);
                    mWrist.setPosition(Constants.kWristShootCargoShip);
                break;
                case ROCKET_ONE:
                    mLift.setPosition(Constants.kLiftShootRocketOne);
                    mWrist.setPosition(Constants.kWristShootRocketOne);
                break;
                case ROCKET_TWO:
                    mLift.setPosition(Constants.kLiftShootRocketTwo);
                    mWrist.setPosition(Constants.kWristShootRocketTwo);
                break;
            }
        }

        return mWantedState;
    }


    private CarryHeight mWantedCarryHeight = CarryHeight.LOW;
    private CarryHeight mCurrentCarryHeight = CarryHeight.LOW;
    boolean carryBallUpdate = false;

    private SystemState handleCarryBall() {
        if(mStateChanged){
            mIntake.setWantedState(Intake.SystemState.IDLE);
        }
 
        if(carryBallUpdate||mStateChanged){//} || mCurrentCarryHeight != mWantedCarryHeight){
            carryBallUpdate=false;
            mCurrentCarryHeight=mWantedCarryHeight;
            switch(mCurrentCarryHeight){
                case LOW:
                    mLift.setPosition(Constants.kLiftCarryLow);
                    mWrist.setPosition(Constants.kWristCarryLow);
                break;
                case MIDDLE:
                    mLift.setPosition(Constants.kLiftCarryMiddle);
                    mWrist.setPosition(Constants.kWristCarryMiddle);
                break;
            }
        }


        return mWantedState;
    }

  // private void climbUpdater(){
  //      if(mClimbingHelper.getClimbing()) {
  //          setWantedState(SystemState.CLIMB);
   //     }
  // }
   
   /* private SystemState handleClimb() {
        if(mStateChanged){
            mIntake.setWantedState(Intake.SystemState.IDLE);
            
        }
        

        switch(mWantedState){
            case CLIMB:
                return SystemState.CLIMB;
            case  IDLE:
            case  PICKUPBALL:
            case SHOOTBALLPOSITION:
            case SHOOT:
            case CARRYBALL:
            case HOME:
            default:
                if(mClimbingHelper.getClimbing()){
                    mClimbingHelper.setWantedState(ClimbingHelper.SystemState.IDLE);
                    return SystemState.CLIMB;
                }else{
                    return mWantedState;
                }
        }
    }*/


    private double startedAt=0;

    private SystemState handleShoot(double time) {
       if(mStateChanged){
           mIntake.setWantedState(Intake.SystemState.SHOOTING);
           startedAt=time;
       }

       if(!mIntake.hasBall()&& time-startedAt>=Constants.kCarryPauseAfterShoot){
           if(Constants.kCarryAfterShoot){
            mWantedState=SystemState.CARRYBALL;
                return SystemState.CARRYBALL;
           }
           else return defaultIdleTest();
       }else return mWantedState;

    }


    private SystemState handleHome() {
       if(mStateChanged){
           mLift.setWantedState(Lift.ControlState.HOMING);
           mWrist.setWantedState(Wrist.ControlState.HOMING);
       }

        //return mWantedState;
        mWantedState=BallControlHelper.SystemState.IDLE;
        return BallControlHelper.SystemState.IDLE;
    }

    //Set Mode Commands
        //Pick Up
            public enum PickUpHeight{
                LOADING_STATION,
                FLOOR
            }

            public void pickUp(PickUpHeight mode){
                mWantedState=SystemState.PICKUPBALL;
                mWantedPickUpHeight=mode;
                pickUpUpdate=true;
            }


        //ShootPostion
        public enum ShootHeight{
            CARGO_SHIP,
            ROCKET_ONE,
            ROCKET_TWO
        }

        public void shootPosition(ShootHeight mode){
            mWantedState=SystemState.SHOOTBALLPOSITION;
            mWantedShootHeight=mode;
            shootPositionUpdate=true;
        }
        //Carry
        public enum CarryHeight{
            MIDDLE,
            LOW
        }

        public void carry(CarryHeight mode){
            mWantedState=SystemState.CARRYBALL;
            mWantedCarryHeight=mode;
            carryBallUpdate=true;
        }


    //Jog Commands
        public void jogLift(double amount){
          /* if(!mClimbingHelper.getClimbing())*/ mLift.jog(amount);
           
        }

        public void jogWrist(double amount){
          /*  if(!mClimbingHelper.getClimbing()) */mWrist.jog(amount);
        }

  
    //BORRING Stupid stuff needed :(
   
        public synchronized void setWantedState(SystemState wantedState) {
            mWantedState = wantedState;
        }
    

        @Override
        public void outputToSmartDashboard() {
        
        }

        @Override
        public void stop() {
            mLift.setWantedState(Lift.ControlState.IDLE);
            mWrist.setWantedState(Wrist.ControlState.IDLE);
            mIntake.setWantedState(Intake.SystemState.IDLE);
        }

        @Override
        public void zeroSensors() {

        }

        @Override
        public void registerEnabledLoops(Looper enabledLooper) {
            enabledLooper.register(mLoop);
        }

}
