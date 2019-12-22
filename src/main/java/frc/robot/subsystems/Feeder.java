package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;



import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.NidecBrushless;

/**
 * The suspension subsystem consists of dynamo motors that are meant to send one ball at a time into the shooter.
 * There are ir sensors placed before and after the suspension to sense blockage or emptiness of the hopper.
 * The main things this subsystem has to are feed fuel and unjam
 * 
 *y
 */
public class Feeder extends Subsystem {
    
   

    private static Feeder sInstance = null;

    public static Feeder getInstance() {
        if (sInstance == null) {
            sInstance = new Feeder();
        }
        return sInstance;
    }

    private NidecBrushless mLeftMotor, mRightMotor; 
    //private DigitalInput mPhotoeye;

    public Feeder() {
        
        //Nidec Initialization 
        //LINDA - initialize mLeftMotor and mRightMotor based on the ports defined in the Constants.java (Constants.kFeeder....)
        mLeftMotor = new NidecBrushless(Constants.kFeederLeftPWM, Constants.kFeederLeftDIO);
        mRightMotor = new NidecBrushless(Constants.kFeederRightPWM, Constants.kFeederRightDIO);

        mLeftMotor.enable();
        mRightMotor.enable();
        
        //Photoeye Initialization
        //mPhotoeye=new DigitalInput(Constants.kFeederSensorPort);
  
     
    }

    public enum SystemState {
        IDLE,
        FEEDING
    }

    private SystemState mSystemState = SystemState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Feeder.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
                System.out.println("Feeder started");
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Feeder.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case FEEDING:
                    newState = handleFeeding(timestamp);
                    break;             
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Feeder state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
           // ballUpdate(timestamp);
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    

    private SystemState handleIdle() {
        if(mStateChanged){  
            stopMotor();
        }
        
        return mSystemState;
    }

    private SystemState handleFeeding(double now){
        if(mStateChanged){
            //setMotor(a constant from constants); LINDA
            setMotor(Constants.kFeederSpeed);
        }

        return mSystemState;
    }


/*
    //Ball Handling for Sensor ----------------
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

            if(ballSeenTime-ballStartSeenTime>=Constants.kFeederBallRequiredTime){
                mHasBall=true;
            }
        }

   */
    
    public synchronized void setMotor(double s){
       //LINDA be able to set both left and right motors, one is in the opposite direction (-s)
        
       if (s==0){ //Disabling them prevents the whine, but stops them from holding position
           mLeftMotor.disable();
           mRightMotor.disable();
       }else{
       mLeftMotor.set(s);
        mRightMotor.set(-s);
       }
    }

    public synchronized void setSystemState(SystemState s){
        mSystemState=s;
    }
   

    //Boring Stuff

        private void stopMotor(){
          //LINDA - set them to 0
            mLeftMotor.set(0);
            mRightMotor.set(0);
        }


        

        @Override
        public void outputToSmartDashboard() {
            // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kIntakeSensorGearReduction);
        }

        @Override
        public void stop() {
            mSystemState = SystemState.IDLE;
            stopMotor();
        }

        @Override
        public void zeroSensors() {
        }


        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

        public boolean checkSystem(double timestamp) {
            System.out.println("Testing Feeder.-----------------------------------");
            boolean failure=false;       

            //LINDA - make a test that turns on left motor, then right motor each for a set time
            //feel free to use Timer.delay(amount) to add delays to account for spin up and spin down time
            //Include printouts that report each step in the process
            return !failure;
        }

}