package frc.robot.subsystems;


import frc.lib.util.drivers.Talon.CANTalonFactory;
import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.NidecBrushless;

/**
 * The suspension subsystem consists of dynamo motors that are meant to send one ball at a time into the shooter.
 * There are ir sensors placed before and after the suspension to sense blockage or emptiness of the hopper.
 * The main things this subsystem has to are feed fuel and unjam
 * 
 *y
 */
public class Flywheel extends Subsystem {
    
   

    private static Flywheel sInstance = null;

    public static Flywheel getInstance() {
        if (sInstance == null) {
            sInstance = new Flywheel();
        }
        return sInstance;
    }

    private TalonSRX mMotor;

    public Flywheel() {
        
        //Talon Initialization 
       
        
       mMotor = CANTalonFactory.createTalon(Constants.kShooterId, false, NeutralMode.Coast, FeedbackDevice.QuadEncoder, 0, true);
        
       //mMasterSrx.changeControlMode(TalonControlMode.Voltage);
    
       
       
       mMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
       mMotor.configVelocityMeasurementWindow(32);
       
       mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 2);
       mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 2);
  
     
    }


    public enum WantedState {
        IDLE,
        SHOOT
    }


    public enum SystemState {
        IDLE,
        SPINUP,
        SHOOTING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Flywheel.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
                System.out.println("Flywheel started");
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Flywheel.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case SPINUP:
                    newState = handleSpinup();
                    break;             
                case SHOOTING: 
                    newState = handleShooting();
                    break;
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Flywheel state " + mSystemState + " to " + newState);
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
            stopMotor();
        }
        
        switch(mWantedState){
            case SHOOT:
                return SystemState.SPINUP;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleSpinup(){
        if(mStateChanged){
        
            setMotor(1);
        }



        switch(mWantedState){
            case IDLE:
                return SystemState.IDLE;
            case SHOOT:
                if(getVelocity()>=Constants.kShooterMinVelocity){
                    return SystemState.SHOOTING;
                }
            default:
                return SystemState.SPINUP;
        }

    }

    private SystemState handleShooting(){



        switch(mWantedState){
            case IDLE:
                return SystemState.IDLE;
            default:
                return SystemState.SHOOTING;
        }

    }



    
    public synchronized void setMotor(double s){
       mMotor.set(ControlMode.PercentOutput,s);
    }

    public synchronized double getVelocity(){
        return mMotor.getSelectedSensorVelocity();
    }

    public synchronized void setWantedState(WantedState s){
        mWantedState=s;
    }
   
    public synchronized SystemState getSystemState(){
        return mSystemState;
    }
    //Boring Stuff

        private void stopMotor(){
         mMotor.set(ControlMode.Disabled,0);
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