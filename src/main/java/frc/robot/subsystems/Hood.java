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
 *jkladfslfkjsdjlsdf,.mm.,fadm,.s
 */

public class Hood extends Subsystem {
    
   

    private static Hood sInstance = null;

    public static Hood getInstance() {
        if (sInstance == null) {
            sInstance = new Hood();
        }
        return sInstance;
    }

    //JAMESON KADEN - linear actuator definition

    public Hood() {
        
         //JAMESON KADEN - Linear Actuator Initialization 

 
    }

    public enum SystemState {
        IDLE,
        MOVING
    }

    private SystemState mSystemState = SystemState.IDLE;
    

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Hood.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
                System.out.println("Hood started");
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Hood.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case MOVING:
                    newState = handleMoving(timestamp);
                    break;      
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Hood state " + mSystemState + " to " + newState);
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


   

    private SystemState handleIdle() {
        if(mStateChanged){  
          
        }
     
        return mSystemState;
    }

    private SystemState handleMoving(double now){
        if(mStateChanged){
            
        }

       //JAMESON KADEN figure out a way to know how long it will take to make the move, keep track of how far it has to go

        return //JAMESON KADEN make it only return IDLE when its done moving, if not, return moving
    }




 

   
    //JAMESON KADEN make methods to set the pwm value (0 to 1) as well as one to set the actual distance extended
    //JAMESON KADEN using math make a method to set the hood to a launch angle (parameter is an angle from the ground)
    //JAMESON KADEN place constants in the Constants.java file under the Hood section
    //JAMESON KADEN make it so that when the set position is different than current, system state switches to moving automatically
    public synchronized void setMotor(double s){ 
        
        
    }

    
 

    //Boring Stuff

        private void stopMotor(){
           
        }


       

        @Override
        public void outputToSmartDashboard() {
            // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kIntakeSensorGearReduction);
        }

        @Override
        public void stop() {
            mSystemState=SystemState.IDLE;
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
            System.out.println("Testing HOOD.-----------------------------------");
            boolean failure=false;       

            //JAMESON KADEN make a test to make sure the hood works correctly
            //JAMESON KADEN add print outs to mark what is happening for each step
            //JAMESON KADEN feel free to use Timer.delay(amount) to add delays

            return !failure;
        }

}