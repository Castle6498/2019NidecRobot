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
import edu.wpi.first.wpilibj.Servo;

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
    Servo actuator;

    public Hood() {
        
         //JAMESON KADEN - Linear Actuator Initialization 
        actuator = new Servo(Constants.kHoodLinearActuatorPort);
        actuator.setBounds(2.0,1.8,1.5,1.2,1.0);
       
 
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

           
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };


   private boolean onPosition=true;

    private SystemState handleIdle() {
        if(mStateChanged){  
          onPosition=true;
        }
     
        return mSystemState;
    }



    double timeRequired=0;
    double startTime=0;
    private SystemState handleMoving(double now){
        if(mStateChanged||positionUpdate){
            startTime=now;
            timeRequired =delta/Constants.kHoodSpeed; //(7mm/s)
            positionUpdate=false;
        }

       //JAMESON KADEN figure out a way to know how long it will take to make the move, keep track of how far it has to go
       //JAMESON KADEN make it only return IDLE when its done moving, if not, return moving
        if(now>=startTime+timeRequired){
            return SystemState.IDLE;
        }else{
            onPosition=false;
            return SystemState.MOVING;
        }

      
    }




 

   
    //JAMESON KADEN make methods to set the pwm value (0 to 1) as well as one to set the actual distance extended
    //JAMESON KADEN using math make a method to set the hood to a launch angle (parameter is an angle from the ground)
    //JAMESON KADEN place constants in the Constants.java file under the Hood section
    //JAMESON KADEN make it so that when the set position is different than current, system state switches to moving automatically
    public synchronized void setRaw(double s){ 
        actuator.setSpeed(s);
    }


    double setPoint = 0;
    double delta = 0 ;
    boolean positionUpdate=false;

    /**
     * @param d Distance in mm
     */
    public synchronized void setDistance(double d){ 
        //140 mm stroke total
        if(d!=setPoint){
        setRaw((2/Constants.kHoodStroke)*d-1); //(raw value/mm)*distance-1 (to make 0 a -1)
            delta = Math.abs(d-setPoint);
            setPoint = d;
            mSystemState=SystemState.MOVING;
            positionUpdate=true;
        }
    }

    /**
     * 
     * @param a Angle (degrees) of elevation max of 66.485
     */
    public synchronized void setAngle(double a){
        double angle = Math.max(Math.min(66.485,a),0); //limit between 0 and max angle
        setDistance(140-120.65*(Math.PI/180)*angle); //max stroke - radius(pi/180)*angle of elevation
    }
    
    public boolean onPosition(){
        return onPosition;
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