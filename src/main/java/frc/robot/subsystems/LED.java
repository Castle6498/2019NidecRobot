
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;

import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 * The LED subsystem consists of the green ring light on the front of the robot used for aiming and communicating
 * information with the drivers (when a gear is picked up, when the robot loses connection) and the blue "range finding"
 * LED strip on the back of the robot used for signaling to the human player. The main things this subsystem has to do
 * is turn each LED on, off, or make it blink.
 * 
 * @see Subsystem.java
 */
/*
public class LED extends Subsystem {
    public static final int kDefaultBlinkCount = 4;
    public static final double kDefaultBlinkDuration = 0.2; // seconds for full cycle
    private static final double kDefaultTotalBlinkDuration = kDefaultBlinkCount * kDefaultBlinkDuration;

    private static LED mInstance = null;

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }


     // Internal state of the system
     public enum SystemState {
        OFF, FIXED_ON, BLINKING
    }


    enum Color {Blue, Green, Red, Off}

    private class LEDController{
        DigitalOutput mRed, mGreen, mBlue;

         public boolean mIsBlinking = false;

         public boolean mIsOn=false;

         public double mBlinkDuration;
         public int mBlinkCount;
         public double mTotalBlinkDuration;

         private SystemState mSystemState = SystemState.OFF;
         private SystemState mWantedState = SystemState.OFF;

         public Color mWantedColor=Color.Off;


        public LEDController(int blue, int red, int green){
            mRed = new DigitalOutput(red);
            mBlue = new DigitalOutput(blue);
            mGreen = new DigitalOutput(green);


            reset();
        }

        public void reset(){
            mIsOn=false;
            setOff();
            mIsBlinking=false;

            mSystemState = SystemState.OFF;
            mWantedState = SystemState.OFF;
        }

        public void setColor(Color color){
            switch(color){
                case Red:
                setRed();
                break;
                case Green:
                setGreen();
                break;
                case Blue:
                setBlue();
                break;
                case Off:
                setOff();
                break;
            }
        }

        public void setRed(){
            setRGB(true,false,false);
        }

        public void setGreen(){
            setRGB(false,true,false);
        }

        public void setBlue(){
            setRGB(false,false,true);
        }

        public void setOff(){
            setRGB(false, false, false);
        }

        public void setRGB(boolean red, boolean green, boolean blue){
            if(red||blue||green)mIsOn=true;
            else mIsOn=false;
            mRed.set(red);
            mGreen.set(green);
            mBlue.set(blue);
        }

        public boolean isOn(){
            return mIsOn;
        }

        public void configureBlink(double duration, int count){
            mBlinkCount=count;
            mBlinkDuration=duration;
            mTotalBlinkDuration=count*duration;
            mIsBlinking=true;
        }
    }


    
    private LEDController mPlateLED;
   

    public LED() {
        mPlateLED = new LEDController(Constants.kPlateLEDPorts[0], Constants.kPlateLEDPorts[1], Constants.kPlateLEDPorts[2]);
        
        mPlateLED.configureBlink(kDefaultBlinkDuration, kDefaultBlinkCount);    



    }

    private Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;

        @Override
        public void onStart(double timestamp) {
            synchronized (LED.this) {

                mPlateLED.reset();
                



            }

            mCurrentStateStartTime = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (LED.this) {
                SystemState newPlateState;
                double timeInState = timestamp - mCurrentStateStartTime;
                switch (mPlateLED.mSystemState) {
                case BLINKING:
                newPlateState = handleBlinking(mPlateLED, timeInState);
                    break;
                case FIXED_ON:
                newPlateState = handleFixedOn(mPlateLED);
                    break;
                default:
                    System.out.println("Fell through on LED states!!");
                    newPlateState = SystemState.OFF;
                }
                if (newPlateState != mPlateLED.mSystemState) {
                    System.out.println("LED state " + mPlateLED.mSystemState + " to " + newPlateState);
                    mPlateLED.mSystemState = newPlateState;
                    mCurrentStateStartTime = timestamp;
                }









            }
        }

        @Override
        public void onStop(double timestamp) {
            mPlateLED.reset();
        }
    };

    

    private synchronized SystemState handleFixedOn(LEDController led) {
        led.setColor(led.mWantedColor);

        return led.mWantedState;
    }


    private synchronized SystemState handleBlinking(LEDController led, double timeInState) {
        if (timeInState > led.mTotalBlinkDuration) {
            led.setOff();
            // Transition to OFF state and clear wanted state.
            led.mWantedState = SystemState.OFF;
            return SystemState.OFF;
        }

        int cycleNum = (int) (timeInState / (led.mBlinkDuration / 2.0));
        if ((cycleNum % 2) == 0) {
            led.setColor(led.mWantedColor);
        } else {
            led.setOff();
        }
        return SystemState.BLINKING;
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    public synchronized void configureBlink(int blinkCount, double blinkDuration) {
        mBlinkDuration = blinkDuration;
        mBlinkCount = blinkCount;
        mTotalBlinkDuration = mBlinkCount * mBlinkDuration;
    }
}*/