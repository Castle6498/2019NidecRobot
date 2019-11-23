package frc.lib.util.drivers.Talon;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * Creates TalonSRX objects and configures all the parameters we care about
 * to factory defaults. Closed-loop and sensor parameters are not set, as these
 * are expected to be set by the application.
 */
public class CANTalonFactory {

  

   
 /*// Create a TalonSRX with the default (out of the box) configuration.
    public static TalonSRX createDefaultTalon(int id) {
        
        return createTalon(id, new Configuration());
        
    }
*/
   

    public static TalonSRX createTalon(int id, boolean inverted, NeutralMode neutralMode, 
    FeedbackDevice device, int Controlid, boolean reverseSensor) {
        TalonSRX talon = new TalonSRX(id);
               
        talon.clearStickyFaults();
       
        talon.setInverted(inverted);
        talon.setNeutralMode(neutralMode);

        talon.configSelectedFeedbackSensor(device, Controlid, 0);
        talon.setSensorPhase(reverseSensor);

        return talon;
    }

    public static TalonSRX setupHardLimits(TalonSRX talon, LimitSwitchSource forwardSource, 
    LimitSwitchNormal forwardNormal, boolean clearPosOnForward, LimitSwitchSource reverseSource, 
    LimitSwitchNormal reverseNormal, boolean clearPosOnReverse){

        talon.configForwardLimitSwitchSource(forwardSource, forwardNormal);
        talon.configReverseLimitSwitchSource(reverseSource, reverseNormal);

        talon.configClearPositionOnLimitF(clearPosOnForward, 0);
        talon.configClearPositionOnLimitR(clearPosOnReverse, 0);

       return talon; 
    }
   
    public static TalonSRX setupSoftLimits(TalonSRX talon, boolean forwardEnable, int forwardThreshold,
    boolean reverseEnable, int reverseThreshold){

        talon.configForwardSoftLimitThreshold(forwardThreshold);
        talon.configReverseSoftLimitThreshold(reverseThreshold);

        talon.configForwardSoftLimitEnable(forwardEnable);
        talon.configReverseSoftLimitEnable(reverseEnable);

        return talon;
    }

    public static TalonSRX tuneLoops(TalonSRX talon, int id, double P, double I, double D, double F){

        talon.config_kP(id, P);
        talon.config_kI(id, I);
        talon.config_kD(id, D);
        talon.config_kF(id, F);
        

        return talon;
    }
}