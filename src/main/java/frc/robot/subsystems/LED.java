package frc.robot.subsystems;

import frc.lib.util.drivers.M_I2C;

public class LED{

M_I2C i2c;

private static LED mInstance = new LED();

public static LED getInstance() {
    return mInstance;
}

public LED(){
    i2c = new M_I2C();
}

public enum LedMode{IDLE,SHOOT};

public void setLED(LedMode m){
//LARRY - setting led status 
    if(m==LedMode.IDLE){
        i2c.write("IDLE");
    }else if(m==LedMode.SHOOT){
        i2c.write("SHOOT");
    }
}

}