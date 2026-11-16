package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightSys {

    public enum LightState{
        intaking,
        aiming,
        on_target,
        FULL_POWER,
        MID_POWER,
        LOW_POWER,
        FULL_intaking,
        MID_intaking,
        LOW_intaking
    }

    LightState currentState;
    RevBlinkinLedDriver lights;

    public void init(HardwareMap hwmap){
        lights = hwmap.get(RevBlinkinLedDriver.class,"blinken");
        currentState = LightState.FULL_POWER;
    }

    public void setLightState(LightState lightState){
        currentState = lightState;
    }

    public void updateLights(){
        if(currentState == LightState.intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        }else if(currentState == LightState.FULL_POWER){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }else if(currentState == LightState.MID_POWER){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else if(currentState == LightState.LOW_POWER){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }else if(currentState == LightState.LOW_intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE);
        }else if(currentState == LightState.MID_intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
        }else if(currentState == LightState.FULL_intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }else if(currentState == LightState.on_target){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }else if(currentState == LightState.aiming){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }

    }
}
