package frc.robot.subsystems.Inputs;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Lights extends SubsystemBase{

    public final boolean disabled = false;
    
    RobotContainer r;
    InputCal cal;

    AddressableLED led;

    AddressableLEDBuffer ledBuffer;
    PowerDistribution pdh;
    boolean switchableChannel = true;

    public Lights(RobotContainer r, InputCal cal){
        this.r = r;
        this.cal = cal;
        if(disabled) return;

        initOutputLed();
        ledOutEnable(true);

        led = new AddressableLED(cal.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(cal.BUFFER_LENGTH);

        led.setLength(ledBuffer.getLength());
        led.start();

        pdh = new PowerDistribution(1, ModuleType.kRev);
    }

    public void underglow(boolean on){
        if(on != switchableChannel){
            pdh.setSwitchableChannel(on);
            switchableChannel = on;
        }
    }

    int offset = 0;
    double switchTime = 0;
    @Override
    public void periodic(){
        if(disabled) return;

        underglow((DriverStation.isFMSAttached() || DriverStation.isEnabled()) && r.inputs.getFieldMode());
        
        if(r.inputs.balanceMode.getAsBoolean()){
            if(Timer.getFPGATimestamp() > switchTime){
                for(int i = 0; i + 2 < ledBuffer.getLength(); i += 3){
                    int offsetWrapper = (i + offset) % ledBuffer.getLength();
                    int offsetPlusOne = (i + offset + 1) % ledBuffer.getLength();
                    int offsetPlusTwo = (i + offset + 2) % ledBuffer.getLength();
                    ledBuffer.setLED(offsetWrapper, new Color(0, 255, 255));
                    ledBuffer.setLED(offsetPlusOne, new Color(0, 0, 255));
                    ledBuffer.setLED(offsetPlusTwo, new Color(255, 255, 255));
                }
                offset++;
                switchTime = Timer.getFPGATimestamp() + 0.3;
            }if(r.inputs.alignMode.getAsBoolean()){
            if(Timer.getFPGATimestamp() > switchTime){
                for(int i = 0; i + 2 < ledBuffer.getLength(); i += 3){
                    int offsetWrapper = (i + offset) % ledBuffer.getLength();
                    int offsetPlusOne = (i + offset + 1) % ledBuffer.getLength();
                    int offsetPlusTwo = (i + offset + 2) % ledBuffer.getLength();
                    ledBuffer.setLED(offsetWrapper, new Color(0, 255, 255));
                    ledBuffer.setLED(offsetPlusOne, new Color(0, 0, 255));
                    ledBuffer.setLED(offsetPlusTwo, new Color(255, 255, 255));
                }
                offset++;
                switchTime = Timer.getFPGATimestamp() + 0.3;
            }
}

        } else if(r.inputs.parkMode.getAsBoolean()){
            Color colorSet = new Color(255, 0, 0);
            for(int i = 0; i < ledBuffer.getLength(); i++){
                ledBuffer.setLED(i, colorSet);
            }
        } else{
            switchTime = Timer.getFPGATimestamp();
            Color colorSet;
            /*if(r.inputs.alignMode.getAsBoolean()){
                colorSet = new Color(170, 0, 255);
            }else{
                colorSet = new Color(255, 50, 0);
            }*/
            if(r.inputs.isCube()){
                colorSet = new Color(255, 0, 255);
            } else {
                colorSet = new Color(255, 100, 0);
            }
            for(int i = 0; i < ledBuffer.getLength(); i++){
                ledBuffer.setLED(i, colorSet);
            }
        }

        if(led != null){
            led.setData(ledBuffer);
        }
    }

    public void skittles()
        {
            if(Timer.getFPGATimestamp() > switchTime){
                for(int i = 0; i + 2 < ledBuffer.getLength(); i += 5){
                    int offsetWrapper = (i + offset) % ledBuffer.getLength();
                    int offsetPlusOne = (i + offset + 1) % ledBuffer.getLength();
                    int offsetPlusTwo = (i + offset + 2) % ledBuffer.getLength();
                    int offsetPlusThree = (i + offset + 3) % ledBuffer.getLength();
                    int offsetPlusFour = (i + offset + 4) % ledBuffer.getLength();
                    int offsetPlusFive = (i + offset + 5) % ledBuffer.getLength();
                    ledBuffer.setLED(offsetWrapper, new Color(255, 0, 0));
                    ledBuffer.setLED(offsetPlusOne, new Color(255, 165, 0));
                    ledBuffer.setLED(offsetPlusTwo, new Color(255, 255, 0));
                    ledBuffer.setLED(offsetPlusThree, new Color(0, 255, 0));
                    ledBuffer.setLED(offsetPlusFour, new Color(0, 0, 255));
                    ledBuffer.setLED(offsetPlusFive, new Color(255, 0, 255));
                }
                offset++;
                switchTime = Timer.getFPGATimestamp() + 0.3;
            }
    }

    BooleanPublisher ledEnable;
    IntegerPublisher ledValue;
    int localLedValue = 0;
    public void initOutputLed(){
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable table = nt.getTable("ControlBoard");
        ledEnable = table.getBooleanTopic("LED_Enable").publish();
        ledValue = table.getIntegerTopic("LED_Output").publish();
    }

    public void ledOutEnable(boolean b){
        ledEnable.set(b);
    }

    public void ledOutputSet(int value, boolean on){
        int v = 1 << value;
        if(on){
            localLedValue |= v;
        } else {
            localLedValue &= ~v;
        }
        ledValue.set(localLedValue);
    }
}
