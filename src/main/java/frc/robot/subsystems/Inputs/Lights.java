package frc.robot.subsystems.Inputs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Inputs.Inputs.Level;

public class Lights extends SubsystemBase{

    public final boolean disabled = false;
    
    RobotContainer r;
    InputCal cal;

    AddressableLED led;

    AddressableLEDBuffer ledBuffer;
    public PowerDistribution pdh;
    boolean switchableChannel = true;

    Color[] rainbow = {
        new Color(255,0,0),
        new Color(255,127,0),
        new Color(255,255,0),
        new Color(0,255,0),
        new Color(0,0,255),
        new Color(75,0,130),
        new Color(148,0,211)
    };

    Color[] thanos = {
        new Color(5, 0, 10),
        new Color(15, 0, 33),
        new Color(25, 0, 45),
        new Color(50, 0, 100),
        new Color(75, 0, 125),
        new Color(100, 0, 150),
        new Color(255, 0, 255),
        new Color(255, 50, 255),
        new Color(255, 75, 255),
    };

    Color[] chowder ={
        new Color (255, 0, 255),
        new Color (255, 75, 255),
        new Color (175, 0, 190)
    };

    Color[] spongebob = {
        new Color (0, 0, 0),
        new Color (10, 3, 0),
        new Color (25, 7, 0),
        new Color (50, 20, 0),
        new Color (75, 30, 0),
        new Color (150, 40, 0),
        new Color (200, 75, 0),
        new Color (200, 75, 0),
        new Color (225, 80, 0),
        new Color (255, 100, 0),
        new Color (255, 100, 0),
    };

    Color[] banana = {
        new Color (255, 100, 0),
        new Color (200, 75, 0),
        new Color (150, 40, 0)
    };

    Color[] crabRave = {
        new Color (255, 0 ,0),
        new Color (100, 0, 0),
        new Color (255, 0, 0),
        new Color (75, 0, 0),
        new Color (255, 0, 0),
        new Color (50, 0, 0)
    };

    Color[] snowBall = {
        new Color (0, 255, 255),
        new Color (0, 0, 255),
        new Color (255, 255, 255)
    };

    Color[] wednesday = {
        new Color (200, 0, 100),
        new Color (175, 0, 75),
        new Color (150, 0, 60),
        new Color (100, 0, 40),
        new Color (75, 0, 10)
    };

    Color[] blueStripes = {
        new Color (0, 0, 200),
        new Color (0, 0, 175),
        new Color (0, 0, 150),
        new Color (0, 0, 100),
        new Color (0, 0, 50),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0)
    };

    Color[] redStripes = {
        new Color (200, 0, 0),
        new Color (175, 0, 0),
        new Color (150, 0, 0),
        new Color (100, 0, 0),
        new Color (50, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
    };

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

        //dont underglow when disabled in the pits
        underglow((DriverStation.isFMSAttached() || DriverStation.isEnabled()) && r.inputs.getFieldMode());
        
        if(DriverStation.isDisabled()){
            //disabled
            if(r.inputs.selectedLevel == Level.NONE){
                if(DriverStation.getAlliance() == Alliance.Blue){
                    skittles2(blueStripes, false);
                } else {
                    skittles2(redStripes, false);
                }
            } else{
                testMode();
            }
        } else if(DriverStation.isAutonomous()){
            //auton
            skittles2(snowBall, true);
        } else if(r.inputs.parkMode.getAsBoolean()){
            //parked
            skittles2(crabRave, false);
        } else if(r.inputs.balanceMode.getAsBoolean()){
            //balance
            skittles2(snowBall, true);
        } else if(r.inputs.alignMode.getAsBoolean()){
            //manual align
            skittles2(snowBall, false);
        } else if(r.inputs.isCube() && r.inputs.isShelf()){
            //shelf cube
            skittles2(thanos, true);
        } else if(r.inputs. isCube()){
            //floor cube
            skittles2(chowder, false);
        } else if (r.inputs.isCube() == false && r.inputs.isShelf())  {
            //shelf cone
            skittles2(spongebob, true);
        } else {
            //floor cone
            skittles2(banana, false);
        }

        if(led != null){
            led.setData(ledBuffer);
        }
    }
    

    public void testMode(){
        switch(r.inputs.buttonAssignment){
            case 1:
            case 2:
                skittles2(rainbow, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 3:
            case 4:
                skittles2(crabRave, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 5:
            case 6:
                skittles2(snowBall, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 7:
            case 8:
                skittles2(spongebob, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 9:
            case 10:
                skittles2(banana, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 11:
            case 12:
                skittles2(thanos, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 13:
            case 14:
                skittles2(chowder, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 15:
            case 16:
                skittles2(wednesday, r.inputs.buttonAssignment % 2 > 0);
                break;
            default:
                skittles2(rainbow, false);
        }
    }

    public void skittles(){
        if(Timer.getFPGATimestamp() > switchTime){
            int len = ledBuffer.getLength() / 2;
            for(int i = 0; i < len; i += 18){
                int offsetWrapper = (i + offset) % len;
                int offsetPlusOne = (i + offset + 1) % len;
                int offsetPlusTwo = (i + offset + 2) % len;
                int offsetPlusThree = (i + offset + 3) % len;
                int offsetPlusFour = (i + offset + 4) % len;
                int offsetPlusFive = (i + offset + 5) % len;
                int offsetPlusSix = (i + offset + 6) % len;
                int offsetPlusSeven = (i + offset + 7) % len;
                int offsetPlusEight = (i + offset + 8) % len;
                int offsetPlusNine = (i + offset + 9) % len;
                int offsetPlusTen = (i + offset + 10) % len;
                int offsetPlusEleven = (i + offset + 11) % len;
                int offsetPlusTwelve = (i + offset + 12) % len;
                int offsetPlusThirteen = (i + offset + 13) % len;
                int offsetPlusFourteen = (i + offset + 14) % len;
                int offsetPlusFifteen = (i + offset + 15) % len;
                int offsetPlusSixteen = (i + offset + 16) % len;
                int offsetPlusSeventeen = (i + offset + 17) % len;
                mirrorLed(offsetWrapper, new Color(255, 0, 0));
                mirrorLed(offsetPlusOne, new Color(255, 0, 0));
                mirrorLed(offsetPlusTwo, new Color(255, 0, 0));
                mirrorLed(offsetPlusThree, new Color(255, 165, 0));
                mirrorLed(offsetPlusFour, new Color(255, 165, 0));
                mirrorLed(offsetPlusFive, new Color(255, 165, 0));
                mirrorLed(offsetPlusSix, new Color(255, 255, 0));
                mirrorLed(offsetPlusSeven, new Color(255, 255, 0));
                mirrorLed(offsetPlusEight, new Color(255, 255, 0));
                mirrorLed(offsetPlusNine, new Color(0, 255, 0));
                mirrorLed(offsetPlusTen, new Color(0, 255, 0));
                mirrorLed(offsetPlusEleven, new Color(0, 255, 0));
                mirrorLed(offsetPlusTwelve, new Color(0, 0, 255));
                mirrorLed(offsetPlusThirteen, new Color(0, 0, 255));
                mirrorLed(offsetPlusFourteen, new Color(0, 0, 255));
                mirrorLed(offsetPlusFifteen, new Color(255, 0, 255));
                mirrorLed(offsetPlusSixteen, new Color(255, 0, 255));
                mirrorLed(offsetPlusSeventeen, new Color(255, 0, 255));
            }
            offset++;
            switchTime = Timer.getFPGATimestamp() + 0.5;
        }
    }

    public void skittles2(Color[] colors, boolean up){
        if(Timer.getFPGATimestamp() > switchTime){
            int len = ledBuffer.getLength() / 2;
            for(int i = 0; i < len; i++){
                int idx = (i + offset) % len;
                if(idx < 0) idx += len;
                int colorIdx = ((i) % (colors.length * 2)) / 2;
                if(colorIdx < 0) colorIdx += colors.length;
                mirrorLed(idx, colors[colorIdx]);
            }
            if (up){
                offset++;
            } else {
                offset--;
            }
            switchTime = Timer.getFPGATimestamp() + 0.03;
        }
    }
    

    public void mirrorLed(int index, Color c){
        ledBuffer.setLED(index, c);
        ledBuffer.setLED(ledBuffer.getLength() - index - 1, c);
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