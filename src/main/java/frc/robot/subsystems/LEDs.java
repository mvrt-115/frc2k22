// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.TurretState;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  public AddressableLED led, turretLed;
  public AddressableLEDBuffer lBuffer, turretBuffer;

  public final int LED_PORT = 0;
  public final int TURRET_LED_PORT = 1;
  public final int LED_LENGTH = 56;
  public final int TURRET_LED_LENGTH = 39;
  public int counter = 0;

  public enum LedState {CLIMBER, TURRET_ALIGNED, TURRET_UNALIGNED, SHOOTING, DEFAULT};
  public LedState prevState = LedState.DEFAULT;
  public LedState currState = LedState.DEFAULT;

  public LEDs() 
  {
    led = new AddressableLED(LED_PORT);
    lBuffer = new AddressableLEDBuffer(LED_LENGTH);

    led.setLength(LED_LENGTH);
    led.start();

    turretLed = new AddressableLED(TURRET_LED_PORT);
    turretBuffer = new AddressableLEDBuffer(TURRET_LED_LENGTH);

    turretLed.setLength(TURRET_LED_LENGTH);
    turretLed.start();

    setGradient(0, TURRET_LED_LENGTH/3, Color.kBlack, Color.kGreen);
    setGradient(TURRET_LED_LENGTH/3, (TURRET_LED_LENGTH*2)/3, Color.kBlack, Color.kGreen);
    setGradient((TURRET_LED_LENGTH*2)/3, TURRET_LED_LENGTH, Color.kBlack, Color.kGreen);
  }

  public Color RBGToColor(int[] values) 
  {
    return new Color(values[0]/255.0, values[1]/255.0, values[2]/255.0);
  }

  public void setLEDBulb(int index, Color color) 
  {
    lBuffer.setLED(index, color);
  }

  public void setLEDBulbTurret(int index, Color color) 
  {
    turretBuffer.setLED(index, color);
  }

  public Color getLEDColor(int index)
  {
    return lBuffer.getLED(index);
  }

  public Color getLEDColorTurret(int index)
  {
    return turretBuffer.getLED(index);
  }

  public void sendData() 
  {
    led.setData(lBuffer);
  }

  public void sendDataTurret() 
  {
    turretLed.setData(turretBuffer);
  }

  /**
   * 
   * @param startIndex first led you want lit
   * @param endIndex last led you want lit (includes this led)
   * @param color color for all of the block
   */
  public void setSingleBlock(int startIndex, int endIndex, Color color) 
  {
    if(startIndex >= 0 && endIndex < LED_LENGTH) {
      for(int i = startIndex; i <= endIndex; i++)
      {
        lBuffer.setLED(i, color);
      }
    }
  }

  public void setSingleBlockTurret(int startIndex, int endIndex, Color color) 
  {
    if(startIndex >= 0 && endIndex < LED_LENGTH) {
      for(int i = startIndex; i <= endIndex; i++)
      {
        turretBuffer.setLED(i, color);
      }
    }
  }

  public void setFullLength(Color color)
  {
    setSingleBlock(0, LED_LENGTH-1, color);
  }

  public void setFullLengthTurret(Color color)
  {
    setSingleBlockTurret(0, LED_LENGTH-1, color);
  }


  //DIDN'T WORK
  public void moveUp(int startIndex, int endIndex, double delay)
  {
    Color tempColor = lBuffer.getLED(endIndex);
    
    for(int i = endIndex; i > startIndex; i--)
    {
      setLEDBulb(i, lBuffer.getLED(i-1));
    }

    setLEDBulb(0, tempColor);

    sendData();
    Timer.delay(delay);
  }

  public void moveUpTurret(int startIndex, int endIndex, double delay)
  {
    Color tempColor = lBuffer.getLED(endIndex);
    
    for(int i = endIndex; i > startIndex; i--)
    {
      setLEDBulbTurret(i, lBuffer.getLED(i-1));
    }

    setLEDBulbTurret(0, tempColor);

    sendDataTurret();
    Timer.delay(delay);
  }

  //DIDN'T WORK
  public void moveDown(int startIndex, int endIndex, double delay)
  {
    Color tempColor = lBuffer.getLED(0);
    
    for(int i = startIndex; i < endIndex; i++)
    {
      setLEDBulb(i, lBuffer.getLED(i+1));
    }

    setLEDBulb(endIndex, tempColor);

    sendData();
    Timer.delay(delay);
  }

  public void setWave(int startIndex, int endIndex, int waveLength, Color darkColor, Color lightColor)
  {
    for(int i = startIndex; i <= endIndex; i += waveLength)
    {
      setGradientOnTwoSides(i, i + waveLength - 1, darkColor, lightColor);
    }
  }

  //DIDN'T WORK
  public void setGradient(int startIndex, int endIndex, Color startColor, Color endColor)
  {
    int length = endIndex - startIndex;

    double redShift = (double)(endColor.red-startColor.red)/length;
    double greenShift = (double)(endColor.green-startColor.green)/length;
    double blueShift = (double)(endColor.blue-startColor.blue)/length;

    for(int i = 0; i <= length; i++)
    {
      setLEDBulb(i + startIndex, new Color((startColor.red+redShift*i), (startColor.green+greenShift*i), 
        (startColor.blue+blueShift*i)));
    }
  }

  public void setGradientTurret(int startIndex, int endIndex, Color startColor, Color endColor)
  {
    int length = endIndex - startIndex;

    double redShift = (double)(endColor.red-startColor.red)/length;
    double greenShift = (double)(endColor.green-startColor.green)/length;
    double blueShift = (double)(endColor.blue-startColor.blue)/length;

    for(int i = 0; i <= length; i++)
    {
      setLEDBulbTurret(i + startIndex, new Color((startColor.red+redShift*i), (startColor.green+greenShift*i), 
        (startColor.blue+blueShift*i)));
    }
  }

  public void setGradientOnTwoSides(int startIndex, int endIndex, Color endColor, Color centerColor)
  {
    int center = (endIndex + startIndex)/2;

    setGradient(center, endIndex, centerColor, endColor);
    setGradient(startIndex, center, endColor, centerColor);
  }

  public void setRainbow(int startIndex, int endIndex) 
  {
    int blockLength = (endIndex-startIndex)/3;
    setGradient(startIndex, startIndex + blockLength, Color.kRed, Color.kGreen);
    setGradient(startIndex + blockLength, endIndex - blockLength, Color.kGreen, Color.kBlue);
    setGradient(endIndex - blockLength, endIndex, Color.kBlue, Color.kRed);
  }

  public void setMultiBlock(int length, Color[] colors)
  {
    for(int i=0; i < LED_LENGTH; i += length)
    {
      setSingleBlock(i, i + length - 1, colors[(i/length) % colors.length]);
    }
  }

  public LedState getPreviousState() {
    return prevState;
  }

  public LedState getCurrentState() {
    return currState;
  }

  public void setPreviousState(LedState newState) {
    prevState = newState;
  }

  public void setCurrentState(LedState newState) {
    prevState = currState;
    currState = newState;
  }

  public Color getColorAtGradient(Color startColor, Color endColor, int length, int index) 
  {
    double redShift = (double)(endColor.red-startColor.red)/length;
    double greenShift = (double)(endColor.green-startColor.green)/length;
    double blueShift = (double)(endColor.blue-startColor.blue)/length;

    return new Color((startColor.red+redShift*index), (startColor.green+greenShift*index), 
        (startColor.blue+blueShift*index));
  }

  public void addGradientToColor(int index, int redAdd, int greenAdd, int blueAdd)
  {
    Color addColor = RBGToColor(new int[]{redAdd, greenAdd, blueAdd});
    Color originalColor = getLEDColor(index);
    setLEDBulb(index, new Color((originalColor.red + addColor.red)%256, (originalColor.green + addColor.green)%256, 
      (originalColor.blue + addColor.blue)%256));
  }

  public void setTurretLEDs(double flywheelSpeed, double targetSpeed, TurretState turretState)
  {
    if(turretState == TurretState.DISABLED) //purple
      setFullLengthTurret(RBGToColor(new int[]{254, 252, 56}));
    else if(turretState == TurretState.CAN_SHOOT)
    {
      Color newColor = getColorAtGradient(Color.kRed, Color.kGreen, (int)targetSpeed, (int)flywheelSpeed);
      Color boldColor = Color.kBlack;
      for(int i = 0; i < TURRET_LED_LENGTH; i++)
      {
        if(getLEDColorTurret(i).red >= boldColor.red && getLEDColorTurret(i).green >= boldColor.green 
          && getLEDColorTurret(i).blue >= boldColor.blue)
            boldColor = getLEDColorTurret(i);
      }

      for(int i = 0; i < TURRET_LED_LENGTH; i++) {
        addGradientToColor(i, (int)(newColor.red-boldColor.red), (int)(newColor.green-boldColor.green), 
          (int)(newColor.blue-boldColor.blue));
      }
      

      if(turretState == TurretState.TARGETING)
      {
        Color[] gradientColors = new Color[TURRET_LED_LENGTH];
        for(int i = 0; i < TURRET_LED_LENGTH; i++)
        {
          gradientColors[i] = getLEDColorTurret(i);
        }
        
        setFullLengthTurret(Color.kBlack);
        sendDataTurret();

        Timer.delay(0.2);

        for(int i = 0; i < TURRET_LED_LENGTH; i++)
        {
          setLEDBulb(i, gradientColors[i]);
        }
      }      
    }

    moveUpTurret(0, TURRET_LED_LENGTH-1, 0.1);
    sendDataTurret(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
