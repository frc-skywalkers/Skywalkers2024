// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightstripConstants;
import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.Range;
import frc.robot.lightstrip.TempLedState;
import java.util.ArrayList;
import java.util.List;

public class Lightstrip extends SubsystemBase {
  AddressableLED leds = new AddressableLED(LightstripConstants.pwmPort);

  private LedState defaultColor = LightstripConstants.defaultState;
  private Timer timer = new Timer();
  private List<LedState> currentColor = new ArrayList<LedState>();
  private List<Range> currentRange = new ArrayList<Range>();
  private List<TempLedState> tempColor = new ArrayList<TempLedState>();
  private List<Range> tempRange = new ArrayList<Range>();
  private List<Double> tempStart = new ArrayList<Double>();
  private AddressableLEDBuffer buffer;

  private boolean isDefault = false;

  /** Creates a new Lightstrip. */
  public Lightstrip() {
    timer.reset();
    timer.start();

    leds.setLength(LightstripConstants.ledCount);
    leds.start();

    isDefault = false;
  }

  @Override
  public void periodic() {
    /*if (isDefault) {
      System.out.println("Default Lightstrip: " + isDefault);
      return;
    }*/

    buffer = new AddressableLEDBuffer(LightstripConstants.ledCount);

    update(defaultColor, timer, LightstripConstants.Ranges.full);

    for (int i = 0; i < currentColor.size(); i++) {
      update(currentColor.get(i), timer, currentRange.get(i));
    }

    for (int i = 0; i < tempColor.size(); ) {
      update(tempColor.get(i), timer, tempRange.get(i));

      System.out.println(
          timer.get() + " " + tempStart.get(i) + " " + tempColor.get(i).getSeconds());

      if (timer.get() - tempStart.get(i) > tempColor.get(i).getSeconds()) {
        tempStart.remove(i);
        tempColor.remove(i);
        tempRange.remove(i);
      } else {
        i++;
      }
    }

    leds.setData(buffer);
  }

  public void setDefault(boolean defaultState) {
    isDefault = defaultState;

    /*if (isDefault) {
      candle.animate(rainbowAnim, 0);
    }*/
  }

  private void update(LedState state, Timer timer, Range range) {
    SmartDashboard.putNumber("Timer", timer.get() % 1 - 0.50 * state.getRed());

    if (state.getEffect() == "Solid") {
      for (int i = range.getStart(); i < range.getEnd(); i++) {
        buffer.setRGB(i, state.getRed(), state.getGreen(), state.getBlue());
      }
    } else if (state.getEffect() == "Blink") {
      if ((timer.get() % 2) < 1) {
        for (int i = range.getStart(); i < range.getEnd(); i++) {
          buffer.setRGB(i, state.getRed(), state.getGreen(), state.getBlue());
        }
      } else if ((timer.get() % 2) >= 1) {
        for (int i = range.getStart(); i < range.getEnd(); i++) {
          buffer.setRGB(i, 0, 0, 0);
        }
      }
    } else if (state.getEffect() == "Fast Blink") {
      if ((timer.get() % 0.5) < 0.25) {
        for (int i = range.getStart(); i < range.getEnd(); i++) {
          buffer.setRGB(i, state.getRed(), state.getGreen(), state.getBlue());
        }
      } else if ((timer.get() % 0.5) >= 0.25) {
        for (int i = range.getStart(); i < range.getEnd(); i++) {
          buffer.setRGB(i, 0, 0, 0);
        }
      }
    } else if (state.getEffect() == "Fade") {
      for (int i = range.getStart(); i < range.getEnd(); i++) {
        buffer.setRGB(
            i,
            (int) (state.getRed() * Math.abs((timer.get() % 2) - 1)),
            (int) (state.getGreen() * Math.abs((timer.get() % 2) - 1)),
            (int) (state.getBlue() * Math.abs((timer.get() % 2) - 1)));
      }
    } else if (state.getEffect() == "Signal") {
      int chain = 0;

      if ((timer.get() % 1)
          < 0.125) { // i know there is prob a better way to do this, but im too lazy to
        chain = 0;
      } else if ((timer.get() % 1) < 0.25) {
        chain = 1;
      } else if ((timer.get() % 1) < 0.375) {
        chain = 2;
      } else if ((timer.get() % 1) < 0.5) {
        chain = 3;
      } else if ((timer.get() % 1) < 0.625) {
        chain = 4;
      } else if ((timer.get() % 1) < 0.75) {
        chain = 5;
      } else if ((timer.get() % 1) < 0.875) {
        chain = 6;
      } else if ((timer.get() % 1) < 1) {
        chain = 7;
      }

      for (int i = 0; i < range.getStart() - range.getEnd() + 1; i++) {
        int ratio = 6 - chain;

        if (ratio < 0) {
          ratio = 0;
        }

        buffer.setRGB(
            (int) (state.getRed() * (ratio / 6)),
            (int) (state.getGreen() * (ratio / 6)),
            (int) (state.getBlue() * (ratio / 6)),
            range.getStart() + i);

        chain++;

        if (chain == 8) {
          chain = 0;
        }
      }
    } else if (state.getEffect() == "Wave") {
      int chain = 0;

      if ((timer.get() % 1)
          < 0.125) { // i know there is prob a better way to do this, but im too lazy to
        chain = 0;
      } else if ((timer.get() % 1) < 0.25) {
        chain = 1;
      } else if ((timer.get() % 1) < 0.375) {
        chain = 2;
      } else if ((timer.get() % 1) < 0.5) {
        chain = 3;
      } else if ((timer.get() % 1) < 0.625) {
        chain = 4;
      } else if ((timer.get() % 1) < 0.75) {
        chain = 5;
      } else if ((timer.get() % 1) < 0.875) {
        chain = 6;
      } else if ((timer.get() % 1) < 1) {
        chain = 7;
      }

      for (int i = 0; i < range.getStart() - range.getEnd() + 1; i++) {
        int ratio = 4 - chain;

        if (ratio < 0) {
          ratio = chain - 5;
        }

        buffer.setRGB(
            (int) (state.getRed() * chain / 6),
            (int) (state.getGreen() * chain / 6),
            (int) (state.getBlue() * chain / 6),
            range.getStart() + i);

        chain++;

        if (chain == 8) {
          chain = 0;
        }
      }
    }
  }

  public void toggleOnColor(LedState state, Range range) {
    setColor(state, range);
  }

  public void toggleOffColor(LedState state, Range range) {
    List<LedState> states = new ArrayList<LedState>();
    List<Range> ranges = new ArrayList<Range>();

    for (int i = 0; i < currentColor.size(); i++) {
      if (!(currentColor.get(i).equals(state) && currentRange.get(i).equals(range))) {
        states.add(state);
        ranges.add(range);
      }
    }

    currentColor = states;
    currentRange = ranges;
  }

  public void setColor(LedState state, Range range) {
    toggleOffColor(state, range);

    currentColor.add(state);
    currentRange.add(range);
  }

  public void tempColor(TempLedState state, Range range) {
    tempColor.add(state);
    tempRange.add(range);
    tempStart.add(Double.valueOf(timer.get()));
  }
}
