// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Comparator;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem ledSubsystem = null;
  private Spark ledControl = new Spark(0);
  private Spark backLED = new Spark(1);
  private LEDBack ledBack;

  private PriorityQueue<PriorityColor> queue = null;
  

  public final PriorityColor DEFAULT = new PriorityColor(LEDConstants.FIRE_MED, 100, "fire");
  public final PriorityColor BALANCED = new PriorityColor(LEDConstants.CONFETTI, 20, "party");
  public final PriorityColor INTAKE_CONE = new PriorityColor(LEDConstants.STROBE_GOLD, 31, "cone"); // 30
  public final PriorityColor INTAKE_CUBE = new PriorityColor(LEDConstants.SOLID_VIOLET, 30, "cube");
  public final PriorityColor PICKED_UP = new PriorityColor(LEDConstants.SOLID_GREEN, 25, "picked up");

  
  private LEDSubsystem() {
    ledBack = new LEDBack();
    queue = new PriorityQueue<PriorityColor>(new PriorityColor());
    addDefaultColor();
    
  }

  public boolean queueContains(PriorityColor color) {
    return queue.contains(color);
  }

  public static LEDSubsystem getInstance() {
    if (ledSubsystem == null) {
      ledSubsystem = new LEDSubsystem();
    }
    return ledSubsystem;
  }

  public LEDBack getBackInstance() {
    return ledBack;
  }

  public void addColor(PriorityColor color) {
    if(!queue.contains(color)) {
      queue.add(color);
    }
  }

  public void removeColor(PriorityColor color) {
    if(queue.contains(color)) {
      queue.remove(color);
    }
  }

  private void addDefaultColor() {
    queue.add(DEFAULT);
  }

  public void ledTest() {
    ledControl.set(-.25);
  }

  public String printQueue() {
    String queueItems = "";
    for (PriorityColor color : queue) {
      queueItems += color.name + ": " + color.priority + ", ";
    }
    
    return queueItems;  
  }

  @Override
  public void periodic() {
    /*try {
      ledControl.set(queue.peek().color);
      backLED.set(ledBack.getColor());
    } catch (NullPointerException e) {
      System.err.println("queue is empty");
    }*/

    ledControl.set(queue.peek().color);
    backLED.set(ledBack.getColor());
    
    //ledTest();
  }

  public class LEDBack {
    private PriorityQueue<PriorityColor> queue2 = null;
    LEDBack ledBack;

    private LEDBack() {
      queue2 = new PriorityQueue<PriorityColor>(new PriorityColor());
    }
/* 
    public LEDBack getInstance() {
      if (ledBack == null) {
        ledBack = new LEDBack();
      }
      return ledBack;
    }
*/
    public double getColor() {
      if(queue2.peek() == null) {
        return .93; // solid white (default for back led)
      }
      return queue2.peek().color;
    }

    public void addColor(PriorityColor color) {
      if(!queue2.contains(color)) {
        queue2.add(color);
      }
    }

    public void removeColor(PriorityColor color) {
      if(queue2.contains(color)) {
        queue2.remove(color);
      }
    }

    public String getQueueString() {
      String queueItems = "";
      for (PriorityColor color : queue2) {
        queueItems += color.name + ": " + color.priority + ", ";
      }
    
      return queueItems;
    }

  }

  private class PriorityColor implements Comparator<PriorityColor>, Comparable<PriorityColor> {

    public double color;
    public String name;
    public int priority;
    public int loopCount = -10;

    public PriorityColor() {}
    
    public PriorityColor(double color, int priority, String name) {
      this.color = color;
      this.priority = priority;
    }

    public int compare(PriorityColor p1 , PriorityColor p2) {
        if(p1.priority>= p2.priority) {
          return 1;
        } else {
          return -1;
        }
    }

    public int compareTo(PriorityColor p2) {
      return compare(this, p2);
    }
  }
}
