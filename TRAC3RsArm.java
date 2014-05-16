/*
Copyright (c) 2014, Christoph Stahl
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import java.awt.geom.Rectangle2D;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/*
* 		upper: 270
* left: -105	right: 125
* 		bottom: 115
* 
* Drawing rectangle size: 155x230
*/

public class TRAC3RsArm {
	private final double length1 = 9.6 * 15; // 15 FLU
	private final double length2 = 9.6 * 16.5; // 16.5 FLU
	
	private final double gearRatio1 = 56.0 / 4.0;
	private final double gearRatio2 = 56.0 / 3.0;
	
	private final Rectangle2D.Double drawingRect = new Rectangle2D.Double(-106.0, 114.0, 232.0, 157.0);
	
	private final String touchSensorPortName1 = "S1";
	private final String touchSensorPortName2 = "S4";
	
	private final Port motorPort1 = MotorPort.B;
	private final Port motorPort2 = MotorPort.C;
	private final Port motorPort3 = MotorPort.D;
	
	private SampleProvider touchSensor1; // sensor for initializing the shoulder
	private SampleProvider touchSensor2; // sensor for initializing the elbow
	
	private EV3LargeRegulatedMotor motor1; // the base or "shoulder" motor
	private EV3LargeRegulatedMotor motor2; // the "elbow" motor
	private EV3MediumRegulatedMotor motor3; // the "hand" motor
	
	private int handUpPosition = 100;
	private int handDownPosition = 50;
	
	public TRAC3RsArm() {
		
	
		System.out.println("Setting up Touch Sensor 1...");
		touchSensor1 = new EV3TouchSensor(LocalEV3.get().getPort(touchSensorPortName1));
	
		System.out.println("Setting up Touch Sensor 2...");
		touchSensor2 = new EV3TouchSensor(LocalEV3.get().getPort(touchSensorPortName2));
	
		System.out.println("Setting up Motor 1...");
		motor1 = new EV3LargeRegulatedMotor(motorPort1);
		motor1.rotateTo(0);

		System.out.println("Setting up Motor 2...");
		motor2 = new EV3LargeRegulatedMotor(motorPort2);
		motor2.rotateTo(0);

		System.out.println("Setting up Motor 3...");
		motor3 = new EV3MediumRegulatedMotor(motorPort3);
		motor3.rotateTo(0);	
	
	}
	

	
	
	public void initialize()
	{
		System.out.println("Initializing arm positon...");
		
		Sound.twoBeeps();
		Button.LEDPattern(9);
	
		setBothJointsSpeed(500);
		// Find home for motor 1
		System.out.println("Homing motor1...");
		
		float[] sample = new float[touchSensor1.sampleSize()];
		
		motor1.forward();
		
		do {
			touchSensor1.fetchSample(sample, 0);
			Thread.yield();
			//Delay.msDelay(10);
		} while (sample[0] == 0.0);
		
		motor1.stop();
		motor1.resetTachoCount();
		
		System.out.println("... motor1 reached home.");
		
		// Find home for motor 2
		System.out.println("Homing motor2...");
		motor1.rotateTo((int)(-115 * gearRatio1), false);
		
		motor2.forward();
		do {
			touchSensor2.fetchSample(sample, 0);
			Thread.yield();
			//Delay.msDelay(10);
		} while (sample[0] == 0.0);
		
		motor2.stop();
		motor2.resetTachoCount();
		System.out.println("... motor2 reached home.");
		
		//Move to position 0, 0 (degrees)
		System.out.println("Stretching arm...");
				
		motor1.rotateTo((int)(-94 * gearRatio1), true);
		motor2.rotateTo((int)(-1 * 67 * gearRatio2), true);
		
		while (areJointsMoving()){
			Thread.yield();
			//Delay.msDelay(10);
		}
		
		motor1.resetTachoCount();
		motor2.resetTachoCount();
		
		System.out.println("... done");
		
		Sound.beepSequenceUp();
		Button.LEDPattern(7);
	}
	
	public void calibrateHand()
	{
		System.out.println("Move hand up...");
		
	    motor3.setStallThreshold(5, 15);
	    motor3.setAcceleration(500);
	    motor3.setSpeed(600);
		motor3.rotate(-720, false);
		motor3.resetTachoCount();
		
		positionHand(handDownPosition, true);
		
		boolean finished = false;
		while (!finished) {
			LCD.clear();
			LCD.drawString("hand down position: ", 0, 0);
			LCD.drawString("" + handDownPosition+ "%" , 5, 1);
			LCD.drawString("up, down: +-10%", 0, 2);
			LCD.drawString("left, right: +-2%", 0, 3);
			LCD.refresh();

			int buttonId = Button.waitForAnyPress();
			switch (buttonId) {
			case Button.ID_ESCAPE:
				finished = true;
				break;
			case Button.ID_UP:
				handDownPosition += 10;
				if (handDownPosition > 100) handDownPosition = 100;
				break;
			case Button.ID_DOWN:
				handDownPosition -= 10;
				if (handDownPosition < 0) handDownPosition = 0;
				break;
			case Button.ID_ENTER:
				finished = true;
				break;
			case Button.ID_LEFT:
				handDownPosition -= 2;
				if (handDownPosition < 0) handDownPosition = 0;
				break;
			case Button.ID_RIGHT:
				handDownPosition += 2;
				if (handDownPosition > 100) handDownPosition = 100;
				break;
			}
			positionHand(handDownPosition, true);
		}
		LCD.clear();
		LCD.drawString("set hand down ", 0, 3);
		LCD.drawString("position to " + handDownPosition+ "%" , 0, 4);
		LCD.refresh();
		positionHand(100, false);
		
	}
	
	public void testHandPosCodeFreely()
	{
		motor1.flt();
		motor2.flt();
				
		while (true) {
			double phi1 = getJoint1Position();
			double phi2 = getJoint2Position();
			int hp = handPositionForAngles(phi1, phi2);
			positionHand(hp, true);
		}
	}
	

	
	public int handPositionForAngles(double phi1, double phi2)
	{
		int handPos = handDownPosition;
		
		double phi1Factor = -6.0/30.0;
		
		double phi2aFactor = -3.5/50.0;
		
		double phi2bFactor = 28.0/40.0;
		double phi2bZeroOffset = -7.7;
		
		int newHandPos;
		if (phi2 > 110.0) {
			newHandPos = (int)(handPos - (phi1Factor * phi1) - (phi2bFactor * (phi2 - 110.0) + phi2bZeroOffset));
		} else {

			newHandPos = (int)(handPos - (phi1Factor * phi1) - (phi2aFactor * phi2));
		}
		
		System.out.format("Phi1: %.1f phi2: %.1f, hdp: %d, nhdp %d\n", phi1, phi2, handDownPosition, newHandPos);
		return newHandPos;
	}
	
	public void setBothJointsSpeed(int speed){
		//set motor speed
		//the faster motor should be set according to defined default
		double jointRatio = this.gearRatio1 / this.gearRatio2;
		if (jointRatio > 1) {
			// joint1 motor needs to be faster (and default) than joint2 to turn arm as fast
			motor1.setSpeed(speed);
			motor2.setSpeed((int) (speed / jointRatio));
		} else {
			//joint2 needs to be faster (and default!) than joint1
			motor1.setSpeed((int) (speed * jointRatio));
			motor2.setSpeed(speed);
		}
	}
	
	public void setBothJointsAcceleration(int acceleration){
		//set motor acceleration
		//the faster motor should be set according to defined default
		double jointRatio = this.gearRatio1 / this.gearRatio2;
		if (jointRatio > 1) {
			// joint1 motor needs to be faster (and default) than joint2 to turn arm as fast
			motor1.setAcceleration(acceleration);
			motor2.setAcceleration((int) (acceleration / jointRatio));
		} else {
			//joint2 needs to be faster (and default!) than joint1
			motor1.setAcceleration((int) (acceleration * jointRatio));
			motor2.setAcceleration(acceleration);
		}
	}
	
	public void setJoint1Speed(int speed)
	{
		motor1.setSpeed(speed);
	}
	
	public void setJoint2Speed(int speed)
	{
		motor2.setSpeed(speed);
	}
	
//	void rotateJoint1(double limitAngle, boolean immediateReturn)
//	{
//		motor1.rotate((int) (limitAngle * gearRatio1), immediateReturn);
//	}
//	
//	void rotateJoint2(double limitAngle, boolean immediateReturn)
//	{
//		motor2.rotate((int) (-1 * limitAngle * gearRatio2), immediateReturn);
//	}
	
	public void rotateJoint1To(double limitAngle, boolean immediateReturn)
	{
		if (limitAngle >= -35 && limitAngle <= 90) {
			motor1.rotateTo((int)(limitAngle * gearRatio1), immediateReturn);
		}
		else System.out.format("Joint1: Request limitAngle %f is not allowed!\n", limitAngle);
	}
	
	public void rotateJoint2To(double limitAngle, boolean immediateReturn)
	{
		if (limitAngle >= 0 && limitAngle <= 140) {
			motor2.rotateTo((int)(-1 * limitAngle * gearRatio2), immediateReturn);
		}
		else System.out.format("Joint2: Request limitAngle %f is not allowed!\n", limitAngle);
	}
	
	public void rotateJoints(double angle1, double angle2) //returns immediately
	{
		rotateJoint1To(angle1, true);
		rotateJoint2To(angle2, true);
	}
	
	public void moveTo(double x, double y) // returns immediately
	{
		if (drawingRect.contains(x, y)) {
			setJoint1Speed(900);
			setJoint2Speed(900);
			Kinematics kinematics = new Kinematics(length1, length2);
			double[] angles = kinematics .inverseKinematics(x, y);
			rotateJoints(angles[0], angles[1]);
		} else
			System.out.format("Point %.1f, %.1f is not in drawing area!\n", x, y);
	}
	
	void displayState()
	{
		motor1.flt();
		motor2.flt();
		LCD.clear();
		Kinematics kinematics = new Kinematics(getLength1(), getLength2());
		double[] XY = kinematics.forwardKinematics(getJoint1Position(), getJoint2Position());
		
		LCD.drawString("Cur X, Y: " + (int)XY[0] + "," +  (int)XY[1], 0, 0);
		LCD.drawString("Cur <: " + (int)this.getJoint1Position() + ", " + (int)this.getJoint2Position(), 0, 1);
		LCD.refresh();
	}
	

		
	public boolean isJoint1Moving()
	{
		return motor1.isMoving();
	}
	
	public boolean isJoint2Moving()
	{
		return motor2.isMoving();
	}
	
	public boolean areJointsMoving()
	{
		return motor1.isMoving() || motor2.isMoving();
	}
	
	public double getLength1()
	{
		return length1;
	}
	
	public double getLength2()
	{
		return length2;
	}
	
	public void positionHand(int amount, boolean immediateReturn) // 0 to 100
	{
		if (amount > 100) amount = 100;
		else if (amount < 0) amount = 0;
		
		double target = -7.2 * amount + 720;

		motor3.rotateTo((int)target, immediateReturn);
	
	}
	
	public double getJoint1Position()
	{
		return motor1.getTachoCount() / gearRatio1;
	}
	
	public double getJoint2Position()
	{
		return -1 * motor2.getTachoCount() / gearRatio2;
	}
	
	public int getJoint1TachoCount()
	{
		return motor1.getTachoCount();
	}
	
	public int getJoint2TachoCount()
	{
		return motor2.getTachoCount();
	}
	
	public int getJoint1LimitAngle()
	{
		return motor1.getLimitAngle();
	}
	
	public int getJoint2LimitAngle()
	{
		return motor2.getLimitAngle();
	}
	
	public int getJoint1Distance()
	{
		return motor1.getLimitAngle() - motor1.getTachoCount();
	}
	
	public int getJoint2Distance()
	{
		return motor2.getLimitAngle() - motor2.getTachoCount();
	}
	
	public double getGearRatio1()
	{
		return gearRatio1;
	}
	
	public double getGearRatio2()
	{
		return gearRatio2;
	}
	
	public Rectangle2D.Double getDrawingRect()
	{
		return drawingRect;
	}
}

