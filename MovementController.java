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

import java.util.*;
import java.util.Collections.*;
import java.util.concurrent.BlockingQueue;
import java.awt.geom.Rectangle2D;

import lejos.utility.Delay;


public class MovementController implements Runnable
{
	private TRAC3RsArm arm;
	private BlockingQueue<List<double[]>> queue;
	
	public MovementController(TRAC3RsArm arm, BlockingQueue<List<double[]>> queue)
	{
		this.arm = arm;
		this.queue = queue;
	}
	
	@Override
	public void run() {

		System.out.println("Starting drawing all paths.");
		List<double[]> pathAngles;
		try {
			while (true){
				System.out.println("Taking path angles from queue.");
				pathAngles = queue.take();
				if (pathAngles.size() == 0) break;
				System.out.println("Starting drawing path.");
				drawPath(pathAngles);
				System.out.println("Finished drawing path.");
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println("Finished drawing all paths.");
		arm.positionHand(100,false);
		moveHome();
		
	}
	
	public void moveHome()
	{
		arm.setBothJointsSpeed(300);
		arm.rotateJoints(0, 0);
		waitForJointMovement();
	}
	
	void waitForJointMovement()
	{
		while (arm.areJointsMoving()) {
			Thread.yield();
		}
	}
	
	public void drawPath(List<double[]> pathAngles)
	{
		System.out.println("Drawing List of Angles");
		
		int epsilon = 2;

		//move to start of path
		double[] startAngles = pathAngles.get(0);
		arm.setBothJointsSpeed(200);
		arm.rotateJoints(startAngles[0], startAngles[1]);
		waitForJointMovement();
		
		//draw path
		arm.setBothJointsSpeed(100);
		arm.positionHand(arm.handPositionForAngles(startAngles[0], startAngles[1]), false);
		
		boolean isJoint1Moving;
		int joint1Limit,
		    joint1Distance;
		
		boolean isJoint2Moving;
		int joint2Limit,
		    joint2Distance;
		
		double currentTarget1,
			   currentTarget2;
		
		ListIterator<double[]> iter = pathAngles.listIterator();
		
		while (iter.hasNext()){
			double[] current = iter.next();
			
			arm.rotateJoints(current[0], current[1]);
			arm.positionHand(arm.handPositionForAngles(current[0], current[1]), true);
			
			while (true){
				isJoint1Moving = arm.isJoint1Moving();
				joint1Limit = arm.getJoint1LimitAngle();
				joint1Distance = arm.getJoint1Distance();
				
				isJoint2Moving = arm.isJoint2Moving();
				joint2Limit = arm.getJoint2LimitAngle();
				joint2Distance = arm.getJoint2Distance();
				
				currentTarget1 = current[0] * arm.getGearRatio1();
				currentTarget2 = -1 * current[1] * arm.getGearRatio2();
				
				if (!arm.areJointsMoving()) 
					break;
				else {
					boolean needWait1 = isJoint1Moving && ((joint1Distance > 0 && currentTarget1 < joint1Limit) || (joint1Distance < 0 && currentTarget1 > joint1Limit));
					boolean needWait2 = isJoint2Moving && ((joint2Distance > 0 && currentTarget2 < joint1Limit) || (joint2Distance < 0 && currentTarget2 > joint2Limit));
					
					if (!needWait1 && !needWait2) {
						//System.out.println("Starting early!");
						boolean startNext1 = !isJoint1Moving || ((Math.abs(joint1Distance) <= epsilon));
						boolean startNext2 = !isJoint2Moving || ((Math.abs(joint2Distance) <= epsilon));
						if (startNext1 && startNext2)
							break;
					}
				}
				//Delay.msDelay(10);
				 
			}
			
		}
		
		arm.positionHand(95, false);

	}
	
}


