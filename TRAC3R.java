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

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import com.kitfox.svg.SVGException;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

//import lejos.robotics.RegulatedMotorListener;




/* Notes
* Button.LEDPattern(3); //off 0, static: green 1, red 2, yellow 3, rhythmic: green 7, red 8, yellow 9
* Drawing Rectangle in mm from origin:
* 
* 		upper: 270
* left: -105	right: 125
* 		bottom: 115
* 
* Drawing rectangle size: 155x230
*/

public class TRAC3R
{
	private TRAC3RsArm arm;
	
	public TRAC3R()
	{
		System.out.println("Setting up TRAC3R...");
		
		LCD.clear();
		
		System.out.println("Setting up TRAC3RsArm...");
		arm = new TRAC3RsArm();
		
	}
	
	
	public void start() throws InterruptedException
	{
	    
	    arm.initialize();
	   
	    arm.calibrateHand();
	    
	    //arm.testHandPosCodeFreely();
	    
		//generate interpolated paths from file
		List<Path> paths = null;
		try {
			paths = SVGHandler.pathsInSVGFile(new URI("file:///tmp/job.svg"));
		} catch (SVGException | URISyntaxException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		if (paths != null) {
			//calculate inverse kinematics and draw as soon as first path is calculated
			BlockingQueue<List<double[]>> queue = new ArrayBlockingQueue<List<double[]>>(1);
			
			Thread producer = new Thread(new MovementDataProducer(arm, queue, paths));
			Thread consumer = new Thread(new MovementController(arm, queue));
			
			producer.setDaemon(true);
			consumer.setDaemon(true);
			
			producer.start();
			consumer.start();
			
			producer.join();
			consumer.join();
			System.out.println("Finished all.");	
		}
		
		Button.LEDPattern(0);
	}
	
	
	
	public static void main(String[] args) throws InterruptedException 
	{

		TRAC3R robot = new TRAC3R();
		robot.start();
	}
}


