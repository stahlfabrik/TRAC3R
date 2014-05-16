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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.BlockingQueue;



public class MovementDataProducer implements Runnable
{

	private BlockingQueue<List<double[]>> queue;
	private List<Path> paths;
	private Kinematics kinematics;
	private TRAC3RsArm arm;
	
	public MovementDataProducer(TRAC3RsArm arm, BlockingQueue<List<double[]>> queue, List<Path> paths)
	{
		this.arm = arm;
		kinematics = new Kinematics(arm.getLength1(), arm.getLength2());
		this.queue = queue;
		this.paths = paths;
	}
	
	@Override
	public void run() {
		
		System.out.println("Angle producer was started.");
		Collections.sort(paths); //sort paths by their length
		Collections.reverse(paths); //Longest path first
		
		for (Path path : paths){
		
			//check if path fits drawing area
			if (!pathFits(path)){
				System.out.println("Path does not fit drawing area! Skipping it. PathBBox:" + path.getBoundingBox());
				continue;
			}
			
			List<double[]> angles = new ArrayList<double[]>();
			for (double[] point : path.getPoints()) {
				angles.add(kinematics.inverseKinematics(point[0], point[1]));
			}
			
			try {
				System.out.println("Adding list of " + angles.size() + " angles to queue");
				queue.put(angles);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		// add Zero Path to signal end of all paths
		System.out.println("Adding empty list of angles to queue");
		List<double[]> angles = new ArrayList<double[]>();
		try {
			queue.put(angles);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private boolean pathFits(Path path)
	{
		for (double[] point : path.getPoints()) {
			if (!arm.getDrawingRect().contains(point[0], point[1])) {
				return false;
			}
		}
		return true;
	}

}
