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

import java.lang.Math;


public class Kinematics 
{
	private final double length1;
	private final double length2;
	
	public Kinematics(double length1, double length2)
	{
		this.length1 = length1;
		this.length2 = length2;
	}
	
	public double[] inverseKinematics(double x, double y){
		
		//phi2
		double cphi2 = ((x * x) + (y * y) - (length1 * length1) - (length2 * length2)) / (2 * length1 * length2);
		double temp = Math.max(0.0, 1 - cphi2 * cphi2);
		double sphi2 = Math.sqrt(temp); //+- ! But we compute only one of the two valid positions ("elbow down")
		double phi2 = Math.atan2(sphi2, cphi2);
		
		//phi1
		double cphi1 = (x * (length1 + length2 * Math.cos(phi2)) + y * length2 * Math.sin(phi2)) / (x * x + y * y);
		
		temp = Math.max(0.0, 1 - cphi1 * cphi1);
		
		double sphi1 = Math.sqrt(temp); //+-! We need to compute both and select one
		
		double phi1_1 = Math.atan2(sphi1, cphi1);
		double phi1_2 = Math.atan2(-sphi1, cphi1);
		
		//select the correct phi1
		double[] forward = forwardKinematics(Math.toDegrees(phi1_1), Math.toDegrees(phi2));
		
		
		if ( (Math.abs(forward[0] - x) < 0.001) && (Math.abs(forward[1] - y) < 0.001) ) {
			//System.out.format("x: %.2f, y: %.2f, phi1: %.2f, phi2: %.2f\n", x, y, Math.toDegrees(phi1_1), Math.toDegrees(phi2));
			return new double[]{ Math.toDegrees(phi1_1),  Math.toDegrees(phi2)};
		} else {
			//System.out.format("x: %.2f, y: %.2f, phi1: %.2f, phi2: %.2f\n", x, y, Math.toDegrees(phi1_2), Math.toDegrees(phi2));
			return new double[]{ Math.toDegrees(phi1_2),  Math.toDegrees(phi2)};
		}
	}
	
	public double[] forwardKinematics(double phi1, double phi2)
	{
		double x = this.length1 * Math.cos(Math.toRadians(phi1)) + this.length2 * Math.cos(Math.toRadians(phi1) + Math.toRadians(phi2));
		double y = this.length1 * Math.sin(Math.toRadians(phi1)) + this.length2 * Math.sin(Math.toRadians(phi1) + Math.toRadians(phi2));
		
		return new double[]{x,y};
	}
}
