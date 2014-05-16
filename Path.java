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
import java.util.ArrayList;
import java.util.List;

public class Path implements Comparable<Path>
{
	private final double minDistance = 1.0;
	private List<double[]> points;
	private double length;
	private Rectangle2D boundingBox;
	
	public Path(List<double[]> points)
	{
		this.points = points;
		calculateBoundingBox();
		interpolate();
		length = calculateLength();
		
		System.out.println(this);
	//	System.out.println("Path");
		//for (double[] point : points)
			//System.out.println(point[0] + ", " + point[1]);
	}
	
	private void calculateBoundingBox()
	{
		double minX, maxX, minY, maxY;
		minX = minY = Double.POSITIVE_INFINITY;
		maxX = maxY = Double.NEGATIVE_INFINITY;
		for (double[] point : points) {
			if (point[0] < minX) minX = point[0];
			if (point[0] > maxX) maxX = point[0];
			if (point[1] < minY) minY = point[1];
			if (point[1] > maxY) maxY = point[1];
		}
		boundingBox = new Rectangle2D.Double(minX, minY, maxX - minX, maxY - minY);
	}
	
	public Rectangle2D getBoundingBox()
	{
		return boundingBox;
	}
	
	public List<double[]> getPoints()
	{
		return points;
	}
	
	public void setPoints(List<double[]> points)
	{
		this.points = points;
		interpolate();
		length = calculateLength();
	}
	
	public double[] getStart()
	{
		return points.get(0);
	}
	
	private void interpolate()
	{
		int size = points.size();
		
		List<Object[]> toInsert = new ArrayList<Object[]>();
		for (int i = 0; i < size; i++){
			if (i != size - 1) {
				double dist = distance(points.get(i), points.get(i + 1));
				if (dist > 2 * minDistance){
					toInsert.add(0, new Object[]{i+1, interpolatePoints(points.get(i), points.get(i + 1))});
				}
			}
		}
		for (Object[] pair : toInsert){
			int index = (int)pair[0];
			@SuppressWarnings("unchecked")
			List<double[]> interpolated = (List<double[]>)pair[1];
			points.addAll(index, interpolated);
		}
	}
	
	private List<double[]> interpolatePoints(double[] p1, double[] p2)
	{
		double dist = distance(p1, p2);
		int steps = (int)(dist/minDistance);
		double stepLength = dist / steps;
		
		double[] normdirection = new double[]{(p2[0] - p1[0]) / dist, (p2[1] - p1[1]) / dist};
		
		List<double[]> interpolated = new ArrayList<double[]>();
		
		for(int i = 1; i < steps; i++)
		{
			//System.out.format("P1 %.1f, %.1f P2 %.1f, %.1f : %d -> %.1f, %.1f\n", p1[0], p1[1], p2[0], p2[1], i, p1[0] + normdirection[0] * stepLength * i, p1[1] + normdirection[1] * stepLength * i );
			interpolated.add(new double[]{p1[0] + normdirection[0] * stepLength * i, p1[1] + normdirection[1] * stepLength * i});
		}
		return interpolated;
	}
	
	
	public static double distance(double[]p1, double[]p2)
	{
		return Math.sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]));
	}
	
	private double calculateLength()
	{
		double length = 0.0;
		int size = points.size();
		for (int i = 0; i < size; i++){
			if (i != size - 1) {
				length += distance(points.get(i), points.get(i + 1));
			}
		}
		return length;
	}
	
	public double getLength()
	{
		return length;
	}
	
	public int numberOfPoints()
	{
		return points.size();
	}
	
	
	@Override
	public String toString(){
		return "Path with length " + getLength() + ", " + numberOfPoints() + " points, boundingbox: " + boundingBox;
	}

	@Override
	public int compareTo(Path o) {
		if (this == o) return 0;
		else if (this.getLength() < o.getLength()) return -1;
		else if (this.getLength() > o.getLength()) return 1;
		else
			return 0;
	}
	
	
}
