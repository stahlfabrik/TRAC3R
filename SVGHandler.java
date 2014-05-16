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

// import svgSalamander-tiny.jar from https://svgsalamander.java.net into the project!

import java.awt.Shape;
import java.awt.geom.PathIterator;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.net.URI;
import java.util.*;

import com.kitfox.svg.*;


public class SVGHandler {
	
	public static List<Path> pathsInSVGFile(URI uri) throws SVGException
	{
		List<Path> listOfPaths = new ArrayList<Path>();
		
		// Get SVGUniverse and set it to load the SVG file
		System.out.println("Reading SVG file to memory...");
		SVGDiagram diagram = SVGCache.getSVGUniverse().getDiagram(uri);
		System.out.println("... done reading SVG file to memory.");
		
		// Get Root Element and search its children...
		SVGRoot root = diagram.getRoot();
		//root.calcBoundingBox();
		Rectangle2D bound = root.getBoundingBox();
		
		/*
		* 		upper: 270
		* left: -105	right: 125
		* 		bottom: 115
		* 
		* Drawing rectangle size: 155x230
		*/
		
		System.out.println(bound);
		double totalX = bound.getX() + bound.getWidth();
		double totalY = bound.getY() + bound.getHeight();
		double scaleX = 1.0;
		double scaleY = 1.0;
		
		if (totalY > 155.0){
			scaleY = 155.0 / totalY;
			System.out.println("Total Y " + totalY + " bigger than 155. Set Y scale to " + scaleY);
		}
		if (totalX > 230.0){
			scaleX = 230.0 / totalX;
			System.out.println("Total X " + totalX + " bigger than 230. Set X scale to " + scaleX);
		}
		double scaleMin = Math.min(scaleX, scaleY);
		
		
		AffineTransform at = new AffineTransform();


		at.translate(-105.0, 115.0);
		at.scale(scaleMin, scaleMin);
		at.translate( (bound.getY() + (bound.getHeight() / 2.0)), (bound.getX() + (bound.getWidth() / 2.0)) );
		at.rotate(Math.PI/2.0);
		at.translate( -(bound.getX() + (bound.getWidth() / 2.0)), -(bound.getY() + (bound.getHeight() / 2.0)) );
		
		
		HashSet<com.kitfox.svg.ShapeElement> shapes = new HashSet<com.kitfox.svg.ShapeElement>();
		
		Stack<SVGElement> stack = new Stack<SVGElement>();
		stack.push(diagram.getRoot());
		
		while (!stack.isEmpty()) {
			SVGElement node = stack.pop();
			
			if (node instanceof com.kitfox.svg.Text || 
				node instanceof com.kitfox.svg.Path ||
				node instanceof com.kitfox.svg.Circle ||
				node instanceof com.kitfox.svg.Ellipse ||
				node instanceof com.kitfox.svg.Line ||
				node instanceof com.kitfox.svg.Polygon ||
				node instanceof com.kitfox.svg.Polyline ||
				node instanceof com.kitfox.svg.Rect) 
			{
				System.out.println("Found supported shape " + node);
				shapes.add((com.kitfox.svg.ShapeElement)node);
				
			} else {
				System.out.println("No supported shape " + node);
			}
			System.out.println("Pushing children of ... " + node);
			for (Object element : node.getChildren(null)) {
				System.out.println("... child " + element);
				stack.push((SVGElement)element);
			}	
		}
		
		System.out.println("Found " + shapes.size() + " shapes in SVG file!");
		for (com.kitfox.svg.ShapeElement shape : shapes) {
			System.out.println("Iterating through shape: " + shape);
						
			Shape awtShape = shape.getShape();

			PathIterator pathIterator = awtShape.getPathIterator(at, 1.0d);
			

			List<double[]> points = null;
			double[] coords = null;
			double[] lastMoveTo = null;
			int retValue;
			boolean pathAdded = false;
			
			while(!pathIterator.isDone()){
				coords = new double[2];
				retValue = pathIterator.currentSegment(coords);
				switch (retValue) {
				case PathIterator.SEG_CLOSE:
					System.out.println("SEG_CLOSE " + coords[0] + ", " + coords[1]);
					points.add(lastMoveTo);
					System.out.println("... done generating path object from SVG-path...");
					listOfPaths.add(new Path(points));
					pathAdded = true;
					break;
				case PathIterator.SEG_CUBICTO:
					System.out.println("SEG_CUBICTO NOT IMPLEMENTED! " + coords[0] + ", " + coords[1]);
					break;
				case PathIterator.SEG_LINETO:
					System.out.println("SEG_LINETO" + coords[0] + ", " + coords[1]);
					points.add(coords);
					break;
				case PathIterator.SEG_MOVETO:
					System.out.println("SEG_MOVETO " + coords[0] + ", " + coords[1]);
					System.out.println("Generating path object from SVG-path...");
					points = new ArrayList<double[]>();
					lastMoveTo = coords;
					points.add(coords);
					pathAdded = false;
					break;
				case PathIterator.SEG_QUADTO:
					System.out.println("SEG_QUADTO NOT IMPLEMENTED!" + coords[0] + ", " + coords[1]);
					break;
			//	case PathIterator.WIND_EVEN_ODD:
			//		System.out.println("WIND_EVEN_ODD " + coords[0] + ", " + coords[1]);
			//		break;
			//	case PathIterator.WIND_NON_ZERO:
			//		System.out.println("WIND_NON_ZERO " + coords[0] + ", " + coords[1]);
			//		break;

				default:
					break;
				}
				pathIterator.next();
			}
			if (!pathAdded) {
				System.out.println("... finally done generating path object from awtShape...");
				listOfPaths.add(new Path(points));
			}
		}

		System.out.println("Finished processing of SVG File. Found " + listOfPaths.size() + " paths...");
		return listOfPaths;
	}
	
	public static void main(String[] args) {
		
		
		

	}
}
