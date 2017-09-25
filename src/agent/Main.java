package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Stack;

public class Main {
	
	
    public static void main(String args[]) throws IOException {
    	//Load in the problem
        ProblemSpec spec = new ProblemSpec();
        spec.loadProblem(args[0]);
        //Generate the HbvTree from the obstacles
        HBVNode obs = generateHBVTree(spec.getObstacles());
        //Create the graph to hold our valid samples 
        Graph configSpace = new Graph();
        //intialize the sampler from the spec and the hbvtree
        Sampler sampler = new Sampler(spec,obs,configSpace);
        //Create PathGenerator to generate the edges
        PathGenerator pathGenerator = new PathGenerator(configSpace,obs);
        pathGenerator.generateEdges();
        //Initialize the searcher
        Search searcher = new Search(configSpace);
        //Loop Sample then Search until we find a solution
        List<ASVConfig> solution = searcher.searcher();
        while ( solution== null){
        	sampler.sampleConfigSpace();
        	solution = searcher.searcher();
        }
        //Once the solution is found output to file
    }
    
    static private HBVNode generateHBVTree(List<Obstacle> obstacles) {
    	//To construct the HBV tree start by constructing smallest bounding volume for each primitive
    	//FIFO queue represented by a LinkedList to store the nodes
        LinkedList<HBVNode> nodes = new LinkedList<HBVNode>();
        //Point to store the start and end points of segments, and the line primitive for leaf nodes
        Point2D.Double start = null;
        Point2D.Double end = null;
        Line2D.Double primitive;
	    if(obstacles.size()>0){
	        for(Obstacle obs : obstacles){
	            //retrieve obstacle rectangle
	            Rectangle2D rectangle = obs.getRect();
	            //retrieve the PathIterator over the rectangle
	            PathIterator rectanglePath =rectangle.getPathIterator(new AffineTransform());
	            //create the array to store the coordinates
	            double[] coords = new double[6];
	            while(!rectanglePath.isDone()){
	                //retreive current segment coordinates
	                int result = rectanglePath.currentSegment(coords);
	                if(result ==PathIterator.SEG_MOVETO){	
	                	//New Starting point for a segment
	                	start = new Point2D.Double(coords[0], coords[1]);
	                    //create point from the coordinates
	                }
	                if(result == PathIterator.SEG_LINETO){
	                	//New end point for a segment
	                	end = new Point2D.Double(coords[0], coords[1]);
	                	//Create primitive from start to end 
	                	primitive = new Line2D.Double(start, end);
	                	//create primitive HBVNode and add to stack
	                	HBVNode leafNode = new HBVNode(primitive);
	                	nodes.add(leafNode);
	                }
	                rectanglePath.next();
	            }
	        }
            System.out.println(nodes);
			/*
			 * we now have full list of leaf nodes generate the tree from these
			 * Iterate over the list and create a node for every 2 nodes in the list
			 */
	        while(nodes.size()>1){
	            //retrieve the 2 first nodes
	            HBVNode n1 = nodes.remove();
	            HBVNode n2 = nodes.remove();
	            // create a new node as a parent of n1 &n2 with a volume that encompasses both.
	            HBVNode parent = new HBVNode(n1,n2);
	            //add the parent to the FIFO queue
	            nodes.add(parent);
	            
	        }
	        //The last node in the queue contains the whole HBVTree
	        return nodes.get(0);
        }
        return new HBVNode();
    }
    
}
