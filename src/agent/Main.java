package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.*;

public class Main {
	
	
    public static void main(String args[]) throws IOException {
    	//Load in the problem
        ProblemSpec spec = new ProblemSpec();
        spec.loadProblem(args[0]);
        //Generate the HbvTree from the obstacles
        HBVNode obs = generateHBVTree(spec.getObstacles());
        //Create the graph to hold our valid samples 
        Graph configSpace = new Graph();
        //Create a Tester to test for valid configs
        Tester tester = new Tester();
        //intialize the sampler from the spec and the hbvtree (this also adds start & end ot the graph)
        Sampler sampler = new Sampler(spec,obs,configSpace,tester);
        //Create PathGenerator to generate the edges
        PathGenerator pathGenerator = new PathGenerator(configSpace,obs,tester);
        pathGenerator.generateEdges();
        //Initialize the searcher
        Search searcher = new Search(configSpace, spec.getInitialState(), spec.getGoalState());
        //Loop Sample then Search until we find a solution
        List<ASVConfig> solution = searcher.searcher();

        while ( solution== null){
        	//Sample the config space for new vertices
        	List<Vertex> toAdd = sampler.sampleConfigSpace();
        	//add Vertices to the configSpace
			configSpace.addAllLocations(toAdd);
        	//Create edges between newly created vertices and rest of the graph
			pathGenerator.generateEdges(toAdd);
			//

			//Check for a solution
        	solution = searcher.searcher();
        }
        //Once the solution is found output to file
//        System.out.println(spec.getInitialState());
//        System.out.println(spec.getGoalState());
//        System.out.println(solution);
		spec.setPath(solution);
        spec.saveSolution("solution.txt");
        tester.setProblemSpec(spec);
        List<String> testsToRun = new ArrayList<String>();
        testsToRun.addAll(Arrays.asList(new String[] { "initial", "goal",
                    "steps", "cost" }));
        testsToRun.addAll(Arrays.asList(new String[] { "booms", "convexity",
                "areas", "bounds", "collisions" }));
        int testNo = 1;
        int numFailures = 0;
        for (String name : testsToRun) {
            if (!tester.testByName(name, testNo, true)) {
                numFailures++;
            }
            testNo++;
        }
        System.out.print("Number of failures: "+numFailures);
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
	        for(Obstacle obs : obstacles) {
                //retrieve obstacle rectangle
                Rectangle2D rectangle = obs.getRect();
                //retrieve the PathIterator over the rectangle
                HBVNode leafNode = new HBVNode(rectangle);
                nodes.add(leafNode);
            }
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
