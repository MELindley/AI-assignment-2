package agent;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import problem.ASVConfig;
import tester.Tester;

public class PathGenerator {
	Graph configSpace;
	HBVNode obstacles;
	HashSet<Edge>invalidEdges;
	Tester tester;
	
	public PathGenerator(Graph configSpace, HBVNode obstacles, Tester tester){
		this.configSpace = configSpace;
		this.obstacles = obstacles;
		invalidEdges = new HashSet<Edge>();
		this.tester = tester;
	}
	
	/***
    * Helper function to generate edges between vertices of the config space.
	 * Modifies the configspace, edges and vertices directly !
    */
   public  void generateEdges(){
		//initialize result
		ArrayList<Edge> result = new ArrayList<Edge>();
		//For each Vertex in the graph
		for(Vertex v: this.configSpace.getLocations()){
			for(Vertex v1: this.configSpace.getLocations()){
				//Check that v != v1 and that the edge is not already in the graphs edges or has already been tested and is invalid
				Edge toTest = new Edge(v,v1);
				if(!v.equals(v1)&& !this.configSpace.getEdges().contains(toTest) && ! this.invalidEdges.contains(toTest)){
						//Check that the line is valid 
						 if(checkLineValid(toTest,this.obstacles)){
							 this.configSpace.addE(toTest);
							 v.addE(toTest);
							 v1.addE(toTest);
						 }else{
							 //add this edge to the invalid edges
							 this.invalidEdges.add(toTest);
						 }
				}
				 
			}
		}
	
	}
   /***
  	 * Checks that a line is valid between two configurations. 
  	 * By Checking every primitive step between them. 
  	 * A Primitive step is valid if:
  	 * 2. The	system	must	 form a	connected	chain at	all	 times. Ok
  	 * A connected	chain	means each	ASV	can	be	connected to at	most two brooms and	each end of	each broom is tied to an ASV
  	 * 5. Brooms	and	ASVs	must	never intersect	with	obstacles.
  	 * 6. Brooms	&	ASVs	cannot	move	outside	the	[0,1]X[0,1]	workspace.
  	 * 7. The planned path must be given as a sequence of positions (primitive steps) such that	on each	step, each individual ASV moves	by a distance of at	most 0.001 units.	
  	 * 8. Requirements	1-6 must hold at each primitive	step. Since	the	distances are very small (at most 0.001	unit length for	each ASV), it is sufficient to test the
  	 * 	requirements	only	at	the	end	of	each	primitive	step.
  	 * @param toTest Edge to be tested for validity
  	 * @param obs HBVNode of obstacles
  	 * @return True if the line is valid, false otherwise 
  	 */
  	private boolean checkLineValid(Edge toTest,HBVNode obs) {
  		//Generate the valid primitive steps between the two vertices
  		ArrayList<ASVConfig> primitiveSteps = generatePrimitiveSteps(toTest.getV1(),toTest.getV2());
  		//Check for any collisions
  		if(obs.hasCollision(primitiveSteps)){
  			return false;
		}else{
  			//No collisions
			//Update the primitiveSteps in the edge
			toTest.setPrimitiveSteps(primitiveSteps);
			//return
			return true;
		}
  	}
  
  /***
   * Generates primitive steps between two vertices by using the following trigonometry:
   *Each steps is max distance of 0.001
   *Each config created is valid 
   * 1. The	length	of	each	broom is	fixed	at	0.05	units	in	length.
   *  3. The polygon  formed by connecting the two ends of the connected chain with a straight line segment must,
   *  at all	times,be convex	and	have an	area of at least	πrmin2,	where	rmin =	0.007(n-1) and n is	the	number	of	ASVs.
   * 4. The brooms must never intersect with each	other.
   * @param v1
   * @param v2
   * @return List of ASVConfig describing the primitive steps to take
   */
    private ArrayList<ASVConfig> generatePrimitiveSteps(ASVConfig start, ASVConfig goal){
        ArrayList<ASVConfig> steps = new ArrayList<>();
        ArrayList<Double> currentAngles = new ArrayList<>();
        ArrayList<Double> goalAngles = new ArrayList<>();

        ASVConfig currentASV = new ASVConfig(start.getASVCount(), start.toString());


        double maxAngleChange = Math.atan(0.001/0.05);
        double anglesToChange = start.getASVCount() - 1;
        for( int i = 0; i < currentASV.getASVCount() - 1; i++ ){

            currentAngles.add(currentASV.getAngle(i));
            goalAngles.add(start.getAngle(i));

            if( currentAngles.get(i) == goalAngles.get(i) ){
                anglesToChange -= 1;
            }
        }

        maxAngleChange = (maxAngleChange/ anglesToChange);




        while( true ){

        }

        return null;
  	}
}
