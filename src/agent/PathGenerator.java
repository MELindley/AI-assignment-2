package agent;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import problem.ASVConfig;

public class PathGenerator {
	Graph configSpace;
	HBVNode obstacles;
	HashSet<Edge>invalidEdges;
	
	public PathGenerator(Graph configSpace, HBVNode obstacles){
		this.configSpace = configSpace;
		this.obstacles = obstacles;
		invalidEdges = new HashSet<Edge>();
	}
	
	/***
    * Helper function to generate edges between vertices of the config space.
    * @param configspace the config space with to be connected
    * @param obs HBVNode of obstacles
    * @return
    */
   public  List<Edge> generateEdges(){
		//initialize result
		ArrayList<Edge> result = new ArrayList<Edge>();
		//For each Vertex in the graph
		for(Vertex v: this.configSpace.getLocations()){
			for(Vertex v1: this.configSpace.getLocations()){
				//Check that v != v1 and that the edge is not already in the graphs edges or has already been tested and is invalid
				Edge toTest = new Edge(v,v1);
				if(!v.equals(v1)&& !this.configSpace.getEdges().contains(toTest) && ! this.invalidEdges.contains(toTest)){
						//Check that the line is valid 
						 if(checkLineValid(v,v1,this.obstacles,-1,-1)){
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
		return result;
	
	}
   /***
  	 * Checks that a line is valid between two configurations. 
  	 * By Checking every primitive step between them. 
  	 * A Primitive step is valid if: 
  	 * 1. The	length	of	each	broom is	fixed	at	0.05	units	in	length.	
  	 * 2. The	system	must	 form a	connected	chain at	all	 times.
  	 * A connected	chain	means each	ASV	can	be	connected to at	most two brooms and	each end of	each broom is tied to an ASV.
  	 * 3. The polygon  formed by connecting	 the two ends of the connected chain with a straight line segment must,	at	all	times,be convex	and	have an	area of at least	πrmin2,	where	rmin =	0.007(n-1) and n is	the	number	of	ASVs.	
  	 * 4. The brooms must never intersect with each	other.
  	 * 5. Brooms	and	ASVs	must	never intersect	with	obstacles.
  	 * 6. Brooms	&	ASVs	cannot	move	outside	the	[0,1]X[0,1]	workspace.
  	 * 7. The planned path must be given as a sequence of positions (primitive steps) such that	on each	step, each individual ASV moves	by a distance of at	most 0.001 units.	
  	 * 8. Requirements	1-6 must hold at each primitive	step. Since	the	distances are very small (at most 0.001	unit length for	each ASV), it is sufficient to test the
  	 * 	requirements	only	at	the	end	of	each	primitive	step.
  	 * @param v1 Start vertex to check
  	 * @param v2 End vertex to check
  	 * @param obs HBVNode of obstacles
  	 * @return True if the line is valid, false otherwise 
  	 */
  	private boolean checkLineValid(Vertex v1, Vertex v2,HBVNode obs) {
  		
  		ArrayList<ASVConfig> primtiveSteps = 
  	}
  
  /***
   * Generates primitive steps between two vertices by using the following trigonometry:
   * NO VALIDITY CHECK DONE IN THIS FUNCTION
   * @param v1
   * @param v2
   * @return List of ASVConfig describing the primitive steps to take
   */
   private List<ASVConfig> generatePrimitiveSteps(Vertex v1, Vertex v2){
  			
  	}
}
