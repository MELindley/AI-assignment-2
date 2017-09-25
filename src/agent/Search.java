package agent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import problem.ASVConfig;
import tester.Tester;
import java.awt.geom.Point2D;

public class Search {
    Graph x;

    public Search(Graph x){
        this.x = x;
    }

    public List<ASVConfig> searcher() {

        List<ASVConfig> solution = new ArrayList<>();

        //Priority queue init
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>(10, new Comparator<Vertex>() {
            public int compare(Vertex o1, Vertex o2) {
                if (o1.getF() < o2.getF()) return -1;
                if (o1.getF() > o2.getF()) return 1;
                return 0;
            }});

        Graph environment = x;

        //Init list for explored nodes
        List<Vertex> explored = new ArrayList<Vertex>();

        //Set starting point initial cost to 0
        environment.getLocations().toArray(V)  .setPathCost(0);

        //Add root to PQ
        queue.add(environment.getLocations().get(0));

        //Main PQ Loop
        while (!queue.isEmpty()){

            //Takes top node in PQ
            Vertex current = queue.poll();

            //Runs if solution found
            if(current.equals(x.getLocations().get(1))){
                System.out.println("Solution FOund!");
                solution = resultBuilder(current);
                return solution;
            }

            //Marks node as explored
            explored.add(current);

            //Iterates through each edge on node
            for(Edge e: current.getEdges()){

                double cost = e.getWeight() + current.getPathCost();


                if (e.getV2().equals(x.getLocations().get(1)) || e.getV2().equals(x.getLocations().get(1))) {
                    System.out.println(e.getV1().getC().getBaseCenter()+ " to " + e.getV2().getC().getBaseCenter() + " Cost: " + e.getWeight());
                }


                //Checks if destination already explored or if there is a shorter path to destination
                if (explored.contains(e.getV2()) || e.getV2().getPathCost() < cost) {
                    continue;
                }

                //Calculates heuristic of next vertex and sets it if it is not already set.
                if(e.getV2().getH() == -1) {
                    double heuristic = calculateHeuristic(e.getV2().getC());
                    e.getV2().setH(heuristic);
                }

                //Sets destination parent to current if if destination has no parent or destination is not current's parent
                if ((e.getV2().getParent() == null)
                        || !(current.getParent().equals(e.getV2()))) {
                    e.getV2().setParent(current);
                }

                //Removes queue entry if it already exists
                if(queue.contains(e.getV2())) {
                    queue.remove(e.getV2());
                }

                //Adds destination to queue
                queue.add(e.getV2());

                //Updates destination total cost
                e.getV2().setPathCost(cost);
            }

        }


        //Prints error message if solution does not exist
        System.out.println("Solution does not exist.");
        return null;
    }


	/*
	 Calculates number of primitive steps needed to reach goal as heuristic

	 Calculations based on the fact that each joint and the chair moves independently.

	 Therefore the number of primitive steps = the highest number of primitive steps taken by a single joint or chair.

	 */

    public double calculateHeuristic(ASVConfig a) {

        int totalH = 0;
        double tempH = 0;

        ASVConfig vee1 = a;
        ASVConfig vee2 = x.getLocations().get(1).getC();

        Point2D tempv1 = vee1.getBaseCenter();
        Point2D tempv2 = vee2.getBaseCenter();

        tempH = Math.abs(tempv1.getY() - tempv2.getY()) + Math.abs(tempv1.getX() - tempv2.getX());

        totalH = (int) (tempH/0.001);

        for (int i=0; i < vee1.getJointCount(); i++) {
            tempH = Math.abs(vee2.getJointAngles().get(i) - vee1.getJointAngles().get(i));

            if (totalH < (tempH/Tester.MAX_JOINT_STEP)) {
                totalH = (int) (tempH/Tester.MAX_JOINT_STEP);
            }
        }

        return totalH;
    }

    //Build the result of the search using parent values
    public static List<ASVConfig> resultBuilder(Vertex dest) {
        List<ASVConfig> built = new ArrayList<ASVConfig>();
        for (Vertex i = dest; i != null; i = i.getParent()) {
            built.add(i.getC());
        }

        Collections.reverse(built);

        return built;
    }
    
    /***
     * Helper function to generate edges between vertices of the config space.
     * @param configspace the config space with to be connected
     * @param obs HBVNode of obstacles
     * @return
     */
    private static List<Edge> generateEdges(Graph configspace, HBVNode obs){
		//initialize result
		ArrayList<Edge> result = new ArrayList<Edge>();
		//For each Vertex in the graph
		for(Vertex v: configspace.getLocations()){
			for(Vertex v1: configspace.getLocations()){
				//Check that v != v1 and that the edge is not already in the graphs edges or has already been tested and is invalid
				Edge toTest = new Edge(v,v1);
				if(!v.equals(v1)&& !configspace.getEdges().contains(toTest) && ! invalidEdges.contains(toTest)){
						//Check that the line is valid 
						 if(checkLineValid(v,v1,obs,-1,-1)){
							 configspace.addE(toTest);
							 v.addE(toTest);
							 v1.addE(toTest);
						 }else{
							 //add this edge to the invalid edges
							 invalidEdges.add(toTest);
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
   5. Brooms	and	ASVs	must	never intersect	with	obstacles.
   6. Brooms	&	ASVs	cannot	move	outside	the	[0,1]X[0,1]	workspace.
   7. The	 planned	 path	 must	 be	 given	 as	 a	 sequence	 of	 positions	 (primitive	
   steps)	such	that	on	each	step,	each	individual	ASV	moves	by	a	distance	of	
   at	most	0.001	units.	
   8. Requirements	1-6 must	hold	at	each	primitive	step.	Since	the	distances	are	
   very	small	(at	most	0.001	unit	length	for	each	ASV),	it	is	sufficient to	test	
   the	requirements	only	at	the	end	of	each	primitive	step.
   	 * @param v1 Start vertex to check
   	 * @param v2 End vertex to check
   	 * @param obs HBVNode of obstacles
   	 * @return True if the line is valid, false otherwise 
   	 */
   	private boolean checkLineValid(Vertex v1, Vertex v2,HBVNode obs) {
   		
   		ArrayList<ASVConfig> primtiveSteps = 

}
