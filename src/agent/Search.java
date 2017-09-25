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

}
