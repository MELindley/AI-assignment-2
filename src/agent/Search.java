//package agent;
//
//import java.util.*;
//
//import problem.ASVConfig;
//import tester.Tester;
//import java.awt.geom.Point2D;
//
//public class Search {
//    Graph configSpace;
//    ASVConfig start;
//    ASVConfig end;
//
//    public Search(Graph x, ASVConfig start, ASVConfig end ){
//
//        this.configSpace = x;
//        this.start = start;
//        this.end = end;
//    }
//
//    /***
//     * searcher implements a A* algorithm to searhc the config space and return a valid solution path
//     * @return
//     */
//    public List<ASVConfig> searcher() {
//        //create a list to hold the solution if it is found
//        List<ASVConfig> solution = new ArrayList<>();
//
//        //Create the Priority queue using f(n) = g(n)+h(n) as priority, lowest f(n) has highest priority
//        //java.util.PriorityQueue docs:  The head of this queue is the least element with respect to the specified ordering.
//        PriorityQueue<Vertex> toExplore = new PriorityQueue<Vertex>(10, new Comparator<Vertex>() {
//            public int compare(Vertex o1, Vertex o2) {
//                if (o1.getF() < o2.getF()) return -1;
//                if (o1.getF() > o2.getF()) return 1;
//                return 0;
//            }});
//
//        Graph environment = configSpace;
//
//        //Init list for explored nodes
//        HashSet<Vertex> explored = new HashSet<Vertex>();
//        // A map of vertex to vertex, eventually will contain the most efficient
//        // previous step. MAPS: vertex: Optimal Previous vertex
//        HashMap<Vertex, Vertex> path = new HashMap<Vertex, Vertex>();
//        //retrieve start vertex and set it has root of the search tree
//        //initialize its pathCost to 0
//        Vertex start = environment.getVertexByConfig(this.start);
//        start.setPathCost(0);
//
//        //Add start to PQ
//        toExplore.add(start);
//
//        //Main A* loop
//        while (!toExplore.isEmpty()){
//
//            //Takes top node in PQ
//            Vertex current = toExplore.remove();
//
//            //Marks node as explored
//            explored.add(current);
//
//            //Check if current is the goal node
//            if(current.getC().equals(this.end)){
//                //we have found a solution
//                System.out.println("Solution FOund!");
//                solution = buildPath(path,current);
//                return solution;
//            }
//
//            //Iterates through each edge on node
//            for(Edge e: current.getEdges()) {
//
//                double cost = e.getWeight() + current.getPathCost();
//                Vertex neighbour = e.getOther(current);
//
////                if (e.getV2().equals(configSpace.getLocations().get(1)) || e.getV2().equals(configSpace.getLocations().get(1))) {
////                    System.out.println(e.getV1().getC().getBaseCenter()+ " to " + e.getV2().getC().getBaseCenter() + " Cost: " + e.getWeight());
////                }
//
//
//                //Checks that neighbour has not already been explored and that
//                // that neighbour has no shorter path to it.
//                if (!explored.contains(neighbour) && !(e.getV2().getPathCost() < cost)) {
//                    //Calculates heuristic of neighbour vertex and sets it if it is not already set.
//                    if (neighbour.getH() == -1) {
//                        double heuristic = calculateHeuristic(neighbour.getC());
//                        neighbour.setH(heuristic);
//                    }
//
//                    // We have found a better path to neighbour, add or
//                    // update child in the PQ
//                    neighbour.setPathCost(cost);
//                    toExplore.remove(neighbour);
//                    toExplore.add(neighbour);
//                    path.put(neighbour, current);
//                }
//            }
//
//        }
//
//
//        //Prints error message if solution does not exist
//        System.out.println("Solution does not exist.");
//        return null;
//    }
//
//
//	/*
//	 Calculates number of primitive steps needed to reach goal as heuristic
//
//	 Calculations based on the fact that each joint and the chair moves independently.
//
//	 Therefore the number of primitive steps = the highest number of primitive steps taken by a single joint or chair.
//
//	 */
//
//    public double calculateHeuristic(ASVConfig a) {
//
//        int totalH = 0;
//        double tempH = 0;
//
//        ASVConfig vee1 = a;
//        ASVConfig vee2 = configSpace.getLocations().get(1).getC();
//
//        Point2D tempv1 = vee1.getBaseCenter();
//        Point2D tempv2 = vee2.getBaseCenter();
//
//        tempH = Math.abs(tempv1.getY() - tempv2.getY()) + Math.abs(tempv1.getX() - tempv2.getX());
//
//        totalH = (int) (tempH/0.001);
//
//        for (int i=0; i < vee1.getJointCount(); i++) {
//            tempH = Math.abs(vee2.getJointAngles().get(i) - vee1.getJointAngles().get(i));
//
//            if (totalH < (tempH/Tester.MAX_JOINT_STEP)) {
//                totalH = (int) (tempH/Tester.MAX_JOINT_STEP);
//            }
//        }
//
//        return totalH;
//    }
//
//    /**
//     * Build solutuon path using a Map of vertices to the previous best path and the solution vertex
//     * @param path Vertex Map, Mapping Vertex v: Optimal previous vertex
//     * @param curr Goal vertex
//     * @return an arrayList of Vertices containing the path.
//     */
//    private ArrayList<ASVConfig> buildPath(HashMap<Vertex, Vertex> path, Vertex curr) {
//        ArrayList<Vertex> result = new ArrayList<Vertex>();
//        result.add(curr);
//        Vertex i = curr;
//
//        int counter = 1;
//        while (path.keySet().contains(i)) {
//            i = path.get(i);
//            // System.out.println("At Depth "+(counter++)+" Take Vertex: "+i);
//            result.add(i);
//        }
//        Collections.reverse(result);
//        return result;
//    }
//
//
//
//
//}
