package agent;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;
import java.util.List;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;


public class Sampler {
    //Bool defining if we have found a solution
    boolean isPathFound;

    static final double PRIMITIVE_STEP = 0.001;
    static final double BROOM_LENGTH = 0.05;
    static final double SAMPLING_DISTANCE = 10*PRIMITIVE_STEP;
    double maxArea;

    Graph configSpace;
    //List of obstacle defining the workspace
    HBVNode hbvTree;

    ProblemSpec spec;

//    Search searcher;
    
    Tester tester;

    //List of sampling strategies to use
    SamplingStrategyList strategies;
    double n;
    int k;
    boolean rotatesLeft;
  
    public Sampler(ProblemSpec spec, HBVNode obs, Graph graph, Tester tester){
    	//retrieve obstacles from spec
        this.spec = spec;
        this.hbvTree = obs;
        //Create Vertices for initial & goal state
        this.rotatesLeft = rotatesLeft(spec.getInitialState(),0);
        Vertex start = new Vertex( spec.getInitialState() );
        Vertex end = new Vertex( spec.getGoalState() );
        this.configSpace = graph;
        //add vertices to the config space 
        this.configSpace.addLoc(start);
        this.configSpace.addLoc(end);
        //Set the polygon max area according to pi*(0.007*n-1)^2
        this.maxArea = Math.PI*Math.pow(0.007*start.getC().getASVCount()-1,2);
        //Create sampling strategy list and initialize the strategies
        this.strategies = new SamplingStrategyList();
        this.strategies.add(new WeightedSamplingStrategy(SamplingStrategy.UAR, 1));
        //Give initial random weight to strategies
        this.n = Math.random();
        //k = number of strats = 3
        this.k =3;
        for(WeightedSamplingStrategy s: this.strategies){
            //P(strat) = (1-n)*(W_strat(t)/SUM(W_strat(t)))+(n/k)
            s.setProb(((1-n)*(s.getWeight()/this.strategies.getSumOfWeight()))+n/k);
        }
        //initialise the tester
        this.tester = tester;

    }






    /**
     * Exp3 Sampling stategy implementation
     * P(strategy) = (1-n)*(strategy.weight(t)/SUM(strategy.weight(t)))+n/k
     * strategy.weight(t+1)= strategy.weight(t)+ e^((nr/P(strategy))/K)
     * Where:
     * P(strategy) = probability of using SamplingStrat strat
     * strategy.weight(t) = weight of strat at time t
     * n= fixed UAR
     * K= number of strats = 3
     * r = if Num of components in the roadmap increases/decreases
     *
     *
     *@returns the valid configs out of the 100 generated
     */
    public ArrayList<Vertex> sampleConfigSpace(){
        ArrayList<Vertex> retval = new ArrayList<Vertex>();
  
        //Sample a 100 times before adding to the graph
        for(int i = 0; i<100;i+=1) {
            //The way we choose the strat might need to change
            //having a randomSampling first (check for validity)
            // then checking for collisions with obstacles of the random sample
            //if collision then try sampling close to obstacle
            //if collision on second sample then try sampling in passage (rather then choosing strat based on weight


            WeightedSamplingStrategy s = strategies.get(0); //Hardcoded to UAR atm
            Vertex v = randomSampling();

            //DO VALIDITY CHECK ON V
            int r = 0;
            if(validityCheck(v.getC())) {
                //Check that the Configuration is not in collision
                if(hbvTree.hasCollision(v.getC())) {
                    //Do a near obstacle sample
                    Vertex nearObstacle = nearObstacleSampling(v);
                    //Do validity Check
                    if (validityCheck(nearObstacle.getC())) {
                        //Check wether nearObstacle is in collision
                        if (hbvTree.hasCollision(nearObstacle.getC())) {
                            //We hve two configuration in collision. Do a inside passage sample
                            Vertex inPassage = sampleInsidePassage(v, nearObstacle);
                            //Do validity Check and check for no collision
                            if (validityCheck(inPassage.getC()) && !hbvTree.hasCollision(inPassage.getC())) {
                                //The sample inside passage is valid
                                r = 1;
                                retval.add(inPassage);
                            }
                        } else {
                            //the near obstacle sample is not in collision and valid
                            // add the nearObstacle vertex to the return value
                            r = 1;
                            retval.add(nearObstacle);
                        }
                    }

                }else{
                    //V is valid add it to the return value
                    retval.add(v);
                    //Set r to 1
                    r = 1;
                }
            }
            //update the weight of the strat
            this.strategies.get(this.strategies.indexOf(s)).setWeight(
                    s.getWeight()*Math.exp(((n*r)/s.getProb())/k));

        }
        //update the probabilities once the weights are updated
        for(WeightedSamplingStrategy s: this.strategies){
            //P(strat) = (1-n)*(W_strat(t)/SUM(W_strat(t)))+n/k
            s.setProb(((1-n)*(s.getWeight()/this.strategies.getSumOfWeight()))+n/k);
        }
        return retval;
    }

    /**
     * helper function checking the config for validity
     * @param c the config to check
     * @return True if the config is valid ; false otherwise
     */
    public boolean validityCheck(ASVConfig c){
        return tester.hasEnoughArea(c)&& tester.fitsBounds(c)
                && tester.isConvex(c) && tester.hasValidBoomLengths(c)&& (rotatesLeft(c,0) == this.rotatesLeft);
    }

    /**
     * Gets the orientation of the angle
     * https://stackoverflow.com/questions/22668659/calculate-on-which-side-of-a-line-a-point-is
     * @param c
     * @return true if the asv is orientated to the left:
     *   |
     * --
     * False if the asv is orientated to the right
     * --
     *   |
     */
    public boolean rotatesLeft(ASVConfig c,int i) {
        List<Point2D> points = c.getASVPositions();
        Point2D p0 = points.get(i);
        Point2D p1 = points.get(i+1);
        Point2D p2 = points.get(i+2);

        double value = (p1.getX() - p0.getX())*(p2.getY() - p0.getY()) - (p2.getX() - p0.getX())*(p1.getY() - p0.getY());
        if(value == 0){
            return rotatesLeft(c,i++);
        }else{
            if(value<0){
                return false;
            }else{
                return true;
            }
        }
    }

    /**
     * Chooses a strategy to use depending on their Probabilities
     * @param strategies
     * @return
     */
    public WeightedSamplingStrategy chooseStrategy(SamplingStrategyList strategies){
        double ran = Math.random();
        double p1 = strategies.get(0).getProb();
        double p2 = strategies.get(1).getProb();
        double p3 = strategies.get(2).getProb();


        if(ran<=p1){
            return strategies.get(0);
        }else{
            if(ran<=p1+p2){
                return strategies.get(1);
            }else{
                return strategies.get(2);
            }
        }
    }



    /**
     *  Samples a valid configuration i.e
     *  1. The length of each broom is fixed at 0.05 units in length.
     *  2. The system must form a connected chain at all times.
     *  A connected chain means each ASV can be connected to at most two brooms and each end of each broom is tied to an ASV.
     *  3. The polygon formed by connecting the two ends of the connected chain with a straight line segment must,
     *  at all times, be convex and have an area of at least πrmin^2, where rmin = 0.007(n-1) and n is the number of ASVs.
     *  4. The brooms must never intersect with each other.
     *  6. Brooms & ASVs cannot move outside the [0,1]X[0,1] workspace.

     * @return config q  sampled Uniformely at random
     */
    public Vertex randomSampling(){
        ArrayList<Double> angles = new ArrayList<>();
        //retrieve number of ASV
        int ASVcount = spec.getASVCount();
        //generate 1 position for first ASV randomly
        double []coordinates= new double [ASVcount*2];
        coordinates[0]= Math.random();
        coordinates[1] = Math.random();
        //Min angle is 360/ASVCoun && max angle is 180 so generate from that then check
        //range  = 0 - 180-(360/AStVCount)
    	double minAngle= 2*Math.PI/ASVcount;
    	double range = Math.PI-minAngle;

        //For each remaining ASV
        for(int i = 1; i<ASVcount;i++){
        	//generate next position for ASV with distance (previous, current) = 	0.05
        	double nextX,nextY;
        	double angle = ((range*Math.random())+minAngle)*2*Math.PI;

        	//retrieve offset for configSpace & y
        	double xOff = Math.cos(angle)*BROOM_LENGTH;
        	double yOff = Math.sin(angle)*BROOM_LENGTH;

        	//add offsest to previous ASV position
        	double x = coordinates[(i-1) * 2] + xOff;
        	double y = coordinates[(i-1) * 2 + 1] + yOff;
        	coordinates[i * 2] = x;
        	coordinates[i * 2 + 1] = y;
        }
        //create ASVConfig
        //Once generated check that the area is <  pi*(0.007*count)^2
        ASVConfig totest = new ASVConfig(coordinates);
        //return to test in a new vertex
        //THE VERTEX NEEDS TO BE CHECKED FOR VALIDITY !
        return new Vertex(totest);

    }






    /**
     * Samples Uniformely at random a valid config within distance D of c
     * @param c the config from which we are UAR sampling the next
     * @return UAR sampled valid config
     */
    public Vertex randomSamplingFrom(ASVConfig c){
        //retrieve number of ASV
        int ASVcount = spec.getASVCount();
        //generate 1 position for first ASV randomly within D distance from c
        //A  better "random" function is used see http://www.anderswallin.net/2009/05/uniform-random-points-in-a-circle-using-polar-coordinates/
        double r =  SAMPLING_DISTANCE*Math.sqrt(Math.random());
        double theta =2*Math.PI*Math.random();

        double []coordinates= new double [ASVcount*2];
        coordinates[0]= r*Math.cos(theta);
        coordinates[1] = r*Math.sin(theta);
        //Min angle is 360/ASVCoun && max angle is 180 so generate from that then check
        //range  = 0 - 180-(360/AStVCount)
        double minAngle= 2*Math.PI/ASVcount;
        double range = Math.PI-minAngle;
        //For each remaining ASV

        for(int i = 1; i<ASVcount;i++){
            //generate next position for ASV with distance (previous, current) = 	0.05
            double nextX,nextY;
            double angle = ((range*Math.random())+minAngle)*2*Math.PI;
            //retrieve offset for configSpace & y
            double xOff = Math.cos(angle)*BROOM_LENGTH;
            double yOff = Math.sin(angle)*BROOM_LENGTH;
            //add offsest to previous ASV position
            double x = coordinates[(i-1) * 2] + xOff;
            double y = coordinates[(i-1) * 2 + 1] + yOff;
            coordinates[i * 2] = x;
            coordinates[i * 2 + 1] = y;
        }
        //create ASVConfig
        //Once generated check that the area is <  pi*(0.007*count)^2
        ASVConfig totest = new ASVConfig(coordinates);
        //return to test in a new vertex
        //THE VERTEX NEEDS TO BE CHECKED FOR VALIDITY !
        return new Vertex(totest);
    }

    /**
     * Samples a new configuration near a configuration in collision
     * @param collision
     * @return
     */
    public Vertex nearObstacleSampling(Vertex collision){

        //Sample q2 uniformely at random from the set of all configs withing Distance D of collision
        Vertex q2 = randomSamplingFrom(collision.getC());
        // return q2
        return q2;
    }


    /**
     * Sample between 2 configs that are in collision
     * @param collision1
     * @param collision2
     * @return
     */
    public Vertex sampleInsidePassage(Vertex collision1, Vertex collision2){
        //Create new vertex between collision1 and collision 2
        int asvCount = collision1.getC().getASVCount();
        //Create the new vertex in the middle of the 2 others.
        //In this case we define middle as the middle of the average distance between the Two ASV
        double Avgdistance = collision1.getC().totalDistance(collision2.getC())/asvCount;
        //Retrieve ASV Positions
        List<Point2D> positions1 = collision1.getC().getASVPositions();
        List<Point2D> positions2 = collision2.getC().getASVPositions();
        //to find the middle config, draw lines between each asv
        // 1) Start on middle point of the line between two first ASV of each configuration
        //2) From there create a cricle from that point with r = BROOM_LENGTH
        //3) Then next point is the intersection between the line between the i+1 asv of each config and the circle
        //Repeat from 2 until done
        /* Like below
            /-----/-----/
            |---- |-----|
            \-----\-----|
             \-----\----\
         */
        //create coordinate
        ArrayList<Double> coordinates= new ArrayList<Double>();
        //Start on middle point of line between two first ASVs
        Point2D a = positions1.get(0);
        Point2D b = positions2.get(0);

        //create first generated asv in the middle of the two points
        Point2D generatedASV = new Point2D.Double((a.getX()+b.getX())/2, (a.getY()+b.getY())/2);
        Line2D line;
        coordinates.add( generatedASV.getX());
        coordinates.add(generatedASV.getY());



        //Now loop on the remaining avs
        for (int i = 1; i<asvCount; i++){
            a =positions1.get(i);
            b = positions2.get(i);
            line = new Line2D.Double(a,b);
            //circle goes from generated ASV
            //A circle is an ellipse with h= w in this case h=w=BROOM_LENGTH
            Ellipse2D circle = new Ellipse2D.Double(generatedASV.getX()-BROOM_LENGTH,generatedASV.getY()-BROOM_LENGTH,
                    BROOM_LENGTH,BROOM_LENGTH);
            if (line.intersects(circle.getFrame())){
                //retrieve intersections, list of possible next point
                ArrayList<Point2D> intersections = getIntersection(a.getX(),b.getX(),a.getY(),
                        b.getY(),generatedASV.getX(),generatedASV.getY());
                //Keep the one that makes the asv Valid
                generatedASV = intersections.get(0);
                coordinates.add(generatedASV.getX());
                coordinates.add(generatedASV.getY());
                //Generate an asv and test it for validity if valid keep it if not generate the next and hope for the best.
                double[] temp_coords = new double[coordinates.size()];
                i = 0;
                for(Double x: coordinates){
                    temp_coords[i++]= x.doubleValue();
                }
                ASVConfig toTest = new ASVConfig(temp_coords);
                if(!validityCheck(toTest)){
                    //Try other one
                    //remove values from coordinates
                    coordinates.remove(coordinates.size()-1) ;
                    coordinates.remove(coordinates.size()-1) ;
                    //get the other point
                    generatedASV = intersections.get(1);
                    coordinates.add(generatedASV.getX());
                    coordinates.add(generatedASV.getY());
                    //keep going from here
                }

            }else{
                //no collisions with the line? Gotta figure out a way to deal with this possibly ?
                System.out.print("NO COLLISION BETWEEN LINE AND CIRCLE !");
            }
        }
        double[] coords = new double[asvCount*2];
        int i =0;
        for(Double val:coordinates){
            coords[i++]=val;
        }
        return new Vertex(new ASVConfig(coords));
    }


    public static ArrayList<Point2D> getIntersection(double ax, double bx, double ay, double by, double asvX, double asvY) {
        ArrayList<Point2D> points = new ArrayList<Point2D>();

        ax -= asvX;
        ay -= asvY;

        bx -= asvX;
        by -= asvY;

        if (ax == bx) {
            double y = Math.sqrt(BROOM_LENGTH*BROOM_LENGTH-ax*ax);
            if (Math.min(ay, by) <= y && y <= Math.max(ay, by)) {
                points.add(new Point2D.Double(ax+asvX, y+asvY));
            }
            if (Math.min(ay, by) <= -y && -y <= Math.max(ay, ay)) {
                points.add(new Point2D.Double(ax+asvX, -y+asvY));
            }
        }
        else {
            double a = (by - by) / (bx - ax);
            double b = (ay - a*ax);

            double r = a*a*BROOM_LENGTH*BROOM_LENGTH + BROOM_LENGTH*BROOM_LENGTH;
            double s = 2*a*b*BROOM_LENGTH*BROOM_LENGTH;
            double t = BROOM_LENGTH*BROOM_LENGTH*b*b - BROOM_LENGTH*BROOM_LENGTH*BROOM_LENGTH*BROOM_LENGTH;

            double d = s*s - 4*r*t;

            if (d > 0) {
                double xi1 = (-s+Math.sqrt(d))/(2*r);
                double xi2 = (-s-Math.sqrt(d))/(2*r);

                double yi1 = a*xi1+b;
                double yi2 = a*xi2+b;

                if (isPointInLine(ax, bx, ay, by, xi1, yi1)) {
                    points.add(new Point2D.Double(xi1+asvX, yi1+asvY));
                }
                if (isPointInLine(ax, bx, ay, by, xi2, yi2)) {
                    points.add(new Point2D.Double(xi2+asvX, yi2+asvY));
                }
            }
            else if (d == 0) {
                double xi = -s/(2*r);
                double yi = a*xi+b;

                if (isPointInLine(ax, bx, ay, by, xi, yi)) {
                    points.add(new Point2D.Double(xi+asvX, yi+asvY));
                }
            }
        }

        return points;
    }

    public static boolean isPointInLine(double x1, double x2, double y1, double y2, double px, double py) {
        double xMin = Math.min(x1, x2);
        double xMax = Math.max(x1, x2);

        double yMin = Math.min(y1, y2);
        double yMax = Math.max(y1, y2);

        return (xMin <= px && px <= xMax) && (yMin <= py && py <= yMax);
    }


}