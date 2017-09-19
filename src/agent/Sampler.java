package agent;

import java.util.ArrayList;
import java.util.List;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Stack;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;


public class Sampler {
    //Bool defining if we have found a solution
    boolean isPathFound;
    //keep track of amount of vertices sampled
    //int counter = 2; //id 0,1 are reserved for init and goal vertices
    //Distance D we are working with ( = max step)
    static final double D = 0.1;
    static final double CHAIR_STEP = 0.001;
    static final double ANGLE_STEP = 0.1*(Math.PI/180);
    static final double LINK_LENGTH = 0.05;
    static final double MIN_JOINT_ANGLE = -150.0 * Math.PI / 180.0;
    static final double MAX_JOINT_ANGLE = 150 * Math.PI / 180;

    //result
    Graph configSpace = new Graph();
    //List of obstacle defining the workspace
    List<Obstacle> obstacles;

    HBVNode hbvTree;

    ProblemSpec spec;

    Search searcher;

    public Sampler(ProblemSpec spec){

        this.spec = spec;
        this.obstacles = spec.getObstacles();


        Vertex start = new Vertex( spec.getInitialState() );
        Vertex end = new Vertex( spec.getGoalState() );


        System.out.println("strat : " + start + " end: " + end + " ");


        configSpace.addLoc(start);
        configSpace.addLoc(end);

        //see if an edge can be generated directly from start to end;
        this.hbvTree = generateHBVTree();

        configSpace.generateEdge(start, hbvTree);

        this.searcher = new Search(configSpace);

    }
    private HBVNode generateHBVTree() {

        Stack<HBVNode> nodes = new Stack<>();

        if(obstacles.size()>0){

            //Iterate over the obstacles
            for(Obstacle obs : obstacles){

                //retrieve obstacle bounds
                Rectangle2D temp = obs.getRect();
                //retrieve the PathIterator over the bounds
                PathIterator tempIt =temp.getPathIterator(new AffineTransform());
                //create the array to store the coordinates
                double[] coords = new double[6];
                //Variable storing the previous coordinates
                Point2D prev = new Point2D.Double();
                while(!tempIt.isDone()){
                    //retrive current segment coordinates
                    tempIt.currentSegment(coords);
                    //create point from the coordinates
                    Point2D p = new Point2D.Double(coords[0],coords[1]);
                    //create the line between the new point and the previous one
                    Line2D l = new Line2D.Double(prev,p);
                    //add the HBV to the list of leaf nodes
                    nodes.push(new HBVNode(l));
                    //set previous to be
                    prev = p;
                    //retrieve the next coordinates
                    tempIt.next();
                }
            }
            System.out.println(nodes);
			/*
			 * we now have full list of leaf nodes generate the tree from these
			 * Iterate over the list and create a node for every 2 nodes in the list
			 */
            while(nodes.size()>1){
                //retrieve the 2 first nodes

                HBVNode n1 = nodes.pop();
                HBVNode n2 = nodes.pop();
                Rectangle2D volume = n1.getVolume().createUnion(n2.getVolume());
                HBVNode parent = new HBVNode(volume);
                parent.addChild(n1);
                parent.addChild(n2);
                nodes.add(0, parent);
            }
            return nodes.get(0);
        }
        return new HBVNode();

    }





    /**
     * Exp3 Sampling stategy implementation
     * P(strat) = (1-n)*(W_strat(t)/SUM(W_strat(t)))+n/k
     * w_strat(t+1)= W_strat(t)+ e^((nr/P(strat))/K)
     * Where:
     * P(strat) = probability of using SamplingStrat strat
     * w(strat)(t) = weight of strat at time t
     * n= fixed UAR
     * K= number of strats = 3
     * r = if Num of components in the roadmap increases/decreases
     *
     *
     *@returns the Graph graph with 10 more samples in it
     */
    public List<ASVConfig> sampleConfigSpace(){
        //Initialise result
        //Graph result = new Graph();
        //Randomly generate n
        double n = Math.random();
        //k = number of strats = 3
        int k =3;
        //r is te rewards
        int r;
        //Initialise the list for the weighted starts
        StratList strats = new StratList();
        //initialise W_strats(0) with 1
        strats.add(new weightedStrat(SamplingStrat.UAR, 1));
        strats.add(new weightedStrat(SamplingStrat.betweenOBS, 1));
        strats.add(new weightedStrat(SamplingStrat.nearOBS, 1));
        for(weightedStrat s: strats){
            //P(strat) = (1-n)*(W_strat(t)/SUM(W_strat(t)))+n/k
            s.setP(((1-n)*(s.getWeight()/strats.getSumOfWeight()))+n/k);
        }

        while(true){
            //int started = counter;
            System.out.println(configSpace);
            List<ASVConfig> path = searcher.searcher();
            purify();
            //System.out.println("searcher found : "+path+" solution");
            //spec.setPath(path);
            isPathFound = !(path.isEmpty());
            if(!isPathFound){
                //Add 10 samples to the graph
                for(int i = 0; i<10;i+=0){
                    weightedStrat s = chooseStrat(strats);// set something different here to be defined when we know what we will do for search
                    Vertex v=null;
                    switch(s.getStrat()){
                        case UAR: v = randomSampling();
                            break;
                        case betweenOBS:v = sampleInsidePassage();
                            break;
                        case nearOBS: v = nearObstacleSampling();
                            break;
                    }
                    r = 0;
                    if(v != null && !configSpace.getLocations().contains(v)){
                        i++;
                        configSpace.addLoc(v);
                        int j = configSpace.generateEdge(v,hbvTree).size();
                        if(j>0)
                            r =1;
                    }
					/*Update the weight of strat with the formula
					 * w(t+1) = w(t)*exp(((n*r)/P(strat))/k)
					 */
                    //look for new edges if num of connected components increases/decreases update the weight of strat
                    strats.get(strats.indexOf(s)).setWeight( s.getWeight()*Math.exp(((n*r)/s.getProb())/k ));
                }

            }else{
                spec.setPath(configSpace.splitValidPath(path));
                return spec.getPath();
            }
        }
    }



    public void purify(){
        List<Vertex>toCheck = new ArrayList<Vertex>(configSpace.getLocations());
        for(Vertex v: toCheck){
            if(v.validIsSet()&& !v.isValid()){
                configSpace.remLoc(v);
            }
        }
    }


    /**
     * Chooses a strategy to use depending on their Probabilities
     * @param strats
     * @return
     */
    public weightedStrat chooseStrat(StratList strats){
        double ran = Math.random();
        double p1 = strats.get(0).getProb();
        double p2 = strats.get(1).getProb();
        double p3 = strats.get(2).getProb();


        if(ran<=p1){
            return strats.get(0);
        }else{
            if(ran<=p1+p2){
                return strats.get(1);
            }else{
                return strats.get(2);
            }
        }
    }



    /**
     *  Samples a valid configuration(i.e no self collisions and the angles are within [min, max] values)
     * @return config q  sampled Uniformely at random
     */
    public Vertex randomSampling(){
        ArrayList<Double> angles = new ArrayList<>();

        //generate random angles
        for(int i = 0; i < spec.getJointCount(); i++){
            angles.add(Math.random());
        }
        //create ArmConfig
        ASVConfig c = new ASVConfig(new Point2D.Double(Math.random(),Math.random()),angles);
        //if the config is not valid, create a new one
        while(!configIsValid(c)){
            angles.clear();
            for(int i = 0; i<spec.getJointCount();i++){
                angles.add(Math.random());
            }
            c = new ASVConfig(new Point2D.Double(Math.random(),Math.random()),angles);
        }
        // return the valid config
        return new Vertex(c);
    }






    /**
     * Samples Uniformely at random a valid config within distance D of c
     * @param c the config from which we are UAR sampling the next
     * @return UAR sampled valid config
     */
    public Vertex randomSamplingFrom(ASVConfig c){
        List<Double> angles = new ArrayList<Double>();
        //generate angles (within step range)
        for(double d:c.getJointAngles()){
            angles.add(d+Math.random()*ANGLE_STEP);
        }
        ASVConfig c1 = new ASVConfig(new Point2D.Double(c.getBaseCenter().getX()+Math.random()*D,c.getBaseCenter().getY()+Math.random()*D),angles);

        while(!configIsValid(c1)){
            angles.clear();
            for(double d:c.getJointAngles()){
                angles.add(d+Math.random()*ANGLE_STEP);
            }
            c1 = new ASVConfig(new Point2D.Double(c.getBaseCenter().getX()+Math.random()*D,c.getBaseCenter().getY()+Math.random()*D),angles);
        }
        return new Vertex(c1);
    }

    /**
     * Tries to sample a config near an obstacle
     * returns null if none of the sampler configs are valid
     * @return  config sampled near an obstacle or null if none found.
     */
    public Vertex nearObstacleSampling(){

        //Choose a sampling q1 randomly from the config space
        Vertex q1 = randomSampling();
        //Sample q2 uniformely at random from the set of all configs withing Distance D and with joint angles within max step
        Vertex q2 = randomSamplingFrom(q1.getC());
        //Check wether the configs are collidng with obstacles.
        boolean q1Valid=configSpace.testConfigCollision(q1, hbvTree),
                q2Valid=configSpace.testConfigCollision(q2, hbvTree);
        //if one of the 2 is  and the other isn't then we have a sampling near an obstacle

        if(!q1Valid&&q2Valid){
            return q2;
        }else if(!q2Valid&&q1Valid){
            return q1;
        }
        //We didnt find a configuration near an obstacle
        return null;
    }


    /**
     * Tries to sample a config between 2 obstacles
     * @return a config sampled between 2 obstacles or null if notne found
     */
    public Vertex sampleInsidePassage(){

        Vertex q1 = randomSampling();
        Vertex q2 = randomSamplingFrom(q1.getC());

        boolean q1Valid=configSpace.testConfigCollision(q1, hbvTree),
                q2Valid=configSpace.testConfigCollision(q2, hbvTree);
        if(q1Valid == false && q2Valid == false){
            double x = (q1.getC().getBaseCenter().getX()+q2.getC().getBaseCenter().getX())/2;
            double y = (q1.getC().getBaseCenter().getY()+q2.getC().getBaseCenter().getY())/2;
            ArmConfig cm = new ArmConfig(new Point2D.Double(x,y),randomSamplingFrom(q1.getC()).getC().getJointAngles());
            boolean cmValid = configSpace.testConfigCollision(new Vertex(cm), hbvTree);
            if(cmValid){
                return new Vertex(cm);
            }
        }
        //We didnt find a valid config
        return null;
    }



    private enum SamplingStrat{
        UAR,nearOBS,betweenOBS;
    }



    private class weightedStrat{
        SamplingStrat strat;
        double p=0;
        double weight;
        private weightedStrat(SamplingStrat strat, double w){
            this.strat = strat;
            this.weight = w;
        }
        public SamplingStrat getStrat() {
            return strat;
        }
        public double getWeight() {
            return weight;
        }
        public void setStrat(SamplingStrat strat) {
            this.strat = strat;
        }
        public void setWeight(double weight) {
            this.weight = weight;
        }
        public double getProb() {
            return p;
        }
        public void setP(double p) {
            this.p = p;
        }

    }




    private class StratList extends ArrayList<weightedStrat>{
        /**
         *
         */
        private static final long serialVersionUID = 251726265965715745L;

        private StratList(){
            super();
        }

        private int getSumOfWeight(){
            int retval=0;
            for(weightedStrat s : this){
                retval+=s.getWeight();
            }
            return retval;
        }
    }

    private boolean configIsValid(ASVConfig c){
        List<Double> jointAngles = c.getJointAngles();
        if(jointAngles.size()==0)
            return false;
        for (Double angle : jointAngles) {
            if (angle <= MIN_JOINT_ANGLE ) {
                return false;
            } else if (angle >= MAX_JOINT_ANGLE ) {
                return false;
            }
        }
        List<Line2D> links = c.getLinks();
        List<Line2D> chair = c.getChair();
        for (int i = 0; i < links.size(); i++) {
            // check for collision between links
            if (c.hasGripper()) {
                //gripper situations
                if (links.size()-i <= 4) {
                    //check gripper collision with joint links
                    for (int j = 0; j < links.size()-5; j++) {
                        if (links.get(i).intersectsLine(links.get(j))) {
                            return false;
                        }
                    }
                } else {
                    //check collision between joint links
                    for (int j = 0; j < i - 1; j++) {
                        if (links.get(i).intersectsLine(links.get(j))) {
                            return false;
                        }
                    }
                }
            } else {
                //non-gripper situations
                for (int j = 0; j < i - 1; j++) {
                    if (links.get(i).intersectsLine(links.get(j))) {
                        return false;
                    }
                }
            }
            // if not first link, check for collision with chair
            if(i > 0) {
                for(int j = 0; j < 4; j++) {
                    if (links.get(i).intersectsLine(chair.get(j))) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

}