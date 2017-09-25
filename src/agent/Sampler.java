package agent;

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
    double maxArea;

    Graph configSpace;
    //List of obstacle defining the workspace
    HBVNode hbvTree;

    ProblemSpec spec;

    Search searcher;
    
    Tester tester;

    //List of sampling strategies to use
    SamplingStrategyList strategies;
    double n;
    int k;
  
    public Sampler(ProblemSpec spec, HBVNode obs, Graph graph, Tester tester){
    	//retrieve obstacles from spec
        this.spec = spec;
        this.hbvTree = obs;
        //Create Vertices for initial & goal state
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
        this.strategies.add(new WeightedSamplingStrategy(SamplingStrategy.betweenOBS, 1));
        this.strategies.add(new WeightedSamplingStrategy(SamplingStrategy.nearOBS, 1));
        //Give initial random weight to strategies
        this.n = Math.random();
        //k = number of strats = 3
        this.k =3;
        for(WeightedSamplingStrategy s: this.strategies){
            //P(strat) = (1-n)*(W_strat(t)/SUM(W_strat(t)))+n/k
            s.setProb(((1-n)*(s.getWeight()/this.strategies.getSumOfWeight()))+n/k);
        }

        System.out.println("strat : " + start + " end: " + end + " ");

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
     *@returns the valid configs out of the 100 generated
     */
    public List<Vertex> sampleConfigSpace(){
        ArrayList<Vertex> retval = new ArrayList<Vertex>();
  
        //Add 100 samples to the graph
        for(int i = 0; i<100;i+=1){
            WeightedSamplingStrategy s = chooseStrategy(this.strategies);
            Vertex v=null;
            switch(s.getStrat()){
                case UAR: v = randomSampling();
                    break;
                case betweenOBS:v = sampleInsidePassage();
                    break;
                case nearOBS: v = nearObstacleSampling();
                    break;
            }
            int r = 0;
            //NEED TO CHECK FOR VALIDITY
            if(v != null && !configSpace.getLocations().contains(v)){
                retval.add(v);
                if(j>0)
                    r =1;
            }
			/*Update the weight of strat with the formula
			 * w(t+1) = w(t)*exp(((n*r)/P(strat))/k)
			 */
            //look for new edges if num of connected components increases/decreases update the weight of strat
            this.strategies.get(this.strategies.indexOf(s)).setWeight( s.getWeight()*Math.exp(((n*r)/s.getProb())/k ));
        }
        //update the probabilities
        for(WeightedSamplingStrategy s: this.strategies){
            //P(strat) = (1-n)*(W_strat(t)/SUM(W_strat(t)))+n/k
            s.setProb(((1-n)*(s.getWeight()/this.strategies.getSumOfWeight()))+n/k);
        }

//    }else{
//        spec.setPath(configSpace.splitValidPath(path));
//        return spec.getPath();
//    }
//        }
        return
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
     *  Samples a valid configuration(i.e no self collisions and the angles are within [min, max] values)
     * @return config q  sampled Uniformely at random
     */
    public Vertex randomSampling(){
        ArrayList<Double> angles = new ArrayList<>();
        //retrieve number of ASV
        int count = spec.getASVCount();
        //generate 1 position for ASV
        double []coordinates= new double [count*2];
        coordinates[0]= Math.random();
        coordinates[1] = Math.random();
        //Min angle is 360/ASVCount && max angle is 180 so generate from that then check
    	//range  = 0 - 180-(360/ASVCount)
    	double minAngle= 2*Math.PI/count;
    	double range = Math.PI-minAngle;
    	double distance = 0.05;
        //For each remaining ASV
        for(int i = 1; i<count;i++){
        	//generate next position for ASV with distance (previous, current) = 	0.05 
        	double nextX,nextY;
        	double angle = ((range*Math.random())+minAngle)*2*Math.PI;
        	//retrieve offset for configSpace & y
        	double xOff = Math.cos(angle)*distance;
        	double yOff = Math.sin(angle)*distance;
        	//add offsest to previous ASV position
        	double x = coordinates[(i-1) * 2] + xOff;
        	double y = coordinates[(i-1) * 2 - 1] + yOff;
        	coordinates[i * 2] = x;
        	coordinates[i * 2 + 1] = y;
        }
        //Once generated check that the area is <  pi*(0.007*count)^2 
        ASVConfig totest = new ASVConfig(coordinates);
        //use Tester.tester.hasEnoughtArea to see if its valid
        		
        
        //create ASVConfig
        
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