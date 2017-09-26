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
    static final double SAMPLING_DISTANCE = 10*PRIMITIVE_STEP;
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
            //P(strat) = (1-n)*(W_strat(t)/SUM(W_strat(t)))+(n/k)
            s.setProb(((1-n)*(s.getWeight()/this.strategies.getSumOfWeight()))+n/k);
        }
        //initialise the tester
        this.tester = tester;

        System.out.println("strat : " + start + " end: " + end + " ");

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


            WeightedSamplingStrategy s = chooseStrategy(this.strategies);
            Vertex v = null;
            switch (s.getStrat()) {
                case UAR:
                    v = randomSampling();
                    break;
                case betweenOBS:
                    v = sampleInsidePassage();
                    break;
                case nearOBS:
                    v = nearObstacleSampling();
                    break;
            }
            //DO VALIDITY CHECK ON V
            int r = 0;
            if(tester.hasEnoughArea(v.getC())&& tester.fitsBounds(v.getC())
                    && tester.isConvex(v.getC()) && tester.hasValidBoomLengths(v.getC())
                    && !this.hbvTree.hasCollision(v.getC())) {
                // The vertex is valid
                // add the vertex to the return value
                retval.add(v);
                //Set r to 1
                r =1;
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
        	double y = coordinates[(i-1) * 2 - 1] + yOff;
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
            double y = coordinates[(i-1) * 2 - 1] + yOff;
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
        boolean q1Valid=hbvTree.hasCollision(q1.getC()),
                q2Valid=hbvTree.hasCollision(q2.getC());
        //if one of the 2 is  and the other isn't then we have a sampling near an obstacle

        if(!q1Valid&&q2Valid){
            return q2;
        }else if(!q2Valid&&q1Valid){
            return q1;
        }
        //We didnt find a configuration near an obstacle
        return null;
    }

    public Vertex nearObstacleSampling(Vertex collision){

        //Choose a sampling q1 randomly from the config space

        //Sample q2 uniformely at random from the set of all configs withing Distance D and with joint angles within max step
        Vertex q2 = randomSamplingFrom(collision.getC());
        // return q2
        return q2;
    }

    /**
     * Tries to sample a config between 2 obstacles
     * @return a config sampled between 2 obstacles or null if notne found
     */
    public Vertex sampleInsidePassage(){

        Vertex q1 = randomSampling();
        Vertex q2 = randomSamplingFrom(q1.getC());

//        boolean q1Valid=configSpace.testConfigCollision(q1, hbvTree),
//                q2Valid=configSpace.testConfigCollision(q2, hbvTree);
//        if(q1Valid == false && q2Valid == false){
//            double x = (q1.getC().getBaseCenter().getX()+q2.getC().getBaseCenter().getX())/2;
//            double y = (q1.getC().getBaseCenter().getY()+q2.getC().getBaseCenter().getY())/2;
//            ArmConfig cm = new ArmConfig(new Point2D.Double(x,y),randomSamplingFrom(q1.getC()).getC().getJointAngles());
//            boolean cmValid = configSpace.testConfigCollision(new Vertex(cm), hbvTree);
//            if(cmValid){
//                return new Vertex(cm);
//            }
//        }
        //We didnt find a valid config
        return null;
    }

    public Vertex sampleInsidePassage(Vertex collision1, Vertex collision2){
        //Create new vertex between collision1 and collision 2
      return null;
    }



}