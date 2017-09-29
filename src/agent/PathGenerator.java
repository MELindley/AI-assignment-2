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
	 * Generates edges from a list of vertices that have just been added
	 * @param justAdded list of vertices that have just been to avoid iterating over the whole graph twice
	 */
	public  void generateEdges(List<Vertex>justAdded){
		//initialize result
		ArrayList<Edge> result = new ArrayList<Edge>();
		//For each Vertex in the graph
		for(Vertex v: justAdded){
			for(Vertex v1: this.configSpace.getLocations()){
				//Check that v != v1 and that the edge is not already in the graphs edges or has already been tested and is invalid
				Edge toTest = new Edge(v,v1);
				if(!v.equals(v1)&& !this.configSpace.getEdges().contains(toTest) && ! this.invalidEdges.contains(toTest)){
					//Check that the line is valid
					if(checkLineValid(toTest,this.obstacles)){
						this.configSpace.addE(toTest);
						v.addE(toTest);
						v1.addE(toTest);
						//We connected a new edge with this vertex, need to update the Sampling strategy weight used to generate v
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
  		ArrayList<ASVConfig> primitiveSteps = generatePrimitiveSteps(toTest.getV1().getC(),toTest.getV2().getC());
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
   * @param start
   * @param goal
   * @return List of ASVConfig describing the primitive steps to take
   */
    public ArrayList<ASVConfig> generatePrimitiveSteps(ASVConfig start, ASVConfig goal){

        System.out.println(start);
        assert(validityCheck(start));
        System.out.println(goal);
        assert(validityCheck(goal));


        Double maxStep = 0.001;
        Double broomLength = 0.05;

        ArrayList<ASVConfig> steps = new ArrayList<>();
        ArrayList<Double> currentAngles = new ArrayList<>();
        ArrayList<Double> goalAngles = new ArrayList<>();

        ASVConfig currentASV = new ASVConfig(start);

        //Underestimate the maxium angle line based on a straight asv config
        //https://math.stackexchange.com/questions/541824/how-do-i-find-the-base-angles-without-a-vertex-angle-in-a-isosceles-triangle


        double maxAngleChange = 2 *  Math.asin( (maxStep)/2.0 / ( broomLength * (start.getASVCount() - 1) ) );

        double changeInX;
        double changeInY;

        for( int i = 0; i < currentASV.getASVCount() - 1; i++ ){
            currentAngles.add(currentASV.getAngle(i));
            goalAngles.add(goal.getAngle(i));

            System.out.println("Current " + i + ":" + start.getAngle(i));
            System.out.println("Goal    " + i + ":" + goal.getAngle(i));
            System.out.println("   ");

        }

        while( true ){

            for(int i = 0; i < start.getASVCount(); i++ ){

                //If the angle is not the desired angle
                if(!currentAngles.get(i).equals(goalAngles.get(i))){
                    System.out.println("Current angle   :" + currentAngles.get(i) );
                    System.out.println("Goal angle      :" + goalAngles.get(i) );

                    //change angle between i to i+1

                    double angleDiff;
                    double angleChange;
                    double newAngle;
                    int plusMinus = 0;

                    //trying to find out if the change is positive or negative TODO not really sure if this works
                    if( goalAngles.get( i ) > currentAngles.get( i ) ){
                        angleDiff = goalAngles.get( i ) - currentAngles.get( i );
                        plusMinus = 1;
                    } else {
                        angleDiff = currentAngles.get( i ) - goalAngles.get( i );
                        plusMinus = -1;
                    }
                    System.out.println("Angle Dif   :" + angleDiff);

                    //Checks if we can move the desirec amount ( angle diff) or if that is too far to move in one step
                    if( angleDiff > maxAngleChange ) {
                        angleChange = maxAngleChange;
                    } else {
                        angleChange = angleDiff;
                    }
                    
                    newAngle = currentAngles.get( i ) + (plusMinus * angleChange);
                    
                    

                    double originX = currentASV.getPosition( i  ).getX();
                    double originY = currentASV.getPosition( i  ).getY();

                    double newx = (broomLength * Math.cos(newAngle)) + originX;
                    double newy = (broomLength * Math.sin(newAngle)) + originY;

                    changeInX =  newx - currentASV.getPosition(i + 1 ).getX();
                    changeInY =  newy - currentASV.getPosition(i + 1 ).getY();



                    //update the rest of the nodes.
                    for( int j = i; j < start.getASVCount() - 1; j++ ){
                        Point2D origin = currentASV.getPosition( j + 1 );

                        Point2D newPoint = new Point2D.Double(origin.getX() + changeInX ,origin.getY() + changeInY );

                        System.out.println("Change x:" + changeInX);
                        System.out.println("Change y:" + changeInY);


                        //to check max step wasnt exceeded todo maybe check broom too ?
                        if( currentASV.getPosition(j+1).distance(newPoint) > maxStep + 0.0000000001 ) {             //TODO weird rounding issue ????
                            System.out.println("Distance " + currentASV.getPosition(j+1).distance(newPoint));
                            throw new IllegalStateException();
                        }

                        //Set new asv location
                        currentASV.setASVPosition( j + 1, newPoint );

                    }

                    //add step to the list
                    steps.add(new ASVConfig(currentASV));
                }
                break;
            }

            //Time to move forward
            double distanceX = goal.getPosition(1 ).getX() - currentASV.getPosition(1 ).getX();
            double distanceY = goal.getPosition(1 ).getY() - currentASV.getPosition(1 ).getY();
            double angleToGoal = Math.atan2(distanceY, distanceX);

            changeInX = maxStep * Math.cos(angleToGoal);
            changeInY = maxStep * Math.sin(angleToGoal);
            Point2D newPoint = new Point2D.Double(currentASV.getPosition(0 ).getX() + changeInX,
                    currentASV.getPosition(0 ).getY() + changeInY );

            //Checking to see if we have moved too far by moving a max step size
            if( start.getPosition(0 ).distance(goal.getPosition(0 )) < start.getPosition(0 ).distance( newPoint)){
                // Gone to far
                distanceX = goal.getPosition(0 ).getX() - currentASV.getPosition(0 ).getX();
                distanceY = goal.getPosition(0 ).getY() - currentASV.getPosition(0 ).getY();

                //TODO should check that the step made here is not too big

                newPoint = new Point2D.Double(currentASV.getPosition(0 ).getX() + distanceX,
                        currentASV.getPosition(0 ).getY() + distanceY);

            }

            //Set new asv location
            currentASV.setASVPosition( 0 , newPoint );


            for(int i = 1; i < currentASV.getASVCount(); i++ ){
                Point2D origin = currentASV.getPosition( i );

                newPoint = new Point2D.Double(origin.getX() + changeInX ,origin.getY() + changeInY );

                //to check max step wasnt exceeded
                if( currentASV.getPosition( i ).distance(newPoint) > maxStep + 0.0000000001 ) {                             //TODO weird rounding issue ????
                    System.out.println("Error distance = " + currentASV.getPosition( i ).distance(newPoint) );
                    System.out.println(maxStep);
                    throw new IllegalStateException();
                }

                //Set new asv location
                currentASV.setASVPosition( i , newPoint );
            }

            //add step to the list
            steps.add(new ASVConfig(currentASV));

            if(currentASV.equals(goal)){
                return steps;
            }



        }
  	}
    public boolean validityCheck(ASVConfig c){
        return (tester.hasEnoughArea(c))&& tester.fitsBounds(c)
                && tester.isConvex(c) && tester.hasValidBoomLengths(c);
    }
}
