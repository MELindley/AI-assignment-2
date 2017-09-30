package agent;

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
					    //System.out.println("Edge: "+toTest+ " is Valid !");
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
    public ArrayList<ASVConfig> generatePrimitiveSteps(ASVConfig start, ASVConfig goal) {
        if(!validityCheck(start) || !validityCheck(goal)|| rotatesLeft(start,0) != rotatesLeft(goal,0)){
            throw new IllegalStateException();
        }

        Double maxStep = 0.001;
        Double step = 0.00099;
        Double broomLength = 0.05;

        ArrayList<ASVConfig> steps = new ArrayList<>();
        ArrayList<Double> goalAngles = new ArrayList<>();
        ASVConfig currentASV = new ASVConfig(start);

        //Underestimate the maxium angle line based on a straight asv config
        double maxAngleChange = 2 *  Math.asin( (step)/2.0 / ( broomLength * (start.getASVCount() - 1) ) );

        double changeInX;
        double changeInY;

        for( int i = 0; i < currentASV.getASVCount() - 1; i++ ){
            goalAngles.add(goal.getAngle(i));
        }

        while( true ){
            for(int i = 0; i < start.getASVCount() -1 ; i++ ){
                double currentAngle = currentASV.getAngle(i);

                //If the angle is not the desired angle
                if( currentAngle != goalAngles.get(i) ){
                    //change angle between i to i+1

                    double angleDiff;
                    double angleChange;
                    double newAngle;
                    int plusMinus;

                    //trying to find out if the change is positive or negative TODO not really sure if this works
                    if( goalAngles.get( i ) > currentAngle ){
                        angleDiff = goalAngles.get( i ) - currentAngle;
                        plusMinus = 1;

                    } else {
                        angleDiff = currentAngle - goalAngles.get( i );
                        plusMinus = -1;
                    }

                    if( angleDiff > Math.PI){
                        plusMinus = -1 * plusMinus;
                    }

                    //Checks if we can move the desirec amount ( angle diff) or if that is too far to move in one step
                    if( angleDiff > maxAngleChange ) {
                        angleChange = plusMinus*maxAngleChange ;
                    } else {
                        angleChange = plusMinus*angleDiff;
                    }

                    newAngle = currentAngle + angleChange;

//                    System.out.println("Angle Change");
//                    System.out.println("Current angle   :" + currentAngle );
//                    System.out.println("Goal angle      :" + goalAngles.get(i) );
//
//                    System.out.println("new Angle       :" + newAngle);
//                    System.out.println("Angle Dif       :" + angleDiff);
//                    System.out.println("Angle Change    :" + angleChange);

                    double originX = currentASV.getPosition( i ).getX();
                    double originY = currentASV.getPosition( i ).getY();

                    double newx = (broomLength * Math.cos(newAngle)) + originX;
                    double newy = (broomLength * Math.sin(newAngle)) + originY;

                    changeInX =  newx - currentASV.getPosition(i + 1 ).getX();
                    changeInY =  newy - currentASV.getPosition(i + 1 ).getY();


                    //update the rest of the nodes.
                    for( int j = i; j < start.getASVCount() - 1; j++ ){
                        Point2D origin = currentASV.getPosition( j + 1 );
                        Point2D newPoint = new Point2D.Double(origin.getX() + changeInX ,origin.getY() + changeInY );

//                        System.out.println("Goal angle      :" + goalAngles.get(j));
//                        System.out.println("New angle       :" + Angle(currentASV.getPosition( j ),newPoint));

                        //to check max step wasnt exceeded
                        if( currentASV.getPosition(j+1).distance(newPoint) > maxStep ){
//                            System.out.println("Distance " + currentASV.getPosition(j+1 ).distance(newPoint));
                            throw new IllegalStateException();
                        }

                        //Set new asv location
                        currentASV.setASVPosition( j + 1, newPoint );

                    }

                    //add step to the list
//                    if(!validityCheck(currentASV)){
//                        System.out.println("Start Config: \n"+ start+ "\n "+ "End Config: \n"+goal);
//                    }
                    steps.add(new ASVConfig(currentASV));
                } else {
                    continue;
                }
            }

            //Time to move forward
//            System.out.println("Distance Change");

            Point2D goalNode = goal.getPosition(0 );
            Point2D rootNode = currentASV.getPosition(0 );

            double distance = goalNode.distance(rootNode);
//            System.out.println("Distance to goal    :" + distance );

            if( distance < maxStep ){
                changeInX = goalNode.getX() - rootNode.getX();
                changeInY = goalNode.getY() - rootNode.getY();

                int count = 0;
                for(int i = 0; i < currentASV.getASVCount() -1 ; i++ ){
                    if( round( currentASV.getAngle(i) - goalAngles.get(i) ) == 0.0 ){
                        count++;
                    } else {
//                        System.out.println("i :" + round( currentASV.getAngle(i) - goalAngles.get(i) ) );
                        break;
                    }
                }
                if( count == currentASV.getASVCount()-1){
                    steps.add(new ASVConfig(goal));
                    return steps;
                }
            } else {
                double angleToGoal = Angle( rootNode , goalNode );

                changeInX = step * Math.cos(angleToGoal);
                changeInY = step * Math.sin(angleToGoal);
            }


            for(int i = 0; i < currentASV.getASVCount(); i++ ){
                Point2D origin = currentASV.getPosition( i );

                Point2D newPoint = new Point2D.Double(origin.getX() + changeInX ,origin.getY() + changeInY );

                //to check max step wasnt exceeded
                if( currentASV.getPosition( i ).distance(newPoint) > maxStep ) {
                    System.out.println("Error distance  :" + currentASV.getPosition( i ).distance(newPoint) );
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

    public double round(double d){
        return Math.round(d * 1000000000d)/1000000000d;
    }



    public double Angle( Point2D p1, Point2D p2 ){
        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();


        int neg = 1;
        double offset=0;

        if(dx > 0){
            //quad 1 or 4
            if(dy > 0) {
                //quad 1
            } else {
                //quad 4
                neg = -1;
                offset = Math.PI * 2;
            }
        } else {
            //quad 2 or 3
            if(dy > 0) {
                //quad 2
                neg = -1;
                offset = Math.PI;
            } else {
                //quad 3
                offset = Math.PI;
            }
        }

        double tan = Math.atan2( Math.abs(dy), Math.abs(dx) );
        return offset + (neg * tan);
    }


    public boolean validityCheck(ASVConfig c){


        return (tester.hasEnoughArea(c))&& tester.fitsBounds(c)
                && tester.isConvex(c) && tester.hasValidBoomLengths(c);
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
}
