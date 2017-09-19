package agent;

import problem.ASVConfig;
import java.awt.geom.Point2D;

public class Edge {
	Vertex v1;
	Vertex v2;
	double weight;

	public Edge() {
		v1 = new Vertex();
		v2 = new Vertex();
		weight = 0;
	}

	public Edge(Vertex v1, Vertex v2) {
		this.v1 = v1;
		this.v2 = v2;
		this.weight = weightFinder();
	}
	
	public boolean contains(Vertex v){
		return (v==v1)||(v==v2);
	}
	
	public Vertex getOther(Vertex v){
		if(v == v1){
			return v2;
		}
		if(v == v2){
			return v1;
		}
		return new Vertex();
	}
	
	public Vertex getV1() {
		return this.v1;
	}

	public Vertex getV2() {
		return this.v2;
	}
	
	/*Calculates number of primitive steps from initial vertex to next vertex as weight
	 
	 Calculations based on the fact that each joint and the chair moves independently.
	 
	 Therefore the number of primitive steps = the highest number of primitive steps taken by a single joint or chair.
	
	*/
	public int weightFinder() {
		
		int totalWeight = 0;
		double tempWeight = 0;
		
		ASVConfig vee1 = this.v1.getC();
		ASVConfig vee2 = this.v2.getC();
		
		
		Point2D tempv1 = vee1.getBaseCenter();
		Point2D tempv2 = vee2.getBaseCenter();
		
		tempWeight = Math.abs(tempv1.getY() - tempv2.getY()) + Math.abs(tempv1.getX() - tempv2.getX());
		
		totalWeight = (int) (tempWeight/0.001);
		
		for (int i=0; i < vee1.getJointCount(); i++) {
			tempWeight = Math.abs(vee2.getJointAngles().get(i) - vee1.getJointAngles().get(i));

			if (totalWeight < (tempWeight/Tester.MAX_JOINT_STEP)) {
				totalWeight = (int) (tempWeight/Tester.MAX_JOINT_STEP);
			}
		}
		
		return totalWeight;
	}
	
	public double getWeight() {
		return weight;
	}

	@Override
	public boolean equals(Object obj){
		boolean result = false;
		if(obj instanceof Edge){
			Edge e = (Edge) obj;
			result = (e.getV1()==this.getV1() && e.getV2() == this.getV2())
					||(e.getV1()==this.getV2() && e.getV2() == this.getV1());
		}
		return result;
	}
	
	@Override
	public int hashCode() {
		final int prime = 7;
		int result = 1;
		result = prime * result + (v1.hashCode()+v2.hashCode());
		return result;
	}
	
	@Override
	public String toString(){
		return v1.getC()+" - "+v2.getC();
	}

	
}
