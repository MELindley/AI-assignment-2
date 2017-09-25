package agent;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.Shape;
import java.util.ArrayList;
import java.util.List;
import problem.ASVConfig;
import problem.Obstacle;

public class HBVNode {
    private List<HBVNode> children = new ArrayList<HBVNode>();
    //private HBVNode parent = null;
    private Rectangle2D volume = null;
    private Line2D.Double primitive= null;

    public HBVNode(){
        //Empty tree in the case of no Obstacles
    }

    public HBVNode(Line2D.Double primitive) {
	        /*
	         * Constructor for a line primitive
	         */
        this.primitive = primitive;
        this.volume = primitive.getBounds2D();
   
    }
    
    public HBVNode(HBVNode node1, HBVNode node2){
    	/*
    	 * Constructor for HBVNode Emcompassing other 2 nodes
    	 */
    	Rectangle2D volume1 = node1.getVolume();
    	Rectangle2D volume2 = node2.getVolume();
    	this.volume = volume1.createUnion(volume2);
    	this.children.add(node1);
    	this.children.add(node2);
    }

//    public HBVNode(Object primitive, HBVNode parent) {
//        this.primitive = primitive;
//        if(primitive instanceof Line2D){
//            Line2D.Double l= (Line2D.Double)primitive;
//            this.volume = l.getBounds2D();
//        }else{
//            if(primitive instanceof Ellipse2D){
//                this.volume =(Rectangle2D.loat)primitive;
//            }
//        }
//        this.parent = parent;
//    }

    public List<HBVNode> getChildren() {
        return children;
    }

//    public void setParent(HBVNode parent) {
//        // parent.addChild(this);
//        this.parent = parent;
//    }

    public void addChild(HBVNode child) {
        this.children.add(child);
    }

    public Rectangle2D getVolume() {
        return this.volume;
    }
	    
	    
	    /*public void setVolume(T data) {
	        this.data = data;
	    }*/

    public Line2D.Double getPrimitive() {
        return primitive;
    }

    public void setPrimitive(Line2D.Double primitive) {
        this.primitive = primitive;
    }

//    public boolean isRoot() {
//        return (this.parent == null);
//    }

    public boolean isLeaf() {
        if(this.children.size() == 0)
            return true;
        else
            return false;
    }

//    public void removeParent() {
//        this.parent = null;
//    }

//    public boolean isEmpty(){
//        if((children.size() ==0)&& parent == null && volume == null && primitive == null)
//            return true;
//        return false;
//    }

    public boolean hasCollision(ASVConfig c){
        //Retrieve asv position
        List<Point2D> points = c.getASVPositions();
        //initialize return value to false
        //for each point in the asv
        for (int i = 1; i < points.size(); i++) {
            //create a line between this point and the previous
            Line2D.Double cur_line = new Line2D.Double(points.get(i - 1), points.get(i));
            //if the volume intersects
            if(cur_line.intersects(this.volume)){
                //if the node has no children
                if(this.children.size() == 0){
                    //we are at a leaf node and need to check for line intersection
                    return cur_line.intersectsLine(this.primitive);
                }else {
                    //otherwise check the children for collision
                    return (this.children.get(0).hasCollision(c) || this.children.get(1).hasCollision(c));
                }
            }
        }
        return false;
    }

    public boolean hasCollision(List<ASVConfig>configs){
            boolean retval = false;
            for (ASVConfig c :  configs){
                retval |= this.hasCollision(c);
            }
            return retval;
    }
    @Override
    public String toString(){
        return volume+" Children: "+ children+" primitive: "+ primitive;
    }
    
}