package agent;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.awt.Shape;
import java.util.ArrayList;
import java.util.List;

public class HBVNode {
    private List<HBVNode> children = new ArrayList<HBVNode>();
    private HBVNode parent = null;
    private Rectangle2D volume = null;
    private Object primitive= null;

    public HBVNode(){
        //Empty tree in the case of no Obstacles
    }

    public HBVNode(Object primitive) {
	        /*
	         * Two Cases : either the primitive is a Line or it is another volume
	         */
        this.primitive = primitive;
        if(primitive instanceof Line2D){
            Line2D.Double l= (Line2D.Double)primitive;
            this.volume = l.getBounds2D();
        }else{
            if(primitive instanceof Rectangle2D){
                this.volume =(Rectangle2D.Double)primitive;
            }
        }

    }

    public HBVNode(Object primitive, HBVNode parent) {
        this.primitive = primitive;
        if(primitive instanceof Line2D){
            Line2D.Double l= (Line2D.Double)primitive;
            this.volume = l.getBounds2D();
        }else{
            if(primitive instanceof Ellipse2D){
                this.volume =(Rectangle2D.Double)primitive;
            }
        }
        this.parent = parent;
    }

    public List<HBVNode> getChildren() {
        return children;
    }

    public void setParent(HBVNode parent) {
        // parent.addChild(this);
        this.parent = parent;
    }

    public void addChild(Object primitive) {
        HBVNode child = new HBVNode(primitive);
        child.setParent(this);
        this.children.add(child);
    }

    public void addChild(HBVNode child) {
        child.setParent(this);
        this.children.add(child);
    }

    public Rectangle2D getVolume() {
        return this.volume;
    }
	    
	    
	    /*public void setVolume(T data) {
	        this.data = data;
	    }*/

    public Object getPrimitive() {
        return primitive;
    }

    public void setPrimitive(Object primitive) {
        this.primitive = primitive;
    }

    public boolean isRoot() {
        return (this.parent == null);
    }

    public boolean isLeaf() {
        if(this.children.size() == 0)
            return true;
        else
            return false;
    }

    public void removeParent() {
        this.parent = null;
    }

    public boolean isEmpty(){
        if((children.size() ==0)&& parent == null && volume == null && primitive == null)
            return true;
        return false;
    }

    @Override
    public String toString(){
        return volume+" primitives: "+children;
    }
}