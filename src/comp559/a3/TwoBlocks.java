package comp559.a3;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Color3f;
import javax.vecmath.Vector2d;

import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

public class TwoBlocks {
	
	boolean use = false;
	boolean createBodyRequest = true;
	
	private ArrayList<RigidBody> unpinnedBodies = new ArrayList<RigidBody>();
	
    
    RigidBodySystem system;
    
    public TwoBlocks( RigidBodySystem system ) {
        this.system = system;    
        //this.system.bodies.add(Rectangle(10,5,new Color3f(1.000f, 0.412f, 0.706f),20,50));
        //this.unpinnedBodies.add(Rectangle(150,30,new Color3f(1.000f, 0.412f, 0.706f),40,225));
    }
    

	private RigidBody Rectangle(int length, int width, Color3f color, int location_x, int location_y){
    	
    	ArrayList<Block> inside = new ArrayList<Block>();
    	ArrayList<Block> boundary = new ArrayList<Block>();
    	
    	for (int i = location_x; i < location_x + length; i++)
    	{
    		for(int j = location_y; j < location_y + width; j++)
    		{
    			Color3f pink = new Color3f(1.000f, 0.412f, 0.706f);
    			Block b = new Block(j, j, color);
    			//Boundaries
    			if(i == location_x || i == location_x + length -1 || j == location_y || j == location_y + width-1)
    			{
    				boundary.add(b);
    			}
    			else
    			{
    				boundary.add(b);
    				inside.add(b);
    			}
    			
    		}
    	}
    	
    	System.out.println("hello" + boundary.size() + ", " +  inside.size());
    	RigidBody body = new RigidBody(inside, boundary);
    	//body.x0.set(location_x + length / 2.0 , location_y + width / 2.0);
    	return body;
    }
	
    public void reset() {

        system.clear();
        for ( RigidBody b : unpinnedBodies ) {
            system.bodies.add( new RigidBody(b) );
        }
    }
	
	

}
