package comp559.a3;

import java.util.ArrayList;

public class PinkBlocks {
	boolean use = false;
	boolean createBodyRequest = true;
	
	private ArrayList<RigidBody> unpinnedBodies = new ArrayList<RigidBody>();
	
    
    RigidBodySystem system;
    
    public PinkBlocks( RigidBodySystem system ) {
        this.system = system;    
        //this.system.bodies.add(Rectangle(10,5,new Color3f(1.000f, 0.412f, 0.706f),20,50));
        //this.unpinnedBodies.add(Rectangle(150,30,new Color3f(1.000f, 0.412f, 0.706f),40,225));
    }
}
