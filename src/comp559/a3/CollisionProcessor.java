package comp559.a3;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

/**
 * Class for detecting and resolving collisions.  Currently this class uses penalty forces between rigid bodies.
 * @author kry
 */
public class CollisionProcessor {

    private List<RigidBody> bodies;
    
    /**
     * The current contacts that resulted in the last call to process collisions
     */
    public ArrayList<Contact> contacts = new ArrayList<Contact>();
    
    /**
     * Creates this collision processor with the provided set of bodies
     * @param bodies
     */
    public CollisionProcessor( List<RigidBody> bodies ) {
        this.bodies = bodies;
    }
    
    /** keeps track of the time used for collision detection on the last call */
    double collisionDetectTime = 0;
    
    /** keeps track of the time used to solve the LCP based velocity update on the last call */
    double collisionSolveTime = 0;
    
    /**
     * Processes all collisions 
     * @param dt time step
     */
    public void processCollisions( double dt ) {
        contacts.clear();
        Contact.nextContactIndex = 0;
        
        long now = System.nanoTime();
        broadPhase();
        collisionDetectTime = ( System.nanoTime() - now ) * 1e-9;
                
        if ( contacts.size() > 0  && doLCP.getValue() ) {
            now = System.nanoTime();

            //double bounce = restitution.getValue();
            double mu = friction.getValue();
            // TODO: Objective 3: Compute velocity update with iterative solve of contact constraint matrix.
            
            int N = bodies.size();
            int K = contacts.size();

            double[] b = AssembleB(dt); 
            double[] D = AssembleD();
            
            //precompute b_prime
            double[] b_prime = new double[b.length];
            for (int i = 0; i < b.length; i++)
            {
            	b_prime[i] = b[i] / D[i];
            }
    
    
            //Our lambda_i  are initialized to zero
			double[] lambda = new double[2 * K];            
			double[] friction = new double[K];



			//We need to keep deltaV for updating
			double[] deltaV = new double[3 * N];
            

            //Loop for a set number of iterations
            for(int iteration = 0; iteration < iterations.getValue(); iteration ++)
            {
            	/*
            	 * Objective 4 : Optimization
            	 * We will just shuffle the contacts to randomize the order
            	 */

            	Collections.shuffle(contacts);
            	
            	//loop through all contacts
            	
            	for (Contact contact:contacts) 
            	{
					int c_index = contact.index;
					RigidBody b1 = contact.body1;
					RigidBody b2 = contact.body2;
					
					//Normal case:
					double lambda_normal = -b_prime[2 * c_index] + lambda[2 * c_index];
					for (int i = 0; i < 3; i ++) 
					{
						lambda_normal -= contact.Jrow1[i + 0] / D[2* c_index] * deltaV[3* b1.index + i];
						lambda_normal -= contact.Jrow1[i + 3] / D[2* c_index] * deltaV[3* b2.index + i];
					}
					
					//Make sure >= 0
					lambda_normal = Math.max(0, lambda_normal);

					//update friction
					friction[c_index] = mu * lambda_normal;
					
					//delta lambda
					double delta = lambda_normal - lambda[2 * c_index + 0];
					
					//update lambda
					lambda[2 * c_index] = lambda_normal;
					
					
					//Column i of T
					double [] Ti = new double[6]; 
					for (int i = 0; i < Ti.length; i++) 
					{
						Ti[i] = contact.m[i] * contact.Jrow1[i];
					}
					
					//Delta V update
					for (int i = 0; i < 3; i++) 
					{
						deltaV[3 * b1.index + i] += Ti[i + 0] * delta;
						deltaV[3 * b2.index + i] += Ti[i + 3] * delta;
					}
					

					
					double lambda_friction = -b_prime[2 * c_index + 1] + lambda[2 * c_index + 1];
					
					for (int i =0; i < 3; i ++) 
					{
						lambda_friction -= contact.Jrow2[i + 0] / D[2* c_index + 1] * deltaV[3* b1.index +i];
						lambda_friction -= contact.Jrow2[i + 3] / D[2* c_index + 1] * deltaV[3* b2.index +i];
					}
					
					//Constraints
					lambda_friction = Math.max(lambda_friction, -friction[c_index]);
					
					lambda_friction = Math.min(lambda_friction, friction[c_index]);
					

					delta = lambda_friction - lambda[2 * c_index + 1];
					lambda[2 * c_index + 1] = lambda_friction;
					
					for (int i = 0; i < Ti.length; i++) 
					{
						Ti[i] = contact.m[i] * contact.Jrow2[i];
					}
					
					for (int i = 0; i<3; i++) 
					{
						deltaV[3* b1.index + i] += Ti[i + 0] * delta;
						deltaV[3* b2.index + i] += Ti[i + 3] * delta;
					}	
				}	
            }
            
            //We can now use delta V
            for(int i = 0; i < N; i++)
            {
            	RigidBody body = bodies.get(i);
            	int bid = body.index;
            	
            	body.v.x   += deltaV[3 * bid + 0];
            	body.v.y   += deltaV[3 * bid + 1];
            	body.omega += deltaV[3 * bid + 2];
            	
            }
            	

            
            collisionSolveTime = (System.nanoTime() - now) * 1e-9;

        }
    }
    
	private double[] AssembleD() 
	{
		//size of 2*K, this is just the diagonal of the A matrix
		double[] D = new double[2 * contacts.size()];
		
		for (Contact c:contacts) 
		{
			int index = c.index;
			double[] m = c.m;
			double a_11 = 0;
			double a_22 = 0;

			for (int i = 0; i < c.Jrow1.length; i++) 
			{
				a_11 += c.Jrow1[i] * m[i] * c.Jrow1[i];
				a_22 += c.Jrow2[i] * m[i] * c.Jrow2[i];
			}
			
			D[2*index + 0] = a_11;
			D[2*index + 1] = a_22;
		}
		return D;
	}


    
    private double[] AssembleB(double dt)
    {
    	//b is size 2K
    	double[] b = new double[2 *contacts.size()];
    	
    	for (Contact c : contacts)
    	{
    		RigidBody body1 = c.body1;
    		RigidBody body2 = c.body2;
    		int index = c.index;
    		
    		//Normal constraint
    		double b1 = 0;
    		//Friction constraint
    		double b2 = 0;
    		

    		
    		//u_k is u but only for the kth contact
    		double[] u_k = new double[]{body1.v.x, body1.v.y, body1.omega, body2.v.x, body2.v.y, body2.omega};
    		
    		double[] f = new double[] {body1.force.x, body1.force.y, body1.torque, body2.force.x, body2.force.y, body2.torque};
    		
    		double[] m = c.m;
    		
    		double cb = 0;
    		
    		for (int i = 0; i < u_k.length; i++)
    		{
    			cb += c.Jrow1[i] * u_k[i] * restitution.getValue();
    			b1 += c.Jrow1[i] * (u_k[i] + dt * f[i] * m[i]);
    			b2 += c.Jrow2[i] * (u_k[i] + dt * f[i] * m[i]);
    		}
    		
    		b[2 * index + 0] = b1 + cb ;
    		b[2 * index + 1] = b2;
    	}
    	
    	return b;
    }
    

    /**
     * Checks for collisions between bodies.  Note that you can optionaly implement some broad
     * phase test such as spatial hashing to reduce the n squared body-body tests.
     * Currently this does the naive n squared collision check.
     */
    private void broadPhase() {
        // Naive n squared body test.. might not be that bad for small number of bodies 
        visitID++;
        for ( RigidBody b1 : bodies ) {
            for ( RigidBody b2 : bodies ) { // not so inefficient given the continue on the next line
                if ( b1.index >= b2.index ) continue;
                if ( b1.pinned && b2.pinned ) continue;                
                narrowPhase( b1, b2 );                
            }
        }        
    }
    
    /**
     * Checks for collision between boundary blocks on two rigid bodies.
     * TODO: Objective 2: This needs to be improved as the n-squared block test is too slow!
     * @param body1
     * @param body2
     */
    private void narrowPhase( RigidBody body1, RigidBody body2 ) {
        if ( ! useBVTree.getValue() ) {
            for ( Block b1 : body1.blocks ) {
                for ( Block b2 : body2.blocks ) {
                    processCollision( body1, b1, body2, b2 );
                }
            }
        } else {
        	
        	// TODO: Objective 2: implement code to use hierarchical collision detection on body pairs
        	BVNode bv1 = body1.root;
			BVNode bv2 = body2.root;
			bv1.boundingDisc.updatecW();
			bv2.boundingDisc.updatecW();
			
			if (bv1.boundingDisc.intersects(bv2.boundingDisc)) 
				traverseTree(body1, bv1, body2, bv2);
			

        }
    }
    
    /**
     * Go through the bounding volume tree, and solve leaf collisions
     * 
     */
	private void traverseTree( RigidBody b1, BVNode bv1, RigidBody b2, BVNode bv2 ) 
	{
		//If bv1 and bv2 are leaves, we can process the collision
		if (bv1.isLeaf() && bv2.isLeaf()) 
		{
			processCollision( b1, bv1.leafBlock, b2, bv2.leafBlock );
		}
		else if (bv1.isLeaf() && !bv2.isLeaf()) 
		{
			//When 2 is not a leaf, go further

			BVNode c2left = bv2.child1;
			BVNode c2right = bv2.child2;

			if (c2left.visitID != visitID) 
			{
				c2left.boundingDisc.updatecW();
				c2left.visitID = visitID;
			}
			
			if (c2right.visitID != visitID) 
			{
				c2right.boundingDisc.updatecW();
				c2right.visitID = visitID;
			}

			if (bv1.boundingDisc.intersects(c2left.boundingDisc))
				traverseTree(b1, bv1, b2, c2left);
 
			if (bv1.boundingDisc.intersects(c2right.boundingDisc))
				traverseTree(b1, bv1, b2, c2right);

		}
		
		else if (bv2.isLeaf() && !bv1.isLeaf()) 
		{
			BVNode c1left = bv1.child1;
			BVNode c1right = bv1.child2;
			
			if (c1left.visitID != visitID) 
			{
				c1left.boundingDisc.updatecW();
				c1left.visitID = visitID;
			}
			
			if (c1right.visitID != visitID) 
			{
				c1right.boundingDisc.updatecW();
				c1right.visitID = visitID;
			}

			if (c1left.boundingDisc.intersects(bv2.boundingDisc))
				traverseTree(b1, c1left, b2, bv2);

			if (c1right.boundingDisc.intersects(bv2.boundingDisc)) 
				traverseTree(b1, c1right, b2, bv2);

		}
		
		else 
		{
			BVNode c1;
			BVNode c2;

			double radius1 = bv1.boundingDisc.r;
			double radius2 = bv2.boundingDisc.r;

			if (radius1 > radius2) 
			{
				c1 = bv1.child1;
				c2 = bv1.child2;
				if (c1.visitID != visitID) 
				{
					c1.boundingDisc.updatecW();
					c1.visitID = visitID;
				}
				
				if (c2.visitID != visitID) 
				{
					c2.boundingDisc.updatecW();
					c2.visitID = visitID;
				}

				if (c1.boundingDisc.intersects(bv2.boundingDisc)) 
					traverseTree(b1, c1, b2, bv2);

				if (c2.boundingDisc.intersects(bv2.boundingDisc)) 
					traverseTree(b1, c2, b2, bv2);
			}
			else 
			{
				c1 = bv2.child1;
				c2 = bv2.child2;

				if (c1.visitID != visitID) 
				{
					c1.boundingDisc.updatecW();
					c1.visitID = visitID;
				}
				
				if (c2.visitID != visitID) 
				{
					c2.boundingDisc.updatecW();
					c2.visitID = visitID;
				}

				if (bv1.boundingDisc.intersects(c1.boundingDisc)) 
					traverseTree(b1, bv1, b2, c1);

				if (bv1.boundingDisc.intersects(c2.boundingDisc)) 
					traverseTree(b1, bv1, b2, c2);
			}
			
			
		}
	}
    
    
    /** 
     * The visitID is used to tag boundary volumes that are visited in 
     * a given time step.  Marking boundary volume nodes as visited during
     * a time step allows for a visualization of those used, but it can also
     * be used to more efficiently update the centeres of bounding volumes
     * (i.e., call a BVNode's updatecW method at most once on any given timestep)
     */
    int visitID = 0;
    
    /**
     * Resets the state of the collision processor by clearing all
     * currently identified contacts, and reseting the visitID for
     * tracking the bounding volumes used
     */
    public void reset() {
        contacts.clear();
        Contact.nextContactIndex = 0;
        visitID = 0;            
    }
    
    // some working variables for processing collisions
    private Point2d tmp1 = new Point2d();
    private Point2d tmp2 = new Point2d();
    private Point2d contactW = new Point2d();
    private Vector2d force = new Vector2d();
    private Vector2d contactV1 = new Vector2d();
    private Vector2d contactV2 = new Vector2d();
    private Vector2d relativeVelocity = new Vector2d();
    private Vector2d normal = new Vector2d();
        
    /**
     * Processes a collision between two bodies for two given blocks that are colliding.
     * Currently this implements a penalty force
     * @param body1
     * @param b1
     * @param body2
     * @param b2
     */
    private void processCollision( RigidBody body1, Block b1, RigidBody body2, Block b2 ) {        
        double k = contactSpringStiffness.getValue();
        double c1 = contactSpringDamping.getValue();
        double threshold = separationVelocityThreshold.getValue();
        boolean useSpring = enableContactSpring.getValue();
        boolean useDamping = enableContactDamping.getValue();
        
        body1.transformB2W.transform( b1.pB, tmp1 );
        body2.transformB2W.transform( b2.pB, tmp2 );
        double distance = tmp1.distance(tmp2);
        if ( distance < Block.radius * 2 ) {
            // contact point at halfway between points 
            // NOTE: this assumes that the two blocks have the same radius!
            contactW.interpolate( tmp1, tmp2, .5 );
            // contact normal
            normal.sub( tmp2, tmp1 );
            normal.normalize();
            // create the contact
            Contact contact = new Contact( body1, body2, contactW, normal);
            // simple option... add to contact list...
            contacts.add( contact );
            if ( ! doLCP.getValue() ) {
                // compute relative body velocity at contact point
                body1.getSpatialVelocity( contactW, contactV1 );
                body2.getSpatialVelocity( contactW, contactV2 );
                relativeVelocity.sub( contactV1, contactV2 );
                if ( -relativeVelocity.dot( normal ) < threshold ) {
                    if ( useSpring ) {
                        // spring force
                        double interpenetration = distance - Block.radius * 2; // a negative quantity
                        force.scale( -interpenetration * k, normal );
                        body2.applyContactForceW(contactW, force);
                        force.scale(-1);
                        body1.applyContactForceW(contactW, force);
                    }
                    if ( useDamping ) {
                        // spring damping forces!
                        // vertical
                        force.scale( relativeVelocity.dot(normal) * c1, normal );                    
                        body2.applyContactForceW( contactW, force );
                        force.scale(-1);
                        body1.applyContactForceW( contactW, force );
                    }
                }
            }
        }
    }
   
    /** Stiffness of the contact penalty spring */
    private DoubleParameter contactSpringStiffness = new DoubleParameter("penalty contact stiffness", 1e3, 1, 1e5 );
    
    /** Viscous damping coefficient for the contact penalty spring */
    private DoubleParameter contactSpringDamping = new DoubleParameter("penalty contact damping", 10, 1, 1e4 );
    
    /** Threshold for the relative velocity in the normal direction, for determining if spring force will be applied. */
    private DoubleParameter separationVelocityThreshold = new DoubleParameter( "penalty separation velocity threshold (controls bounce)", 1e-9, 1e-9, 1e3 );
    
    /** Enables the contact penalty spring */
    private BooleanParameter enableContactSpring = new BooleanParameter("enable penalty contact spring", true );
    
    /** Enables damping of the contact penalty spring */
    private BooleanParameter enableContactDamping = new BooleanParameter("enable penalty contact damping", true );
    
    /** Restitution parameter for contact constraints */
    public DoubleParameter restitution = new DoubleParameter( "restitution (bounce)", 0, 0, 1 );
    
    /** Coulomb friction coefficient for contact constraint */
    public DoubleParameter friction = new DoubleParameter("Coulomb friction", 0.33, 0, 2 );
    
    /** Number of iterations to use in projected Gauss Seidel solve */
    public IntParameter iterations = new IntParameter("iterations for GS solve", 10, 1, 500);
    
    /** Flag for switching between penalty based contact and contact constraints */
    private BooleanParameter doLCP = new BooleanParameter( "do LCP solve", true );
    
    /** Flag for enabling the use of hierarchical collision detection for body pairs */
    private BooleanParameter useBVTree = new BooleanParameter( "use BVTree", true );
    
    /**
     * @return controls for the collision processor
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Collision Processing Controls") );
        vfp.add( useBVTree.getControls() );
        vfp.add( doLCP.getControls() );
        vfp.add( iterations.getSliderControls() );
        vfp.add( restitution.getSliderControls(false) );
        vfp.add( friction.getSliderControls(false) );
        
        VerticalFlowPanel vfp2 = new VerticalFlowPanel();
        vfp2.setBorder( new TitledBorder("penalty method controls") );
        vfp2.add( contactSpringStiffness.getSliderControls(true) );
        vfp2.add( contactSpringDamping.getSliderControls(true) );
        vfp2.add( separationVelocityThreshold.getSliderControls( true ) );
        vfp2.add( enableContactDamping.getControls() );
        vfp2.add( enableContactSpring.getControls() );
        
        CollapsiblePanel cp = new CollapsiblePanel(vfp2.getPanel());
        cp.collapse();
        vfp.add( cp );        
        return vfp.getPanel();
    }
    
}
