package comp559.a3;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Controller Class provides the interface to interact with the environment
 * Each command is applied for a set amount of duration
 * @author XT
 *
 */
public class Controller {

	/** Body controlled by the controller */
	public RigidBody body;
	
	/** Duration of a command, in number of steps*/
	public int duration = 10;
	
	/** Maximum force allowed to be applied at once */
	public double maximumForce = 100000.0;
	
	protected Vector2d F_UP = new Vector2d(0, -1);
	protected Vector2d F_DOWN = new Vector2d(0, 1);
	protected Vector2d F_LEFT = new Vector2d(-1, 0);
	protected Vector2d F_RIGHT = new Vector2d(1, 0);
	
	/** Constructor */
	public Controller(RigidBody body){
		this.body = body;
	}
	
	/**
	 * Applies an upwards force of given magnitude
	 * @param magnitude
	 */
	public void applyForceUp(double magnitude){
		
		if(magnitude > maximumForce)
			magnitude = maximumForce;
		
		Vector2d tmp = new Vector2d(F_UP);
		tmp.scale(magnitude);

		this.body.force.add(tmp);
	}
	
	/**
	 * Applies an downwards force of given magnitude
	 * @param magnitude
	 */
	public void applyForceDown(double magnitude){
		
		if(magnitude > maximumForce)
			magnitude = maximumForce;
		
		Vector2d tmp = new Vector2d(F_DOWN);
		tmp.scale(magnitude);

		this.body.force.add(tmp);
	}
	
	/**
	 * Applies an Left force of given magnitude
	 * @param magnitude
	 */
	public void applyForceLeft(double magnitude){
		
		if(magnitude > maximumForce)
			magnitude = maximumForce;
		
		Vector2d tmp = new Vector2d(F_LEFT);
		tmp.scale(magnitude);

		this.body.force.add(tmp);
	}
	
	/**
	 * Applies an Right force of given magnitude
	 * @param magnitude
	 */
	public void applyForceRight(double magnitude){
		
		if(magnitude > maximumForce)
			magnitude = maximumForce;
		
		Vector2d tmp = new Vector2d(F_RIGHT);
		tmp.scale(magnitude);

		this.body.force.add(tmp);
	}
	
	/**
	 * Returns a reward the more Right the snake goes
	 * @return reward
	 */
	public double getHorizontalReward(){
		double x_coord = this.body.x.x;
		double y_coord = this.body.x.y;
		
		double Reward;
		if(x_coord > 400)
			if (y_coord < 200 && x_coord > 1500)
			{
				Reward = 2;
			}
			else
			{
				Reward = x_coord / 1200;
			}
				
		else
		{
			if (y_coord < 80 || x_coord < 130)
				Reward = -10;
			else
				Reward = -0.05;
		}
			

		
		System.out.println("(" + (int)x_coord + "," + (int)y_coord + ") | R=" + (int)Reward);
				
		//Reward exponentially
		return Reward;
	}
	
	/**
	 * Returns current state
	 * @return
	 */
	public Point2d getCurrentState(){
		return this.body.x;
	}
	
	
	public void getInfo(){
		StringBuilder sb = new StringBuilder();
		sb.append("BodyIndex:").append(Integer.toString(this.body.index))
		  .append(" |Mass=").append(Integer.toString((int)this.body.massLinear))
		  .append(" |Innertia=").append(Integer.toString((int)this.body.massAngular))
		  .append(" |(").append((int)this.body.x.x).append(", ").append((int)this.body.x.y)
		  .append(") |Force=").append(this.body.previous_force.toString())
		  .append("]");
		System.out.println(sb.toString());
	}
}
