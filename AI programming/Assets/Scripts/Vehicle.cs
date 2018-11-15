using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Vehicle : MonoBehaviour {

    protected int id;

    public float maxSpeed;
    public float maxForce;
    public float maxTurnRate;
    public float boundingRadius;
    public float minDetectionBoxLength;
    public LayerMask blockingLayer;

    //private SteeringBehaviour steeringBehaviour;
    private Rigidbody rig;
    private BoxCollider boxCollider;
    public Vector3 velocity;
    private Vector3 heading;
    private Vector3 side;

    [SerializeField]
    private List<Path> paths = new List<Path>();

    private List<GameObject> taggedObstacles = null;

    //steeringBehaviour reference
    private SteeringBehaviour my_SteeringBehaviour;

    private Vector3 my_SteeringForce = Vector3.zero;

    // Use this for initialization
    void Awake () {
        my_SteeringBehaviour = GetComponent<SteeringBehaviour>();
        my_SteeringBehaviour.SetVehicle(this);
        rig = GetComponent<Rigidbody>();
        boxCollider = GetComponent<BoxCollider>();
        taggedObstacles = new List<GameObject>();
    }

    private void Start()
    {
        //StartCoroutine(Calculate());
    }

    //IEnumerator Calculate()
    //{
    //    while (true)
    //    {
           
    //        yield return new WaitForFixedUpdate();
    //    }
    //}
	
	// Update the physics and velocity of the vehicle 
	void FixedUpdate () {

        //SteeringBehaviour.instance.SetDestination(target.position);

        // calculate the combine force form each steering behavior in the 
        // vehicle's list
        my_SteeringForce = my_SteeringBehaviour.Calculate();
        Vector3 steeringForce = my_SteeringForce;
        Debug.DrawLine(Position(), Position() + steeringForce, Color.red);

        // calculate the acceleration
        Vector3 acceleration = steeringForce / rig.mass;

        // update the velocity
        velocity += acceleration * Time.deltaTime;

        // make sure the velocity doesn't exceed the max velocity
        Vector3 normalized = velocity.normalized;
        if (velocity.magnitude >= maxSpeed)
            velocity = normalized * maxSpeed;

        // update the position
        Vector3 previousPosition = transform.position;
        transform.position += velocity * Time.deltaTime;
        //Debug.Log("velocity2: " + velocity);
        //Debug.DrawLine(Position(), Position() + velocity, Color.green);
        // replaceable method
        //rig.AddForce(steeringForce);

        // update the rotation
        transform.rotation = Quaternion.LookRotation(transform.position - previousPosition);
        

        //Debug.Break();
        // update the heading and side if the velocity is greater than 0
        if (velocity.sqrMagnitude > 0.00000001)
        {
            heading = normalized;
            side = new Vector3(velocity.x, -velocity.y, velocity.z);
        }
	}

    public void TestIfHitWall(Vector3 end, out RaycastHit hit)
    {
        
        Vector3 start = Position();

        boxCollider.enabled = false;
        Physics.Linecast(start, end, out hit, blockingLayer);
        boxCollider.enabled = true;
    }

    public Vector3 GetVelocity(){return velocity;}

    public Vector3 GetHeading(){return heading;}

    public Vector3 GetSide(){return side;}

    public float Speed() { return velocity.magnitude; }

    public Vector3 Position() { return transform.position; }

    public List<Path> GetPath() { return paths; }
}

[System.Serializable]
public class Path
{
    private bool loop = false;
    [SerializeField]
    private List<Vector3> pathNodes = new List<Vector3>();
    private int currentNode = 0;

    public bool isLooping() { return loop; }
    public int GetCurrent() { return currentNode; }
    public List<Vector3> GetPathNodes() { return pathNodes; }
    public void NextPoint() { currentNode = (currentNode+1)%pathNodes.Count; }
    public void LoopPath() {loop = true; }
   
}
