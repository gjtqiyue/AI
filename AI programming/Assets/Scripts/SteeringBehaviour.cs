using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SteeringBehaviour : MonoBehaviour {
    
    [SerializeField]
    private Vector3 destination;

    [SerializeField]
    private Vehicle toPursuit;

    [SerializeField]
    private Vehicle toEvade;

    private Vehicle myVehicle = null;

    [SerializeField, Range(0f,10f)]
    float seekMultAmount = 1;
    [SerializeField, Range(0f, 10f)]
    float arriveMultAmount = 1;
    [SerializeField, Range(0f, 10f)]
    float fleeMultAmount = 1;
    [SerializeField, Range(0f, 10f)]
    float pursuitMultAmount = 1f;
    [SerializeField, Range(0f, 10f)]
    float evadeMultAmount = 2;
    [SerializeField, Range(0f, 10f)]
    float wanderMultAmount = 0.5f;
    [SerializeField, Range(0f, 10f)]
    float obstacleAvoidanceMultAmount = 3f;
    [SerializeField, Range(0f, 10f)]
    float wallAvoidanceMultAmount = 8f;
    [SerializeField, Range(0f, 10f)]
    float interposeMultAmount = 1;
    [SerializeField, Range(0f, 10f)]
    float hideMultAmount = 1;
    [SerializeField, Range(0f, 10f)]
    float pathFollowMultAmount = 1;
    [SerializeField, Range(0f, 10f)]
    float offsetPursuitMultAmount = 1;
    [SerializeField, Range(0f, 10f)]
    float seperationMultAmount = 1.5f;
    [SerializeField, Range(0f, 10f)]
    float cohesionMultAmount = 1.5f;
    [SerializeField, Range(0f, 10f)]
    float alignmentMultAmount = 1.5f;
    

    [SerializeField]
    private bool seekOn = false;
    [SerializeField]
    private bool arriveOn = false;
    [SerializeField]
    private bool fleeOn = false;
    [SerializeField]
    private bool pursuitOn = false;
    [SerializeField]
    private bool evadeOn = false;
    [SerializeField]
    private bool wanderOn = false;
    [SerializeField]
    private bool obstacleAvoidanceOn = false;
    [SerializeField]
    private bool wallAvoidanceOn = false;
    [SerializeField]
    private bool interposeOn = false;
    [SerializeField]
    private bool hideOn = false;
    [SerializeField]
    private bool pathFollowOn = false;
    [SerializeField]
    private bool offsetPursuitOn = false;
    [SerializeField]
    private bool flockingOn = false;
    [SerializeField]
    private bool separationOn = false;
    [SerializeField]
    private bool cohesionOn = false;
    [SerializeField]
    private bool alignmentOn = false;



    /*                        *
     * wander method variable *
     *                        */
    private Vector3 wanderTarget;
    public double wanderRadius;
    public  double wanderDistance;
    public  double wanderJitter;

    /*                         *
    * obstacle method variable *
    *                          */
    private GameObject[] taggedObstacles;
    private double boxLength;

    /*                         *
     * arrive method variable  *
     *                         */
    private enum Deceleration {slow = 3, normal = 2, fast = 1};

    /*                                 *
     * wall avoidance method variable  *
     *                                 */
     public float wallDetectionFeelerLength;
     private Vector3[] myFeelers = new Vector3[3];

    /*                                 *
    * path follow method variable  *
    *                                 */
    [SerializeField]
    private float myWaypointSeekDistance = 0.5f;

    // Use this for initialization
    void Awake () {

        float theta = Random.value * Mathf.PI * 2;
        // only on 2D
        wanderTarget = new Vector3((float)wanderRadius * Mathf.Cos(theta),
                                   0,
                                   (float)wanderRadius * Mathf.Sin(theta));
    }

    public void SetVehicle(Vehicle entity)
    {
        myVehicle = entity;
    }

    /* ************************************************* *
     *                                                   *
     *            Calculate Steering Force               *
     *                                                   *
     * ************************************************* */
    public Vector3 Calculate  ()
    {
        if (myVehicle != null)
        {
            Vector3 steeringForce = Vector3.zero;

            if (obstacleAvoidanceOn)
            {
                Vector3 force = ObstacleAvoidance() * obstacleAvoidanceMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (wallAvoidanceOn)
            {
                Vector3 force = WallAvoidance() * wallAvoidanceMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (evadeOn)
            {
                Vector3 force = Evade(toEvade) * evadeMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (fleeOn)
            {
                Vector3 force = Flee(destination) * fleeMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }


            if (separationOn)
            {
                Vector3 force = Seperation(World.instance.GetAgents()) * seperationMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (cohesionOn)
            {
                Vector3 force = Cohesion(World.instance.GetAgents()) * cohesionMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (alignmentOn)
            {
                Vector3 force = Alignment(World.instance.GetAgents()) * alignmentMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (seekOn)
            {
                Vector3 force = Seek(destination) * seekMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (arriveOn)
            {
                Vector3 force = Arrive(destination, Deceleration.normal) * arriveMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (pursuitOn)
            {
                Vector3 force = Pursuit(toPursuit) * pursuitMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            if (wanderOn)
            {
                Vector3 force = Wander() * wanderMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            //interpose
            if (interposeOn)
            {
                Vector3 force = Interpose(toPursuit, toEvade) * pursuitMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            //offset pursuit

            //hide

            if (pathFollowOn)
            {
                Vector3 force = PathFollow(0) * pathFollowMultAmount;

                if (!(AccumulateForce(ref steeringForce, force))) return steeringForce;
            }

            //Debug.Log(steeringForce);
            return steeringForce;
        }

        return Vector3.zero;
    }

    private bool AccumulateForce(ref Vector3 steeringForce, Vector3 force)
    {
        
        float steeringForceMag = Vector3.Magnitude(steeringForce);
        float forceMag = Vector3.Magnitude(force);

        //calculate the remaining force
        float steeringForceRemain = myVehicle.maxForce - steeringForceMag;

        //if there is nothing left, return false
        if (steeringForceRemain <= 0)
        {
            return false;
        }
        else if (steeringForceRemain > forceMag)
        {
            steeringForce = steeringForce + force;
            return true;
        }
        else
        {
            steeringForce += Vector3.Normalize(force) * steeringForceRemain;
            return true;
        }
    }



    /*  **************************  *
     *                              *
     *  Steering behaviour methods  *
     *                              *
     *  **************************  */

    /**********************  Seek  *************************/

    private Vector3 Seek (Vector3 targetPos) 
    {
        // get to max speed to reach the target
        Vector3 desiredVelocity = Vector3.Normalize(targetPos - myVehicle.Position()) * myVehicle.maxSpeed;

        return (desiredVelocity - myVehicle.GetVelocity());
    }

    /**********************  Flee  *************************/

    private Vector3 Flee (Vector3 targetPos)
    {
        // check if within the panic range
        Vector3 toTarget = targetPos - myVehicle.Position();
        float panicSqr = 10 * 10;

        if (toTarget.sqrMagnitude > panicSqr)
        {
            return new Vector3(0f, 0f, 0f);
        }
        // get the max speed to reach the target in opposite direction
        Vector3 desiredVelocity = Vector3.Normalize(myVehicle.Position() - targetPos) * myVehicle.maxSpeed;
        
        return (desiredVelocity - myVehicle.GetVelocity());
    }

    /**********************  Arrive  *************************/

    private Vector3 Arrive (Vector3 targetPos, Deceleration deceleration)
    {
        Vector3 toTarget = targetPos - myVehicle.Position();

        double distance = toTarget.magnitude;

        // if hasn't arrived at the destination
        if (distance > 0)
        {
            const float tweaker = 0.3f;

            double speed = distance / ((double)deceleration * tweaker);

            // limit the speed with the max speed
            speed = Mathf.Min((float)speed, myVehicle.maxSpeed);

            // get the desired velocity
            Vector3 desiredVelocity = toTarget * (float)speed / (float)distance;

            Debug.Log("arrive force " + (desiredVelocity - myVehicle.GetVelocity()));
            return (desiredVelocity - myVehicle.GetVelocity());
            
        }

        return new Vector3(0f, 0f, 0f);
    }

    /**********************  Pursuit  *************************/

    private Vector3 Pursuit(Vehicle evader)
    {
        // if the evader is just ahead and facing the agent then we just seek
        Vector3 toTarget = evader.Position() - myVehicle.Position();

        double relativeHeading = Vector3.Dot(myVehicle.GetHeading(), evader.GetHeading());

        if (Vector3.Dot(toTarget, myVehicle.GetHeading()) > 0
            && relativeHeading < -0.95) // acos(0.95)=18 degs
        {
            return Seek(evader.Position());
        }

        // otherwise we predict the evader's movement

        // the look ahead time is proportional to the distance
        // and inversely proportional to the su of the agent's velocity
        double lookAheadTime = toTarget.magnitude / (myVehicle.maxSpeed + evader.Speed());

        lookAheadTime += TurnAroundTime(myVehicle, evader.Position());

        // now seek to the predicted future position of the evader
        return Seek(evader.Position() + evader.GetVelocity() * (float)lookAheadTime);
    }

    private double TurnAroundTime(Vehicle myVehicle, Vector3 position)
    {
        Vector3 toTarget = Vector3.Normalize(position - myVehicle.Position());

        // determine the angle between heading and target
        float dot = Vector3.Dot(myVehicle.GetHeading(), toTarget);

        // const coefficient that will adjust the time of turn speed
        // adjust this to get desired behaviour
        const double coefficient = 0.5;

        // the dot product will give 1 if the target is ahead and -1 if it's behind
        // if we get 1 then we don't need any turn aroud time so it's 0
        return (dot - 1) * -coefficient;
    }

    /**********************  Evade  *************************/

    private Vector3 Evade (Vehicle pursuer)
    {
        /* No need to check facing direction this time */

        Vector3 toPursuer = pursuer.Position() - myVehicle.Position();

        // look ahead time is proportional to distance and inversely proportional to speed
        double lookAheadTime = toPursuer.magnitude / (myVehicle.maxSpeed + pursuer.maxSpeed);

        // now flee to the opposite direction
        return Flee(pursuer.Position() + pursuer.velocity * (float)lookAheadTime);
    }

    /**********************  Wander  *************************/

    private Vector3 Wander ()
    {
        // calculate the jitter for this update frame
        double jitterThisTime = wanderJitter * Time.deltaTime;
        

        // first, add a small random vector to the target's position (RandomClamped
        // returns a value between -1 and 1
        wanderTarget += new Vector3((float)(Random.Range(-1f, 1f) * jitterThisTime),
                                    0,
                                    (float)(Random.Range(-1f, 1f) * jitterThisTime));

        // reproject this point onto the circle
        wanderTarget = wanderTarget.normalized * (float)wanderRadius;

        // add the wander distance to the point
        Vector3 targetLocal = wanderTarget + new Vector3(0, 0, (float)wanderDistance);
        //print("local: " + targetLocal);

        // the vector right now is in local space, we need to transfer it to world space
        Vector3 targetWorld = myVehicle.transform.TransformPoint(targetLocal);
        //print("world: " + targetWorld);

        // debug 
        Debug.DrawLine(myVehicle.Position(), targetWorld, Color.red);
        //Debug.DrawLine(myVehicle.Position(), myVehicle.Position() + targetLocal, Color.blue);
        
        // now steer towards it
        return targetWorld - myVehicle.Position();
    }

    /**********************  ObstacleAvoidance  *************************/

    private Vector3 ObstacleAvoidance ()
    {
        // the detection box is proportional to the agent's velocity
        boxLength = myVehicle.minDetectionBoxLength + (myVehicle.Speed() / myVehicle.maxSpeed) * myVehicle.minDetectionBoxLength;
        Debug.DrawLine(myVehicle.Position(), myVehicle.Position() + myVehicle.transform.forward * (float)boxLength, Color.black);
        // keep a track of the cloest intersecting obstacle
        Obstacle closestIntersectingObstacle = null;

        // get tagged obstacle
        List<Obstacle> obstacles = World.instance.TagObstableWithinRange(myVehicle, boxLength);

        // keep a track of the distance to it
        double distanceToCIP = double.MaxValue;

        // local coord of the obstacle
        Vector3 localObstaclePos;

        for (int i = 0; i < obstacles.Count; i++)
        {
            Obstacle obstacle = obstacles[i].GetComponent<Obstacle>();

            // calculate this obstacle's position in local space
            Vector3 localPosition = myVehicle.transform.InverseTransformPoint(obstacle.Position());
            //Debug.Log(obstacle.gameObject.name + ": " + localPosition);

            // if the local position has a negative x value then it must lay
            // behind the agent
            if (localPosition.z >= 0)
            {
                // if the distance from the x axis to the object's position is less
                // than its radius + half of the width of the detection box then there
                // is a potential intersection
                float expandedRadius = myVehicle.boundingRadius + obstacle.boundingRadius;

                if (Mathf.Abs(localPosition.x) < expandedRadius)
                {
                    // calculate the intersection point with the circle
                    float localX = localPosition.z;
                    float localY = localPosition.y;

                    // a = sqrt(r^2 - y^2)
                    float sqrtPart = Mathf.Sqrt(expandedRadius * expandedRadius - localY * localY);

                    // IP = localX - a
                    float ip = localX - sqrtPart;

                    //Debug.Log(ip);
                    // if the first point is <= 0 then take the second possible point
                    if (ip <= 0)
                    {
                        ip = localX + sqrtPart;
                    }

                    // test to see if this is the cloest so far. If it is, keep a record of
                    // the obstacle and its location
                    if (ip < distanceToCIP)
                    {
                        distanceToCIP = ip;

                        closestIntersectingObstacle = obstacle;         
                    }
                }
            }
        }
        //Debug.Log(closestIntersectingObstacle.Position());

        // Calculate the steering force to drive away from the obstacle
        // a lateral force and a braking force
        Vector3 SteeringForce = new Vector3(0f, 0f, 0f);

        if (closestIntersectingObstacle)
        {
            closestIntersectingObstacle.gameObject.GetComponent<MeshRenderer>().material.color = Color.red;
            // recalculate the local position of the cloest obstacle
            localObstaclePos = myVehicle.transform.InverseTransformPoint(closestIntersectingObstacle.Position());

            // the closer the agent is to the obstacle, the stronger the steering force should be
            float multiplier = 1f + ((float)boxLength - localObstaclePos.z) / (float)boxLength;
            double brakingWeight = 0.3;

            // calculate the lateral force on z-axis

            SteeringForce.x = (closestIntersectingObstacle.boundingRadius - localObstaclePos.x) * multiplier;

            SteeringForce.z = (closestIntersectingObstacle.boundingRadius - localObstaclePos.z) * (float)brakingWeight;
            Debug.Log(SteeringForce);
            Debug.DrawLine(myVehicle.Position(), myVehicle.Position() + SteeringForce, Color.blue);
        }

        // finally, convert the steering froce into world space
        
        return myVehicle.transform.TransformVector(SteeringForce);
    }

    /**********************  WallAvoidance  *************************/

    private Vector3 WallAvoidance ()
    {
        // get wall references
        GameObject[] walls = World.instance.GetWalls();

        // create feeler for the vehicle
        CreateFeelers();

        float distToThisIP = 0f;
        float distToCloestIP = float.MaxValue;

        // this will hold and index into the vector of walls
        int cloestWall = -1;

        Vector3 cloestPoint = new Vector3 (0f, 0f, 0f);          // hold the closest point
        Vector3 steeringForce = new Vector3 (0f, 0f, 0f);        // the steering force

        // examine each feeler in turn
        for (int i = 0; i < myFeelers.Length; i++)
        {
            RaycastHit hit;
            myVehicle.TestIfHitWall(myFeelers[i], out hit);
            Debug.DrawLine(myVehicle.Position(), myFeelers[i], Color.green);

            if (hit.transform != null)
            {
                // run through each wall to check for any intersection point
                for (int w = 0; w < walls.Length; w++)
                {
                    if (hit.transform == walls[w].transform)
                    {
                        // is this the cloest found so far? keep a record if yes
                        distToCloestIP = hit.distance;

                        cloestWall = w;

                        cloestPoint = hit.point;
                    }
                } // next wall
            }

            // if an intersection point is detected then calculate the force
            if (cloestWall >= 0)
            {
                Debug.Log(walls[cloestWall].name);
                // calcualte by what distance the projected position of the agent will overshoot the wall
                Vector3 overShoot = myFeelers[i] - cloestPoint;

                // create a force in the direction of the wall normal, with the magnitute of overshoot
                steeringForce += hit.normal * overShoot.magnitude;
            }

        } // next feeler


        return steeringForce;
    }

    private void CreateFeelers()
    {
        // create the first feeler
        myFeelers[0] = myVehicle.Position() + wallDetectionFeelerLength * myVehicle.GetHeading();

        // left feeler
        Vector3 temp = myVehicle.GetHeading();
        temp = Quaternion.Euler(0, 45, 0) * temp;
        myFeelers[1] = myVehicle.Position() + wallDetectionFeelerLength/2f * temp;

        // right feeler
        temp = myVehicle.GetHeading();
        temp = Quaternion.Euler(0, -45, 0) * temp;
        myFeelers[2] = myVehicle.Position() + wallDetectionFeelerLength/2f * temp;

    }

    /**********************  Interpose  *************************/
    private Vector3 Interpose(Vehicle v1, Vehicle v2)
    {
        Vector3 steeringForce = Vector3.zero;

        // get the midpoint of two positions
        Vector3 midPoint = (v1.Position() + v2.Position()) / 2;

        // calculate the time estimated to get there
        float t = (midPoint - myVehicle.Position()).magnitude / myVehicle.maxSpeed;

        // predict the future position of the vehicle assuming the same speed and direction
        Vector3 futPos1 = v1.Position() + v1.velocity * t;
        Vector3 futPos2 = v2.Position() + v2.velocity * t;

        // now steer to the midpoint of future position
        Vector3 newPoint = (futPos1 + futPos2) / 2;

        return Arrive(newPoint, Deceleration.fast);
    }

    private Vector3 Interpose(Vehicle v1, Vector3 des)
    {
        Vector3 steeringForce = Vector3.zero;
        // get the midpoint of two positions
        Vector3 midPoint = (v1.Position() + des) / 2;

        // calculate the time estimated to get there
        float t = (midPoint - myVehicle.Position()).magnitude / myVehicle.maxSpeed;

        // predict the future position of the vehicle assuming the same speed and direction
        Vector3 futPos = v1.Position() + v1.velocity * t;

        // now steer to the midpoint of future position
        Vector3 newPoint = (futPos + des) / 2;
        
        return Arrive(newPoint, Deceleration.fast);
    }

    /**********************  Hide  *************************/

    /**********************  PathFollowing  *************************/
    private Vector3 PathFollow(int index)
    {
        List<Path> paths = myVehicle.GetPath();

        if (paths.Count <= 0) { Debug.Log("no avaliable path"); return new Vector3(0, 0, 0); }

        Path path = paths[index];
        List<Vector3> nodes = path.GetPathNodes();
        int current = path.GetCurrent();

        //move to the next target if close enough to current target
        if (Vector3.Distance(nodes[current], myVehicle.Position()) <= myWaypointSeekDistance)
        {
            path.NextPoint();
        } 

        if (!path.isLooping() && current == nodes.Count - 1)
        {
            return Arrive(nodes[current], Deceleration.normal);
        }
        else
        {
            return Seek(nodes[current]);
        }
    }

    /**********************  OffsetPursuit  *************************/
    public Vector3 OffsetPursuit()
    {
        //coming later
        return Vector3.zero;
    }

    /**********************  Cohesion  *************************/
    private Vector3 Cohesion(List<Vehicle> neighbors)
    {
        Vector3 steeringForce = Vector3.zero;
        Vector3 center = Vector3.zero;

        int count = 0;

        for (int i = 0; i < neighbors.Count; i++)
        {
            // skip self and only consider the vehicles in range
            if (neighbors[i] != myVehicle && neighbors[i].isTagged())
            {
                center += neighbors[i].Position();

                count++;
            }
        }

        if (count > 0)
        {
            center /= count;

            steeringForce = Seek(center);
        }

        return steeringForce;

    }

    /**********************  Allignment  *************************/
    private Vector3 Alignment(List<Vehicle> neighbors)
    {
        Vector3 steeringForce = Vector3.zero;

        int count = 0;

        for (int i = 0; i < neighbors.Count; i++)
        {
            // skip self and only consider the vehicles in range
            if (neighbors[i] != myVehicle && neighbors[i].isTagged())
            {
                steeringForce += neighbors[i].GetHeading();

                count++;
            }
        }

        if (count > 0)
        {
            steeringForce /= count;

            // steering towards it
            steeringForce -= myVehicle.GetHeading();
        }

        return steeringForce;


    }

    /**********************  Seperation  *************************/
    private Vector3 Seperation(List<Vehicle> neighbors)
    {
        Vector3 steeringForce = Vector3.zero;

        for (int i=0; i<neighbors.Count; i++)
        {
            // skip self and only consider the vehicles in range
            if (neighbors[i] != myVehicle && neighbors[i].isTagged())
            {
                Vector3 toAgent = myVehicle.Position() - neighbors[i].Position();

                // calculate the force away from the vehicle
                steeringForce += Vector3.Normalize(toAgent) / toAgent.magnitude;
            }
        }

        return steeringForce;
        
    }



    
    internal void SetDestination(Vector3 t) { destination = t; }

    internal void SetPursuer(Vehicle t) { toPursuit = t; }

    internal void SetEnvade(Vehicle t) { toEvade = t; }
}
