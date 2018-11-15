using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class World : MonoBehaviour {

    public static World instance = null;

    private GameObject[] obstacles;
    private GameObject[] walls;

    private void Awake()
    {
        if (instance == null)
            instance = this;
        else
            Destroy(this);
    }

    // Use this for initialization
    void Start () {
        obstacles = GameObject.FindGameObjectsWithTag("Obstacle");
        walls = GameObject.FindGameObjectsWithTag("Wall");
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    public List<Obstacle> TagObstableWithinRange(Vehicle myVehicle, double myBoxLength)
    {
        List<Obstacle> TaggedObstable = new List<Obstacle>();

        for (int i=0; i<obstacles.Length; i++)
        {
            Obstacle obstacle = obstacles[i].GetComponent<Obstacle>();
            float distance = (obstacles[i].transform.position - myVehicle.Position()).sqrMagnitude;
            float visibleRange = (float)myBoxLength + obstacle.boundingRadius;

            if (distance < Mathf.Pow(visibleRange, 2))
            {
                TaggedObstable.Add(obstacle);
            }
        }

        return TaggedObstable;
    }

    public GameObject[] GetWalls()
    {
        return walls;
    }
}
