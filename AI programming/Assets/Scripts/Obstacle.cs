using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Obstacle : MonoBehaviour {

    public float boundingRadius;
    public float GroundHeight = 0;

    // Use this for initialization
    void Awake () {
        boundingRadius = GetComponent<CapsuleCollider>().radius * transform.localScale.x;
	}
	


    public Vector3 Position() { return new Vector3(transform.position.x, GroundHeight, transform.position.z); }
}
