using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraRotater : MonoBehaviour {

    public GameObject target;
    public float tweaker = 5;

    private Vector3 direction;
    private Vector3 centerPoint = new Vector3(0, 0, 0);
    private Vector3 previousMousePos = new Vector3(0, 0, 0);

    private void Start()
    {
        
        direction = target.transform.position - transform.position;
    }

    // Update is called once per frame
    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            previousMousePos = Input.mousePosition; // let the rotation continue as we repress
        }

        // check if the mouse button is held down
        // we need to do the rotation when the mouse is held down based on the offset of the coordinate
        if (Input.GetMouseButton(0))
        {
            // get the offset
            Vector3 offset = Input.mousePosition - previousMousePos;
            Debug.Log(offset);

            // direction determines the start position every time the mouse is pressed
            // the desired rotation determines the relative angle rotated from the starting position and rotation

            Quaternion desiredRotation = Quaternion.Euler(offset.y * tweaker, offset.x * tweaker, 0); // create an angle as the mouse moves
            
            transform.position = target.transform.position - (desiredRotation * direction); // make the camera always keep some distance with the target as it rotates
            
            transform.LookAt(target.transform); // the camera needs to look at the target in order to have it rotate around it

            //previousMousePos = Input.mousePosition;
        }

        if (Input.GetMouseButtonUp(0))
        {
            direction = target.transform.position - transform.position; // when the mouse button is up, update the direction so it stays on the latest position
        }
    }
}
