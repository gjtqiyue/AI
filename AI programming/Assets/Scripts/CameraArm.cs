using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraArm : MonoBehaviour {

    Vector3 centerPoint = new Vector3(0, 0, 0);
    Vector3 previousMousePos = new Vector3(0, 0, 0);

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            previousMousePos = Input.mousePosition;
        }

        // check if the mouse button is held down
        // we need to do the rotation when the mouse is held down based on the offset of the coordinate
        if (Input.GetMouseButton(0))
        {
            // get the offset
            Vector3 offset = Input.mousePosition - previousMousePos;
            Debug.Log(offset);

            transform.RotateAround(centerPoint, Vector3.up, offset.x);

            previousMousePos = Input.mousePosition;
        }


    }
}
