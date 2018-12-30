using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// for orthogonal effect, emit the z value and keep the rest as identity matrix
// for perspective effect, change the last row to (0, 0, 1, 0)
public class CameraTransformation : Transformation {

    public float focalLength = 1f;

    public override Matrix4x4 Matrix
    {
        get
        {
            Matrix4x4 matrix = new Matrix4x4();
            matrix.SetRow(0, new Vector4(focalLength, 0f, 0f, 0f));
            matrix.SetRow(1, new Vector4(0f, focalLength, 0f, 0f));
            matrix.SetRow(2, new Vector4(0f, 0f, 0f, 0f));
            matrix.SetRow(3, new Vector4(0f, 0f, 1f, 0f));
            return matrix;
        }
    }
}
