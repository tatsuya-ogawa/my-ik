using System;
using System.Linq;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEngine.Serialization;

public class MyJacobianSolver : ISolver
{
    public double epsilon = 1E-12;

    private (Vector3[] positions, float[] angles, Vector3[] parentRelativePositions, Quaternion[]
        parentRelativeRotations) GetInitial(IKNode[] nodes)
    {
        var localPositions = new Vector3[nodes.Length];
        var angles = new float[nodes.Length];
        var parentRelativeRotations = new Quaternion[nodes.Length];
        var parentRelativePositions = new Vector3[nodes.Length];

        IKNode before = null;
        for (var index = 0; index < nodes.Length; index++)
        {
            IKNode current = nodes[index];
            localPositions[index] = Vector3.Scale(current.transform.localPosition, current.transform.parent?.lossyScale ?? Vector3.positiveInfinity);
            angles[index] = Vector3.Dot(current.transform.localEulerAngles, current.axis);
            parentRelativeRotations[index] =
                Quaternion.Inverse(before?.transform.rotation ?? Quaternion.identity) *
                current.transform.parent?.rotation ?? Quaternion.identity;
            parentRelativePositions[index] = Quaternion.Inverse(before?.transform.rotation ??
                                                                Quaternion.identity) *
                                             ((current.transform.parent?.position ??
                                               Vector3.zero) - (before?.transform.position ??
                                                                Vector3.zero));
            before = current;
        }

        return (localPositions, angles, parentRelativePositions, parentRelativeRotations);
    }

    private Vector3 rot2Omega(Quaternion rotation)
    {
        var mat = Matrix4x4.Rotate(rotation);
        var el = new Vector3(mat[3, 2] - mat[2, 3], mat[1, 3] - mat[3, 1], mat[2, 1] - mat[1, 2]);
        var normEl = Vector3.Distance(el, Vector3.zero);
        if (normEl > Mathf.Epsilon)
        {
            return Mathf.Atan2(normEl, mat[1, 1] + mat[2, 2] + mat[3, 3] - 1) / normEl * el;
        }
        else if (mat[1, 1] > 0 && mat[2, 2] > 0 && mat[3, 3] > 0)
        {
            return Vector3.zero;
        }
        else
        {
            return Mathf.PI / 2 * (new Vector3(mat[1, 1] + 1, mat[2, 2] + 1, mat[3, 3] + 1));
        }
    }

    class AngleRange
    {
        private float _base;
        private float _range;

        public AngleRange(float min, float max)
        {
            this._range = max - min;
            this._base = Mathf.Repeat(min + 180f, 360f) - 180f;
        }

        public float Check(float degree)
        {
            degree = Mathf.Repeat(degree + 180f, 360f) - 180f;
            var diff = degree - _base;
            // check angle ranges and fit to nearest one.
            if (diff < 0 || diff > _range)
            {
                if (Mathf.Abs(diff) > Mathf.Abs(diff - _range))
                {
                    return _range + _base;
                }
                else
                {
                    return _base;
                }
            }
            else
            {
                return degree;
            }
        }
    }

    public Quaternion[] Solve(IKNode[] nodes, Vector3 targetPosition, Quaternion? targetRotation = null,
        int iterationLimit = 10)
    {
        var angleRanges = nodes.Select(node => new AngleRange(node.useRotationMin ? node.rotationMin : -180f,
            node.useRotationMax ? node.rotationMax : 180f)).ToArray();
        var wn = DenseMatrix.CreateIdentity(nodes.Length);
        var we = DenseMatrix.CreateDiagonal(6, 6, (int ord) => ord < 3 ? 1 / 0.3f : 0.5f / Mathf.PI);
        var length = nodes.Length;
        var (localPositions, angles, parentRelativePositions, parentRelativeRotations) = GetInitial(nodes);
        var worldRotations = new Quaternion[length];
        var currentEk = float.MaxValue;
        for (var iteration = 0; iteration < iterationLimit; iteration++)
        {
            var jacobianRow = new float[6, length];
            IKNode endIKNode = nodes[nodes.Length - 1];
            var worldPositions = new Vector3[length];
            var tmpWorldRotation = parentRelativeRotations[0];
            var tmpWorldPosition = parentRelativePositions[0];
            worldRotations[0] = tmpWorldRotation * Quaternion.AngleAxis(angles[0], nodes[0].axis);
            worldPositions[0] = tmpWorldPosition + tmpWorldRotation * localPositions[0];
            for (var index = 1; index < length; index++)
            {
                tmpWorldRotation = worldRotations[index - 1] *
                                   parentRelativeRotations[index];
                tmpWorldPosition = worldPositions[index - 1] +
                                   worldRotations[index - 1] * parentRelativePositions[index];

                worldRotations[index] = tmpWorldRotation *
                                        Quaternion.AngleAxis(angles[index], nodes[index].axis);
                worldPositions[index] = tmpWorldPosition + tmpWorldRotation * localPositions[index];
            }

            var endWorldPosition = worldPositions[length - 1];
            var endWorldRotation = worldRotations[length - 1];

            var errPosition = targetPosition - endWorldPosition;
            Vector3 errRotation = targetRotation != null
                ? endWorldRotation * rot2Omega(Quaternion.Inverse(endWorldRotation) * targetRotation.Value)
                : Vector3.zero;
            for (var index = 0; index < length; index++)
            {
                var current = nodes[index];
                var a = worldRotations[index] * current.axis;
                var j = Vector3.Cross(a,
                    endWorldPosition - worldPositions[index]);
                jacobianRow[0, index] = j.x;
                jacobianRow[1, index] = j.y;
                jacobianRow[2, index] = j.z;
                jacobianRow[3, index] = a.x;
                jacobianRow[4, index] = a.y;
                jacobianRow[5, index] = a.z;
            }

            var errMatrix = DenseMatrix.OfArray(new float[,]
            {
                {errPosition.x},
                {errPosition.y},
                {errPosition.z},
                {errRotation.x},
                {errRotation.y},
                {errRotation.z},
            });
            var jacobianMatrix = DenseMatrix.OfArray(jacobianRow);
            var ek = errMatrix.Transpose() * we * errMatrix;
            if (ek[0, 0] < epsilon) break;
            // if (ek[0, 0] > currentEk) break;
            currentEk = ek[0, 0];
            var lambda = currentEk + 0.002f;
            var jh = jacobianMatrix.Transpose() * we * jacobianMatrix + wn * lambda;
            var gerr = jacobianMatrix.Transpose() * we * errMatrix;
            var dAngle = jh.PseudoInverse() * gerr;
            for (var index = 0; index < length; index++)
            {
                // check min/max rotation
                angles[index] =
                    angleRanges[index].Check(angles[index] + dAngle[index, 0] * 180.0f / Mathf.PI);
            }
        }

        return angles.Select((angle,i)=>Quaternion.AngleAxis(angles[i], nodes[i].axis) ).ToArray();
    }
}