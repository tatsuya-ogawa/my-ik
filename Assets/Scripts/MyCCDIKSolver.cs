using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using JetBrains.Annotations;

public class MyCCDIKSolver : ISolver
{
    public float tolerance = 0.01f;

    public Quaternion[] Solve(IKNode[] nodes, Vector3 targetPosition, Quaternion? targetRotation, int iterationLimit)
    {
        return SolveIK(nodes.Select(node =>
            {
                return new IKNodeForCalculation()
                {
                    position = node.transform.position,
                    rotation = node.transform.rotation,
                    axis = node.axis,
                    useRotationMin = node.useRotationMin,
                    useRotationMax = node.useRotationMax,
                    rotationMax = node.rotationMax,
                    rotationMin = node.rotationMin
                };
            }).ToArray(), nodes[0].transform.parent?.rotation ?? Quaternion.identity, targetPosition, targetRotation,
            iterationLimit);
    }

    [CanBeNull]
    private Quaternion[] SolveIK(IKNodeForCalculation[] nodes, Quaternion baseRotation, Vector3 targetPosition,
        Quaternion? targetRotation,
        int iterationLimit)
    {
        int iterations = 0;
        float distanceToTarget = (targetPosition - nodes[nodes.Length - 1].position).magnitude;
        Quaternion[] localRotations = new Quaternion[nodes.Length];
        localRotations[0] = Quaternion.Inverse(baseRotation) * nodes[0].rotation;
        var distances = (new int[nodes.Length - 1])
            .Select((node, i) => (nodes[i + 1].position - nodes[i].position).magnitude).ToArray();

        for (int i = 1; i < nodes.Length; i++)
        {
            localRotations[i] = Quaternion.Inverse(nodes[i - 1].rotation) * nodes[i].rotation;
        }

        // CCD IK Algorithm
        while (distanceToTarget > tolerance && iterations < iterationLimit)
        {
            var end = nodes[nodes.Length - 1].position;
            for (int i = nodes.Length - 2; i >= 0; i--)
            {
                Vector3 targetVector = Quaternion.Inverse(nodes[i].rotation) * (targetPosition - nodes[i].position);
                Vector3 endVector = Quaternion.Inverse(nodes[i].rotation) *
                                    (end - nodes[i].position);

                // axisに垂直な平面にtoTargetを射影
                Vector3 projectedToTarget = Vector3.ProjectOnPlane(targetVector, nodes[i].axis);
                Vector3 projectedToEnd = Vector3.ProjectOnPlane(endVector, nodes[i].axis);
                var diff = Quaternion.FromToRotation(projectedToEnd, projectedToTarget);
                localRotations[i] = diff * localRotations[i];
                localRotations[i] = ApplyRotationConstraint(nodes[i], localRotations[i]);

                end = nodes[i].rotation * (diff * endVector) + nodes[i].position;
            }

            nodes[0].rotation = baseRotation * localRotations[0];
            for (int i = 1; i < nodes.Length; i++)
            {
                nodes[i].rotation = (nodes[i - 1].rotation * localRotations[i]);
                nodes[i].position = nodes[i - 1].rotation * (distances[i - 1] * Vector3.up) +
                                    nodes[i - 1].position;
            }

            // Update distance to target
            distanceToTarget = (targetPosition - nodes[nodes.Length - 1].position).magnitude;
            iterations++;
        }

        return localRotations;
    }

    Quaternion ApplyRotationConstraint(IKNodeForCalculation node, Quaternion rotation)
    {
        if (node.useRotationMin || node.useRotationMax)
        {
            rotation.ToAngleAxis(out float angle, out Vector3 axis);
            if (node.useRotationMin)
            {
                angle = Mathf.Max(angle, node.rotationMin);
            }
            if (node.useRotationMax)
            {
                angle = Mathf.Min(angle, node.rotationMax);
            }
            return Quaternion.AngleAxis(angle, axis);
        }
        else
        {
            return rotation;
        }
    }
}