using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using JetBrains.Annotations;
using UnityEditor;
using UnityEngine.SocialPlatforms;

public class MyFABRIKSolver : ISolver
{
    public float tolerance = 0.01f;

    public Quaternion[] Solve(IKNode[] nodes, Vector3 targetPosition, Quaternion? targetRotation = null,
        int iterationLimit = 10)
    {
        return SolveIK(nodes, nodes.Select(node =>
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
            }).ToArray(), nodes[0].transform.parent?.transform?.rotation ?? Quaternion.identity, targetPosition,
            iterationLimit);
    }

    [CanBeNull]
    private Quaternion[] SolveIK(IKNode[] _nodes, IKNodeForCalculation[] nodes, Quaternion baseRotation,
        Vector3 targetPosition, int maxIterations = 10)
    {
        var initialPosition = nodes[0].position;
        Quaternion[] localRotations = new Quaternion[nodes.Length];
        int iterations = 0;
        float distanceToTarget = (targetPosition - nodes[nodes.Length - 1].position).magnitude;
        var distances = (new int[nodes.Length - 1])
            .Select((node, i) => (nodes[i + 1].position - nodes[i].position).magnitude).ToArray();
        // FABRIK Algorithm
        while (distanceToTarget > tolerance && iterations < maxIterations)
        {
            // Forward
            nodes[nodes.Length - 1].position = targetPosition;
            for (int i = nodes.Length - 2; i >= 0; i--)
            {
                nodes[i].position = (nodes[i].position - nodes[i + 1].position).normalized * distances[i] +
                                    nodes[i + 1].position;
            }

            // Backward
            nodes[0].position = initialPosition;
            for (int i = 1; i < nodes.Length; i++)
            {
                nodes[i].position = (nodes[i].position - nodes[i - 1].position).normalized * distances[i - 1] +
                                    nodes[i - 1].position;
            }


            // Apply rotations from root to end
            for (int i = 0; i < nodes.Length - 1; i++)
            {
                ApplyRotation(nodes[i], nodes[i + 1].position);
            }

            localRotations[0] = Quaternion.Inverse(baseRotation) * nodes[0].rotation;
            for (int i = 1; i < nodes.Length; i++)
            {
                localRotations[i] = Quaternion.Inverse(nodes[i - 1].rotation) * nodes[i].rotation;
            }
            
            for (int i = 0; i < nodes.Length; i++)
            {
                // localRotations[i] = AdjustRotation(localRotations[i], nodes[i].axis);
            }

            // Feedback rotation to position
            Quaternion globalRotation = baseRotation;
            for (int i = 1; i < nodes.Length; i++)
            {
                nodes[i].position = (globalRotation * localRotations[i - 1]) * (distances[i - 1] * Vector3.up) +
                                    nodes[i - 1].position;
                globalRotation = globalRotation * localRotations[i - 1];
            }

            distanceToTarget = (targetPosition - nodes[nodes.Length - 1].position).magnitude;
            iterations++;
        }

        for (int i = 0; i < nodes.Length; i++)
        {
            // _nodes[i].transform.position = nodes[i].position;
            _nodes[i].transform.localRotation = localRotations[i];
        }

        return localRotations;
    }

    public Quaternion AdjustRotation(Quaternion rotation, Vector3 targetAxis)
    {
        // 元のクォータニオンから角度と軸を取得
        rotation.ToAngleAxis(out float angle, out Vector3 axis);
        return Quaternion.AngleAxis(angle, targetAxis.normalized);
    }
    void ApplyRotation(IKNodeForCalculation node, Vector3 nextNodePosition)
    {
        // ここを実装
        Vector3 targetDirection = nextNodePosition - node.position;
        node.rotation = Quaternion.FromToRotation(Vector3.up, targetDirection);
    }
}