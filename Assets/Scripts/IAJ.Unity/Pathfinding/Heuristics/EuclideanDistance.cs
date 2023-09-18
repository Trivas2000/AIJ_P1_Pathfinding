using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using System;
using UnityEngine;


namespace Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics
{
    public class EuclideanDistance : IHeuristic
    {
        public float H(NodeRecord node, NodeRecord goalNode)
        {
            float result = (float)Math.Sqrt(Math.Pow(node.x - goalNode.x, 2)+ Math.Pow(node.y - goalNode.y, 2));
          
            return result;
        }
    }
}
