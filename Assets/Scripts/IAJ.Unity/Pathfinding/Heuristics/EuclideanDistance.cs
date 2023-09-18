using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using UnityEngine;


namespace Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics
{
    public class EuclideanDistance : IHeuristic
    {
        public float H(NodeRecord node, NodeRecord goalNode)
        {           
          
            return Mathf.Sqrt(Mathf.Pow(node.x - goalNode.x, 2) + Mathf.Pow(node.y - goalNode.y, 2));
        }
    }
}
