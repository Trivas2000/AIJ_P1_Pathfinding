using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using System.Collections.Generic;
using UnityEngine;
using Assets.Scripts.Grid;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;

namespace Assets.Scripts.IAJ.Unity.Pathfinding
{
    public class NodeArrayAStarPathfinding : AStarPathfinding
    {
        private static int index = 0;
        protected NodeRecordArray NodeRecordArray { get; set; }

        public NodeArrayAStarPathfinding(IHeuristic heuristic) : base(null, null, heuristic)
        {
            grid = new Grid<NodeRecord>((Grid<NodeRecord> global, int x, int y) => new NodeRecord(x, y, index++));
            this.InProgress = false;
            this.Heuristic = heuristic;
            this.NodesPerSearch = uint.MaxValue;
            this.NodeRecordArray = new NodeRecordArray(grid.getAll());
            this.Open = this.NodeRecordArray;
            this.Closed = this.NodeRecordArray;

        }

        // In Node Array A* the only thing that changes is how you process the child node, the search occurs the exact same way so you can the parent's method
        protected override void ProcessChildNode(NodeRecord parentNode, NodeRecord neighbourNode)
        {
            var newCost = parentNode.gCost + CalculateDistanceCost(parentNode, neighbourNode);
            if (neighbourNode.status == NodeStatus.Open)
            {                
                if (newCost < neighbourNode.gCost)
                {
                    var nodeToBeReplaced = Open.SearchInOpen(neighbourNode);
                    neighbourNode.gCost = newCost;
                    neighbourNode.CalculateFCost();
                    neighbourNode.parent = parentNode;
                    Open.Replace(nodeToBeReplaced, neighbourNode);
                }
            } else if (neighbourNode.status == NodeStatus.Closed)
            {
                if (newCost < neighbourNode.gCost)
                {
                    neighbourNode.gCost = newCost;
                    neighbourNode.CalculateFCost();
                    neighbourNode.parent = parentNode;
                    Closed.RemoveFromClosed(neighbourNode);
                }
            } else
            {
                neighbourNode.gCost = newCost;
                neighbourNode.hCost = Heuristic.H(neighbourNode, GoalNode);
                neighbourNode.CalculateFCost();
                neighbourNode.parent = parentNode;
                Open.AddToOpen(neighbourNode);
            }
            grid.SetGridObject(neighbourNode.x, neighbourNode.y, neighbourNode);
        }
    }
}
