using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assets.Scripts.Grid;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding

{
    public enum Direction
    {
        N, NE, E, SE, S, SW, W, NW
    }
    public class GoalBoundAStarPathfinding : NodeArrayAStarPathfinding
    {
        // You can create a bounding box in several differente ways, this is simply suggestion
        // Goal Bounding Box for each Node  direction - Bounding limits: minX, maxX, minY, maxY
        public Dictionary<Vector2, Dictionary<Direction, Vector4>> goalBounds;
        public List<NodeRecord> neighbours;
        public int debugClosed;
        public NodeRecord debugNode;

        public GoalBoundAStarPathfinding(IHeuristic heuristic) : base(heuristic)
        {
            goalBounds = new Dictionary<Vector2, Dictionary<Direction, Vector4>>();

        }

        public void MapPreprocess()
        {
            for (int i = 0; i < grid.getHeight(); i++)
            {
                for (int j = 0; j < grid.getWidth(); j++)
                {
                    var currentNode = grid.GetGridObject(j, i);
                    if (currentNode == null || !currentNode.isWalkable) continue;
                    // TODO implement
                    FloodFill(currentNode);
                    // Floodfill the grid for each direction..
                    CalculateBoundingBox(currentNode);
                    // Calculate the bounding box and repeat
                    this.Open.Initialize();
                    this.Closed.Initialize();

                }
            }

        }

        // You can change the arguments of the following method....
        public void FloodFill(NodeRecord original)
        {

            //TODO
            FirstFloodFillIteration(original);
            // Quite similar to the A*Search method except the fact that there is no goal....so where does it stop?
            while (this.Open.CountOpen() > 0)
            {
                var currentNode = this.Open.GetBestAndRemove();
                this.debugNode = currentNode;
                this.neighbours = currentNode.GetNeighbourList(grid);
                foreach (var neighbourNode in currentNode.GetNeighbourList(grid))
                {
                    ProcessNeighbourNode(currentNode, neighbourNode);
                }
              
                this.Closed.AddToClosed(currentNode);
                this.debugClosed = this.Closed.All().Count;
            }
            // Do stuff...


            //At the end it is important to "clean" the Open and Closed Set
           // this.Open.Initialize();
           // this.Closed.Initialize();
        }

        public void FirstFloodFillIteration(NodeRecord original)
        {
            this.neighbours = original.GetNeighbourList(grid);
            foreach (var neighbourNode in original.GetNeighbourList(grid))
            {
                neighbourNode.gCost = CalculateDistanceCost(original, neighbourNode);
                this.Open.AddToOpen(neighbourNode);

                if (neighbourNode.x < original.x)
                {
                    if (neighbourNode.y == original.y) neighbourNode.direction = Direction.W;
                    else if (neighbourNode.y < original.y) neighbourNode.direction = Direction.SW;
                    else neighbourNode.direction = Direction.NW;
                }
                else if (neighbourNode.x > original.x)
                {
                    if (neighbourNode.y == original.y) neighbourNode.direction = Direction.E;
                    else if (neighbourNode.y < original.y) neighbourNode.direction = Direction.SE;
                    else neighbourNode.direction = Direction.NE;
                }
                else
                {
                    if (neighbourNode.y < original.y) neighbourNode.direction = Direction.S;
                    else neighbourNode.direction = Direction.N;
                }
                // CalculateBoundingBox(original);
            }
            this.Closed.AddToClosed(original);
            //var s = this.Closed.All().Count;
        }

        public void ProcessNeighbourNode(NodeRecord parentNode, NodeRecord neighbourNode)
        {
            float newCost = parentNode.gCost + CalculateDistanceCost(parentNode, neighbourNode);

            if (Open.SearchInOpen(neighbourNode) != null)
            {
                var currentNodeInOpen = Open.SearchInOpen(neighbourNode);
                if (newCost < currentNodeInOpen.gCost)
                {
                    neighbourNode.gCost = newCost;
                    neighbourNode.parent = parentNode;
                    neighbourNode.direction = parentNode.direction;
                    Open.Replace(currentNodeInOpen, neighbourNode);
                }
            }
            else if (Closed.SearchInClosed(neighbourNode) != null)
            {
                var currentNodeInClosed = Closed.SearchInClosed(neighbourNode);
                if (newCost < currentNodeInClosed.gCost)
                {
                    Closed.RemoveFromClosed(currentNodeInClosed);
                    neighbourNode.gCost = newCost;
                    neighbourNode.parent = parentNode;
                    neighbourNode.direction = parentNode.direction;
                    Open.AddToOpen(neighbourNode);
                }
            }
            else
            {
                neighbourNode.gCost = newCost;
                neighbourNode.parent = parentNode;
                neighbourNode.direction = parentNode.direction;
                Open.AddToOpen(neighbourNode);
            }
        }

        
        protected override void ProcessChildNode(NodeRecord parentNode, NodeRecord node)
        {
            if (InsindeGoalBoundBox(StartNode.x, StartNode.y, node.x, node.y, parentNode.direction))
            {
                base.ProcessChildNode(parentNode, node);
            }
        }



        // Checks is if node(x,Y) is in the node(startx, starty) bounding box for the direction: direction
        public bool InsindeGoalBoundBox(int startX, int startY, int x, int y, Direction direction)
        {
            if (!this.goalBounds.ContainsKey(new Vector2(startX, startY)))
                return false;

            if (!this.goalBounds[new Vector2(startX, startY)].ContainsKey(direction))
                return false;

            var box = this.goalBounds[new Vector2(startX, startY)][direction];

            //This is very ugly
            if (box.x >= -1 && box.y >= -1 && box.z >= -1 && box.w >= -1)
                if (x >= box.x && x <= box.y && y >= box.z && y <= box.w)
                    return true;

            return false;
        }

        public void CalculateBoundingBox(NodeRecord node)
        {
            var solutionDict = new Dictionary<Direction, Vector4>();
            this.debugClosed = Closed.All().Count;
            foreach (var closedNode in this.Closed.All())
            {
                if (!solutionDict.ContainsKey(closedNode.direction))
                {
                    solutionDict.Add(closedNode.direction, new Vector4(closedNode.x, closedNode.x, closedNode.y, closedNode.y));
                }
                else
                {
                    var boundingBox = solutionDict[closedNode.direction];
                    solutionDict[closedNode.direction] = new Vector4(
                        Mathf.Min(boundingBox[0], closedNode.x),
                        Mathf.Max(boundingBox[1], closedNode.x),
                        Mathf.Min(boundingBox[2], closedNode.y),
                        Mathf.Max(boundingBox[3], closedNode.y));
                }
            }

            goalBounds.Add(new Vector2(node.x, node.y), solutionDict);
        }
    }
}
