﻿using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using System.Collections.Generic;
using UnityEngine;
using Assets.Scripts.Grid;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using System;

namespace Assets.Scripts.IAJ.Unity.Pathfinding
{
    [Serializable]
    public class AStarPathfinding
    {
        // Cost of moving through the grid
        protected const float MOVE_STRAIGHT_COST = 1;
        protected const float MOVE_DIAGONAL_COST = 1.5f;
        public Grid<NodeRecord> grid { get; set; }
        public uint NodesPerSearch { get; set; }
        public uint TotalProcessedNodes { get; protected set; }
        public int MaxOpenNodes { get; protected set; }
        public float TotalProcessingTime { get; set; }
        public int Fill { get; set; }
        public bool InProgress { get; set; }
        public bool TieBreakingOn { get; set; }
        public IOpenSet Open { get; protected set; }
        public IClosedSet Closed { get; protected set; }
        public IHeuristic Heuristic { get; protected set; }

        public NodeRecord GoalNode { get; set; }
        public NodeRecord StartNode { get; set; }
        public int StartPositionX { get; set; }
        public int StartPositionY { get; set; }
        public int GoalPositionX { get; set; }
        public int GoalPositionY { get; set; }

        public AStarPathfinding(IOpenSet open, IClosedSet closed, IHeuristic heuristic, bool tieBreakingOn) : this(open, closed, heuristic)
        {
            this.TieBreakingOn = tieBreakingOn;
        }

        public AStarPathfinding(IOpenSet open, IClosedSet closed, IHeuristic heuristic)
        {
            grid = new Grid<NodeRecord>((Grid<NodeRecord> global, int x, int y) => new NodeRecord(x, y));
            this.Open = open;
            this.Closed = closed;
            this.InProgress = false;
            this.Heuristic = heuristic;
            this.NodesPerSearch = 20; //by default we process all nodes in a single request, but you should change this
            this.TieBreakingOn = false;
        }
        public virtual void InitializePathfindingSearch(int startX, int startY, int goalX, int goalY)
        {
            this.StartPositionX = startX;
            this.StartPositionY = startY;
            this.GoalPositionX = goalX;
            this.GoalPositionY = goalY;
            this.StartNode = grid.GetGridObject(StartPositionX, StartPositionY);
            this.GoalNode = grid.GetGridObject(GoalPositionX, GoalPositionY);

            //if it is not possible to quantize the positions and find the corresponding nodes, then we cannot proceed
            if (this.StartNode == null || this.GoalNode == null) return;

            // Reset debug and relevat variables here
            this.InProgress = true;
            this.TotalProcessedNodes = 0;
            this.TotalProcessingTime = 0.0f;
            this.MaxOpenNodes = 0;
            this.Fill = 0;

            //Starting with the first node
            var initialNode = new NodeRecord(StartNode.x, StartNode.y)
            {
                gCost = 0,
                hCost = this.Heuristic.H(this.StartNode, this.GoalNode),
                index = StartNode.index
            };

            initialNode.CalculateFCost();
            this.Open.Initialize();
            this.Open.AddToOpen(initialNode);
            this.Closed.Initialize();
        }
        public virtual bool Search(out List<NodeRecord> solution, bool returnPartialSolution = false) {

            var ProcessedNodes = 1;
            MaxOpenNodes = 0;
            //TotalProcessingTime = 0.0f;

            NodeRecord currentNode = null;

            while (Open.CountOpen() > 0 && ProcessedNodes <= NodesPerSearch)
            {
                MaxOpenNodes = Math.Max(MaxOpenNodes, Open.CountOpen());

                currentNode = Open.GetBestAndRemove();
                if (currentNode.Equals(GoalNode))
                {
                    solution = CalculatePath(currentNode);
                    this.Fill = Open.CountOpen() + Closed.All().Count;
                    return true;
                }                             

                foreach (var neighbourNode in currentNode.GetNeighbourList(grid))
                {
                    this.ProcessChildNode(currentNode, neighbourNode);
                    ProcessedNodes++;
                    TotalProcessedNodes++;
                }

                Closed.AddToClosed(currentNode);

            }
            solution = null;
            if (returnPartialSolution)
            {
                solution = CalculatePath(currentNode);
            }
            return false;

        }

        protected virtual void ProcessChildNode(NodeRecord parentNode, NodeRecord node)
        {
            float newCost = parentNode.gCost + CalculateDistanceCost(parentNode, node);

            if (Open.SearchInOpen(node) != null) 
            {
                var currentNodeInOpen = Open.SearchInOpen(node);
                if (newCost < currentNodeInOpen.gCost)
                {
                    node.gCost = newCost;
                    node.CalculateFCost(this.TieBreakingOn);
                    node.parent = parentNode;
                    Open.Replace(currentNodeInOpen, node);
                }
            } else if (Closed.SearchInClosed(node) != null) 
            {
                var currentNodeInClosed = Closed.SearchInClosed(node);
                if (newCost < currentNodeInClosed.gCost)
                {
                    Closed.RemoveFromClosed(currentNodeInClosed);
                    node.gCost = newCost;
                    node.CalculateFCost(this.TieBreakingOn);
                    node.parent = parentNode;
                    Open.AddToOpen(node);
                }
            } else
            {
                node.gCost = newCost;
                node.hCost = Heuristic.H(node, this.GoalNode);
                node.parent = parentNode;
                node.CalculateFCost(this.TieBreakingOn);
                Open.AddToOpen(node);
            }
           
            grid.SetGridObject(node.x, node.y, node);
        }


        protected float CalculateDistanceCost(NodeRecord a, NodeRecord b)
        {
            // Math.abs is quite slow, thus we try to avoid it
            int xDistance = 0;
            int yDistance = 0;
            int remaining = 0;

            if (b.x > a.x)
                xDistance = Math.Abs(a.x - b.x);
            else xDistance = a.x - b.x;

            if (b.y > a.y)
                yDistance = Math.Abs(a.y - b.y);
            else yDistance = a.y - b.y;

            if (yDistance > xDistance)
                remaining = Math.Abs(xDistance - yDistance);
            else remaining = xDistance - yDistance;

            // Diagonal Cost * Diagonal Size + Horizontal/Vertical Cost * Distance Left
            return MOVE_DIAGONAL_COST * Mathf.Min(xDistance, yDistance) + MOVE_STRAIGHT_COST * remaining;
        }

        // You'll need to use this method during the Search, to get the neighboors
 
 

        // Method to calculate the Path, starts from the end Node and goes up until the beggining
        public List<NodeRecord> CalculatePath(NodeRecord endNode)
        {
            List<NodeRecord> path = new List<NodeRecord>();
            
            var currentNode = endNode;
            while (!currentNode.Equals(StartNode))
            {
                path.Add(currentNode);
                currentNode = currentNode.parent;
            }
            path.Add(StartNode);
            path.Reverse();
            return path;
        }
    }
}
