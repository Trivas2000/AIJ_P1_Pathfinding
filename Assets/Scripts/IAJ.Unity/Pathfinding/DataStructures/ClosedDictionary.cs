﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures
{
    class ClosedDictionary : IClosedSet
    {

        //Tentative dictionary type structure, it is possible that there are better solutions...
        private Dictionary<Vector2, NodeRecord> Closed { get; set; }

        public ClosedDictionary()
        {
            this.Closed = new Dictionary<Vector2, NodeRecord>();
        }

        public void Initialize()
        {
           //TODO implement
            throw new NotImplementedException();
        }


        public void AddToClosed(NodeRecord nodeRecord)
        {
            //TODO implement
            throw new NotImplementedException();
        }

        public void RemoveFromClosed(NodeRecord nodeRecord)
        {
            //TODO implement
            throw new NotImplementedException();
        }

        public NodeRecord SearchInClosed(NodeRecord nodeRecord)
        {
            //TODO implement
            throw new NotImplementedException();
        }   

        public ICollection<NodeRecord> All()
        {
            //TODO implement
            throw new NotImplementedException();
        }

      
    }

}

