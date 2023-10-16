/*
 * PathFinderMapper.cpp
 *
 *  Created on: 31 Mar 2018
 *      Author: manupa
 * 
 *  notes on the mapper (zhaoying):
 *  1) register connection in MRRG: to support store data into reg, it will connect reg.out -> reg.in (next cycle). It will also mark
 * 		this as a reg_conn to set latency as 1.
 * 	2) SortByASAPBAckEdge. It is sorted by path. 
 *  3) Each iteration, it will put the information of congested prt into congestedPorts. The port can only hold one data (even in code), so the
 * 	 later assignment will overwrite the old one. It uses congestedPorts to keep track of port history. It will set congested part a higher cost 
 * 		to avoid congestion in next iteration.
 * 	4) Conflict and congestion are different. Conflict means it cannot connect to multiple ports due to design limitation (usually between REGF 
 * 		and FU in N2N).
 * 	5) Backtrack means, when it cannot place/route for a node, it will undo for last mapped node.
 *  6) It seems mutexpath is useless.
 * 
 */

#include <morpher/mapper/PathFinderMapper.h>

#include <morpher/mapper/HeuristicMapper.h>
#include <queue>
#include <assert.h>
#include <math.h>
#include <algorithm> // std::reverse
#include <morpher/arch/DataPath.h>
#include <morpher/arch/FU.h>

#include <stack>
#include <functional>
#include <set>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <bitset>
//#define FLEX2
namespace CGRAXMLCompile
{

} /* namespace CGRAXMLCompile */

/*struct hash_LatPort {
    size_t operator()(const pair<int, CGRAXMLCompile::Port*>& p) const
    { 
        auto hash1 = hash<int>{}(p.first); 
        auto hash2 = hash<CGRAXMLCompile::Port*>{}(p.second); 
        return hash1 ^ hash2; 
    } 
}; 
*/
bool CGRAXMLCompile::PathFinderMapper::LeastCostPathAstar(LatPort start,
														  LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node,
														  std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode)
{

	//std::cout << "LeastCoastPath started with start=" << start.second->getFullName() << " to end=" << end.second->getFullName() << "\n";

	std::unordered_map<LatPort, int, hash_LatPort> cost_to_port;
	std::unordered_map<LatPort, LatPort, hash_LatPort> cameFrom;
	std::unordered_map<LatPort, int, hash_LatPort> curr_hops_to_port;

	clearReservedPort();
	for (LatPort p : path)
	{
			releaseTempReservedVectorPort(p.second);
	}
	mutexPaths.clear();
	path.clear();


	bool detailedDebug = false;
	// if(currNode->idx==53)detailedDebug=true;

	bool lessthanII = false;
	CGRA *cgra = endDP->getCGRA();
	int II = cgra->get_t_max();
	int latDiff = end.first - start.first;
	if (latDiff < II)
		lessthanII = true;

	struct port_heuristic
	{
		LatPort p;
		int heuristic;
		std::shared_ptr<std::unordered_set<Port *>> path;
		std::shared_ptr<std::vector<LatPort>> pathVec;

		int calc_heuristic(LatPort src, LatPort dest)
		{
			PE *srcPE = src.second->findParentPE();
			assert(srcPE);
			PE *destPE = dest.second->findParentPE();
			assert(destPE);

			CGRA *currCGRA = srcPE->getCGRA();
			assert(currCGRA);

			int dist_dest = std::abs(destPE->Y - srcPE->Y) + std::abs(destPE->X - srcPE->X) + std::abs(dest.first - src.first);
			// int dist_dest = std::abs(dest.first - src.first);
			return dist_dest;
		}

		//					port_heuristic(LatPort p, LatPort dest){
		//						this->p=p;
		//						heuristic=calc_heuristic(p,dest);
		//					}

		port_heuristic(LatPort p, int cost, bool islessThanII = true)
		{
			this->p = p;
			this->heuristic = cost;
			if (!islessThanII)
			{
				this->path = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>());
				this->pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>());
			}
		}

		port_heuristic(LatPort p, LatPort dest, int cost)
		{
			this->p = p;
			this->heuristic = cost * 100 + calc_heuristic(p, dest);
		}

		port_heuristic(LatPort p, LatPort dest, int cost, std::shared_ptr<std::unordered_set<Port *>> &path)
		{
			this->p = p;
			this->heuristic = cost * 100 + calc_heuristic(p, dest);
			this->path = path;
		}

		bool operator<(const port_heuristic &rhs) const
		{
			return this->heuristic > rhs.heuristic;
		}

		//		bool operator>(const port_heuristic& rhs) const{
		//			return this->heuristic > rhs.heuristic;
		//		}
	};

	std::priority_queue<port_heuristic> q;

	q.push(port_heuristic(start, 0, lessthanII));

	//	path.push_back(start);

	cost_to_port[start] = 0;
	curr_hops_to_port[start] = 0;

	LatPort currPort;
	std::vector<LatPort> deadEnds;

	std::map<LatPort, std::shared_ptr<std::unordered_set<Port *>>> paths;

	std::unordered_set<Port *> emptyset;
	//		paths[start] = emptyset;
	//		paths[start].insert(start.second);

	std::vector<LatPort> finalPath;

	Port *newNodeDPOut = endDP->getPotOutputPort(currNode);
	std::set<Port *> newNodeDPOutCP = newNodeDPOut->getMod()->getConflictPorts(newNodeDPOut);
	std::set<Port *> endPortCP = end.second->getMod()->getConflictPorts(end.second);

	int curr_least_cost_to_end = INT32_MAX;
	//printf("Least cost path..\n");
	int count =1000;
	while (!q.empty())
	{
		count --;
		port_heuristic curr = q.top();
		currPort = curr.p;
		//std::cout << "currPort = " << currPort.second->getFullName() << "\n";
		//printf("bef q pop=%d\n",q.size());
		q.pop();
		//printf("aft q pop\n");
		std::unordered_set<Port *> *currPath;
		std::vector<LatPort> *currPathVec;

		//printf("bef less than II\n");
		if (!lessthanII)
		{
			currPath = curr.path.get();
			currPathVec = curr.pathVec.get();
			paths[currPort] = curr.path;
			if (currPort == end)
			{
				finalPath = *curr.pathVec;
			}
		}

		if (detailedDebug){
			std::cout << "currPort=" << currPort.second->getFullName() << ",";
			if(currPort.second->getType() == IN) cout << "type=IN,";
			if(currPort.second->getType() == OUT) cout << "type=OUT,";
			if(currPort.second->getType() == INT) cout << "type=INT,";
		}
		if (detailedDebug)
			std::cout << "latency = " << currPort.first << "\n";
		//printf("bef hops check\n");
		assert(curr_hops_to_port.find(currPort) != curr_hops_to_port.end());
		if(curr_hops_to_port[currPort] > cgra->max_hops){
			continue;
		}
		//printf("end check\n");
		if (currPort == end)
		{
			if(cost_to_port[currPort] < curr_least_cost_to_end){
				curr_least_cost_to_end = cost_to_port[currPort];
			}
			continue;
		}
		//printf("cost check\n");
		if(cost_to_port[currPort] > curr_least_cost_to_end){
			continue;
		}
		//printf("next ports\n");
		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);
		//printf("next ports done\n");

		int q_len = q.size();
		//std::cout << "currPort = " << currPort.second->getFullName() << "\n";
		//printf("end latency = %d\n",end.first);
		for (LatPort nextLatPort : nextPorts)
		{
			//std::cout << "nextPort = " << nextLatPort.second->getFullName() << "\n";
			Port *nextPort = nextLatPort.second;
			//if(currPort.second->getName() == "TREG_RI")
			//	std::cout << "currPort = " << currPort.second->getName() << " nextLatPort" << nextPort->getName() << "\n";
			//printf("FLEX checks\n");
#ifdef FLEX
			if(nextLatPort.second->isReserved)// or nextLatPort.second->isRestricted)
			{
				//std::cout << "Continuing since reserved\n ";
				continue;
			}
			//printf("FLEX checks1\n");
			/*if(nextLatPort.second->isRestricted)
			{
				//std::cout << "Continuing since restricted\n ";
				continue;
			}*/
			if(nextLatPort.second->isTempReserved)
			{
				//std::cout << "Continuing since temp reserved\n ";
				//continue;
			}
			//printf("FLEX checks2\n");
			if((currPort.second->getName().find("_RI") != std::string::npos) && (nextLatPort.second->getName().find("_RO") != std::string::npos) )
			{
				if(((currPort.second->getName().find("TREG") != std::string::npos) && (currPort.second->getName().substr(0, 5) == nextLatPort.second->getName().substr(0, 5))) || ((currPort.second->getName().find("TREG") == std::string::npos) &&(currPort.second->getName().substr(0, 3) == nextLatPort.second->getName().substr(0, 3) )))
					continue;
			}
			//printf("FLEX checks3\n");
			if(node->idx != 0)
				if(!checkVectorPort(nextPort))
				{
					//std::cout << "Continuing since check vector\n ";
					continue;
				}
#endif
			//printf("end FLEX checks\n");
			//std::cout << nextLatPort.first << "\n";
			//printf("first checks\n");
			if (nextLatPort.first > end.first)
				continue; //continue if the next port has higher latency
			//printf("some latency checks\n");
			if((nextLatPort.second->getNode() == node))
			{
				if(nextLatPort.first != nextLatPort.second->getLat())
					continue;

			}
			//assert(nextLatPort.first - currPort.first <= 1);


			//printf("before lessthan II\n");
			if (!lessthanII)
			{
				if (currPath->find(nextPort) != currPath->end())
				{
					continue;
				}
				for (Port *cp : nextPort->getMod()->getConflictPorts(nextPort))
				{
					if (currPath->find(cp) != currPath->end())
					{
						continue;
					}
				}
			}
			//printf("aft lessthan II\n");
			if (newNodeDPOutCP.find(nextPort) != newNodeDPOutCP.end())
			{
				continue;
			}

			if (endPortCP.find(nextPort) != endPortCP.end())
			{
				continue;
			}


			//printf("regs check\n");
			if (currPort.second->getMod()->regCons[std::make_pair(currPort.second, nextLatPort.second)])
			{
				assert(nextLatPort.first != currPort.first);
			}

			bool isRegConType1 = currPort.second->getName().find("REG_O") != std::string::npos &&
								 nextLatPort.second->getName().find("REG_I") != std::string::npos;
			bool isRegConType2 = currPort.second->getName().find("_RO") != std::string::npos &&
								 nextLatPort.second->getName().find("_RI") != std::string::npos;

			if (isRegConType1 || isRegConType2)
			{
				//std::cout << "\tnextPort=" << nextPort->getFullName() << "\n";

				if (nextLatPort.first == currPort.first)
				{
					if(currPort.second->getName().find("ITREG") != std::string::npos)
					{
						std::cout << "\tnextPort=" << nextPort->getFullName() << "\n";
						nextLatPort.first = nextLatPort.first + cgra->get_t_max();
					}
					else
					{
						std::cout << "\tnextPort=" << nextPort->getFullName() << "\n";
						nextLatPort.first = nextLatPort.first + 1;
					}
				}

				if(node->idx != 0)
				{
					checkTempReservedVectorPort(nextLatPort.second);
					checkTempReservedVectorPort(currPort.second);
				}
			}
			//printf("bef true\n");
			if (true)
			{ // unmapped port
				if (detailedDebug)
					std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
				if (detailedDebug)
					std::cout << "latency = " << nextLatPort.first << ",";

				int nextPortCost = cost_to_port[currPort] + calculateCost(currPort, nextLatPort, end);
				//if(node->idx == 15)
				//{
				// std::cout << "currPort = " << currPort.second->getFullName() << " nextLatPort" << nextLatPort.second->getFullName() << "\n";

				//std::cout << "nextPortCost = " << nextPortCost << "cost_to_port[currPort]= " << cost_to_port[currPort] << "calculate cost=" << calculateCost(currPort, nextLatPort, end) << "\n";
				//}
				bool is_true=checkInCameFrom(cameFrom,currPort.second);
				if(is_true)
				{
					nextPortCost += 10000;
					//continue;
				}
				if ((nextPort->getNode() == node) && (node->idx != 0))
				//if (nextPort->getNode() == node)
				{
					nextPortCost = cost_to_port[currPort];
				}

				if((node->idx == 0) && (nextPort->findParentPE() != end.second->findParentPE()))
				{
					//std::cout << nextLatPort.second->getFullName() << "\n";
					nextPortCost += 10000;
				}


				if (checkRecParentViolation(currNode, nextLatPort))
				{
					std::cout << "Port is not inserted, since it violated recurrence parent..\n";
					continue;
				}
				if (detailedDebug)


				if (nextPortCost < cost_to_port[currPort])
				{
					std::cout << "nextPortCost = " << nextPortCost << "\n";
					std::cout << "cost_to_port[currPort] = " << cost_to_port[currPort] << "\n";
				}
				assert(nextPortCost >= cost_to_port[currPort]);
				//printf("bef checking end\n");
				if (cost_to_port.find(nextLatPort) != cost_to_port.end())
				{
					if (cost_to_port[nextLatPort] > nextPortCost)
					{
						cost_to_port[nextLatPort] = nextPortCost;
						cameFrom[nextLatPort] = currPort;

						if(nextLatPort.first == currPort.first && nextLatPort.second->getPE() != currPort.second->getPE()){
							//next latport is inter-PE connection and it is not increasing latency
							//therefore it should be a hop
							curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort] + 1;
						}
						else if(nextLatPort.first != currPort.first){
							curr_hops_to_port[nextLatPort] = 0;
						}
						else{
							curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort];
						}	

						if (!lessthanII)
						{
							std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
							newPath->insert(currPort.second);
							port_heuristic ph(nextLatPort, end, nextPortCost, newPath);
							ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
							ph.pathVec->push_back(currPort);
							q.push(ph);
							//std::cout << "currPort = " << currPort.second->getFullName() << " nextLatPort" << nextLatPort.second->getFullName() << " cost=" << nextPortCost << "\n";
											//std::cout << "nextPortCost = " << nextPortCost << "cost_to_port[currPort]= " << cost_to_port[currPort] << "calculate cost=" << calculateCost(currPort, nextLatPort, end) << "\n";


						}
					}
					else
					{
						if (detailedDebug)
							std::cout << "Port is not inserted..\n";
					}
				}
				else
				{

					cost_to_port[nextLatPort] = nextPortCost;
					cameFrom[nextLatPort] = currPort;

					if(nextLatPort.first == currPort.first && nextLatPort.second->getPE() != currPort.second->getPE()){
						//next latport is inter-PE connection and it is not increasing latency
						//therefore it should be a hop
						curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort] + 1;
					}
					else if(nextLatPort.first != currPort.first){
						curr_hops_to_port[nextLatPort] = 0;
					}	
					else{
						curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort];
					}	

					//						assert(paths.find(nextLatPort)==paths.end());
					//						paths[nextLatPort]=paths[currPort];
					//						paths[nextLatPort].insert(nextLatPort.second);
					//						paths[nextLatPort].insert(currPort.second);
					//						currPath.insert(currPort.second);

					//						pathsLatPort[nextLatPort]=pathsLatPort[currPort];
					//						pathsLatPort[nextLatPort].push_back(currPort);

					if (!lessthanII)
					{
						//std::cout << "Current Port = " << currPort.second->getFullName() << " nextPort = "<< nextLatPort.second->getFullName() << "\n";
						std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
						newPath->insert(currPort.second);
						port_heuristic ph(nextLatPort, end, nextPortCost, newPath);
						//std::cout << "currPort = " << currPort.second->getFullName() << " nextLatPort" << nextLatPort.second->getFullName() << " cost=" << nextPortCost << "\n";
						ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
						ph.pathVec->push_back(currPort);
						q.push(ph);
					}
					else
					{
						//std::cout << "Next Port = " << nextLatPort.second->getFullName() << "\n";
						q.push(port_heuristic(nextLatPort, end, nextPortCost));
					}
				}
			}
			else
			{
				assert(false);
				if (detailedDebug)
					std::cout << "\t[MAPPED=" << nextPort->getNode()->idx << "]nextPort=" << nextPort->getFullName() << "\n";
			}
		}
		if (q.size() == q_len)
		{
			deadEnds.push_back(currPort);
		}
	}

	if (cameFrom.find(end) == cameFrom.end())
	{
		path.clear();
		for (LatPort p : deadEnds)
		{
			std::vector<LatPort> tmpPath;
			while (p != start)
			{
				releaseTempReservedVectorPort(p.second);
				tmpPath.push_back(p);
				assert(cameFrom.find(p) != cameFrom.end());
				p = cameFrom[p];
			}
			tmpPath.push_back(start);
			std::reverse(tmpPath.begin(), tmpPath.end());

			for (LatPort p2 : tmpPath)
			{
				path.push_back(p2);
			}

		}
		//std::cout << "Path Failure from=" << node->idx << "  DEST=" << end.second->getFullName() << "\n";
		return false; //routing failure
	}

	path.clear();
	//		assert(currPort==end);
	//		assert(currPort==end);
	currPort = end;
	while (currPort != start)
	{
		path.push_back(currPort);
		//std::cout << "Current Port = " << currPort.second->getFullName() << "\n";
		assert(cameFrom.find(currPort) != cameFrom.end());
		assert(currPort != cameFrom[currPort]);
		currPort = cameFrom[currPort];
	}
	path.push_back(start);
	std::reverse(path.begin(), path.end());
	cost = cost_to_port[end];

	cost += endDP->getPotOutputPort(currNode)->getCongCost();


	//check if paths is working
	if (!lessthanII)
	{
		paths[end]->insert(end.second);
		finalPath.push_back(end);
		/*if (paths[end]->size() != path.size())
		{
			std::cout << "paths[end] size = " << paths[end]->size() << ",path.size() = " << path.size() << "\n";

			std::cout << "path = \n";
			for (LatPort lp : path)
			{
				std::cout << lp.second->getFullName() << ",lat=" << lp.first << "\n";
				if (paths[end]->find(lp.second) == paths[end]->end())
				{
					std::cout << "Not found in paths!\n";
					//					assert(false);
				}
			}

			std::cout << "paths[end] = \n";
			for (Port *p : *paths[end])
			{
				std::cout << p->getFullName() << "\n";
			}

			std::cout << "finalPath = \n";
			for (LatPort lp : finalPath)
			{
				std::cout << lp.second->getFullName() << ",lat=" << lp.first << "\n";
			}
			//				assert(false);
		}*/
		//			assert(paths[end]->size() == path.size());
		path.clear();
		path = finalPath;
	}

	//std::cout << "Path Exists from=" << node->idx << "  DEST=" << end.second->getFullName() << "\n";
	return true;
}

//Estimating routing for vectorized execution
bool CGRAXMLCompile::PathFinderMapper::estimateRoutingVec(DFGNode *node,
													   std::priority_queue<dest_with_cost> &estimatedRoutes,
													   DFGNode **failedNode)
{

	std::map<DFGNode *, std::vector<Port *>> possibleStarts;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	bool detailedDebug = false;

	LOG(ROUTE) << "EstimateEouting begin...\n";

	for (DFGNode *parent : node->parents)
	{

		LOG(ROUTE) << "parent = " << parent->idx << "\n";
		if (parent->rootDP != NULL)
		{
			LOG(ROUTE) << "add parent to starts = " << parent->idx << "\n";
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

			for (std::pair<Port *, int> pair : parent->routingPorts)
			{
				Port *p = pair.first;
				assert(p->getLat() != -1);
			}
		}
	}

	for (DFGNode *child : node->children)
	{
		if (child->rootDP != NULL)
		{
			LOG(ROUTE)<< "child=" << child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
			assert(child->rootDP->getLat() != -1);
			if (node->childrenOPType[child] == "PS")
			{
				LOG(ROUTE)<< "Skipping.....\n";
				continue;
			}
			assert(child->rootDP->getInPort(node->childrenOPType[child]));
			alreadyMappedChildPorts[child] = child->rootDP->getInPort(node->childrenOPType[child]);

			int ii = child->rootDP->getCGRA()->get_t_max();
			assert(child->rootDP->getLat() != -1);

			if(node->childNextIter[child] == 1){
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii);
			}else if (node->childNextIter[child] == 0){
				if(mapping_method_name.find("SA") != std::string::npos){
					assert(false);
				}
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() );
			}else {
				assert(false);
			}

		}
		else if(child->idx == node->idx){
			alreadyMappedChildPorts[child] == NULL;
		}
	}

	std::vector<DataPath *> candidateDests;
	int penalty = 0;
	std::map<DataPath *, int> dpPenaltyMap;

	unordered_set<PE *> allPEs = cgra->getAllPEList();


	for (PE *currPE : allPEs)
	{

		for (Module *submod : currPE->subModules)
		{
#ifdef FLEX
			if (submod->isReserved)
			{
				printf("Module Name = %s\n",submod->getFullName().c_str());
				continue;
			}
#endif
			if (FU *fu = dynamic_cast<FU *>(submod))
			{

				if (fu->supportedOPs.find(node->op) == fu->supportedOPs.end())
				{
					continue;
				}

				string str1="IterationCounter";
				if(((node->op.compare("SELECT") == 0) and node->idx == 0) && (strstr(submod->getFullName().c_str(),str1.c_str())))
				{
						//printf("Iteration counter found !");
						for (Module *submodFU : fu->subModules)
						{
							if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
							{
								candidateDests.push_back(dp);

								continue;
							}
						}
				}

				if ((fu->currOP.compare(node->op) == 0)  && !((node->op.compare("SELECT") == 0) and node->idx == 0))
				{

					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							if(cgra->is_spm_modelled){
								if(!node->base_pointer_name.empty()){
									//base pointer name is not empty
									if(dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end()){
										//this dp does not support the variable
										continue;
									}
								}
							}

							if (checkDPFree(dp, node, penalty))
							{

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									candidateDests.push_back(dp);
									dpPenaltyMap[dp] = penalty;
								}
							}
						}
					}
				}
				else if ((fu->currOP.compare("NOP") == 0) && !((node->op.compare("SELECT") == 0) and node->idx == 0) )
				{

					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							bool is_mem_op = node->op.find("LOAD") != string::npos || node->op.find("STORE") != string::npos;
							if(cgra->is_spm_modelled){
								if(!node->base_pointer_name.empty() && is_mem_op){
									//base pointer name is not empty
									if(dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end()){
										//this dp does not support the variable
										//cout << "memvar=" << node->base_pointer_name <<  " is not supported in " << dp->getFullName() << "\n";
										continue;
									}
								}
							}

							if (checkDPFree(dp, node, penalty))
							{

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									candidateDests.push_back(dp);
									dpPenaltyMap[dp] = penalty;
								}
							}
						}
					}
				}
			}
		}
	}

	LOG(ROUTE)<< "Candidate Dests = " << candidateDests.size() << "\n";
	printf("Candidate destination size = %d\n",int(candidateDests.size()));
	if (candidateDests.empty())
		return false;

	LOG(ROUTE)<< "getlatMinStartsPHI\n";
	int minLat = getlatMinStartsPHI(node, possibleStarts);
	LOG(ROUTE)<< "getLatCandDests\n";
	std::map<DataPath *, int> minLatDests = getLatCandDests(candidateDests, minLat);
	bool changed = false;
	LOG(ROUTE)<< "modifyMaxLatCandDest\n";
	candidateDests = modifyMaxLatCandDest(minLatDests, node, changed);
	LOG(ROUTE)<< "Candidate Dests = " << candidateDests.size() << "\n";
	int ii = this->cgra->get_t_max();

	int minLatSucc = 1000000000;
	std::priority_queue<dest_with_cost> estimatedRoutesTemp;

	int allowed_time_steps_for_connection = 30;
	int iterations = allowed_time_steps_for_connection;


	for (int i = 0; i < iterations; ++i)
	{
		bool pathFromParentExist = false;
		bool pathExistMappedChild = false;

		for (DataPath *dest : candidateDests)
		{
			int minLatDestVal_prime = minLatDests[dest] + ii * i;
			std::stringstream output_stream;
			output_stream << "Candidate Dest =" ;
			output_stream << dest->getPE()->getName() << ".";
			output_stream << dest->getFU()->getName() << ".";
			output_stream<< dest->getName() << "\n";
			LOG(ROUTE)<<output_stream.str();

			//		std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
			std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
			int minLatDestVal = minLatDestVal_prime;
			pathFromParentExist = true;
			//printf("Possible start points..\n");
			for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
			{
				LOG(ROUTE)<<"estimate parent:"<<pair.first->idx<<" port size:"<<pair.second.size();
				DFGNode *parent = pair.first;

				//Skip parent if the edge is pseudo
				if (parent->getOPtype(node) == "PS")
					continue;

				Port *destPort = dest->getInPort(parent->getOPtype(node));
				minLatDestVal = minLatDestVal_prime + parent->childNextIter[node] * ii;

				std::priority_queue<cand_src_with_cost> res;
				//printf("startCand path..\n");
				for (Port *startCand : pair.second)
				{
					int cost;
					std::vector<LatPort> path;
					std::map<Port *, std::set<DFGNode *>> mutexPaths;
					LOG(ROUTE)<< "par Estimating Path" << startCand->getFullName() << "," << startCand->getLat() << ","
								  << "--->" << destPort->getFullName() << "," << minLatDestVal << "," << ",parent_node = " << parent->idx
								  << "\n";

					LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
					assert(startCand->getLat() != -1);
					LatPort destPortLat = std::make_pair(minLatDestVal, destPort);

					LOG(ROUTE) << "lat = " << destPortLat.first << ",PE=" << destPort->getMod()->getPE()->getName() << ",t=" <<  destPort->getMod()->getPE()->T << "\n";
					assert((minLatDestVal) % destPort->getMod()->getCGRA()->get_t_max() == destPort->getMod()->getPE()->T);

					bool pathExist = false;
					{
						FU *parentFU = dest->getFU();
						assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
						int latency = parentFU->supportedOPs[node->op];
						Port *destPort = dest->getOutputPort(latency);
						LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

						if (canExitCurrPE(destPortLat))
						{
							pathExist = true;
						}
						else
						{
							LOG(ROUTE)<< "Cannot exit from :" << destPortLat.second->getFullName() << "\n";
						}
					}
					//printf("Before leastcost path..\n");
					pathExist = pathExist & LeastCostPathAstar(startCandLat, destPortLat, dest, path, cost, parent, mutexPaths, node);
					path.clear();
					if (!pathExist)
					{
						LOG(ROUTE)<< "par Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
						continue;
					}
					cost += dpPenaltyMap[dest];
					res.push(cand_src_with_cost(startCandLat, destPortLat, cost));
				}
				if (res.empty())
				{
					pathFromParentExist = false;
					*failedNode = parent;
					break;
				}
				parent_cand_src_with_cost pcswc(parent, res);
				parentStartLocs.push(pcswc);
			}

			if (!pathFromParentExist)
			{
				continue;
			}


			pathExistMappedChild = true;
			std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
			for (std::pair<DFGNode *, Port *> pair : alreadyMappedChildPorts)
			{
				DFGNode *child = pair.first;
				Port *childDestPort = pair.second;
				DataPath* childDP = child->rootDP;

				if (child->idx == node->idx)
				{
					childDestPort = dest->getInPort(node->childrenOPType[child]);
					LOG(ROUTE) << "setting latency = " << minLatDestVal + ii << "\n";
					childDestPort->setLat(minLatDestVal + ii);
					childDP = dest;
				}

				std::vector<LatPort> path;
				int cost;

				FU *parentFU = dest->getFU();
				assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
				int latency = parentFU->supportedOPs[node->op];
				Port *destPort = dest->getOutputPort(latency);

				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				LOG(ROUTE)<< "already child Estimating Path" << destPort->getFullName() << "," << minLatDestVal + latency << ","
							  << "--->" << childDestPort->getFullName() << "," << childDestPort->getLat() << "," << "exist_child = " << child->idx
							  << "\n";
				LOG(ROUTE)<< "lat = " << childDestPort->getLat() << ",PE=" << childDestPort->getMod()->getPE()->getName() << ",t=" << childDestPort->getMod()->getPE()->T << "\n";

				LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
				assert(childDestPort->getLat() != -1);
				LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

				pathExistMappedChild = pathExistMappedChild & LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, cost, node, mutexPaths, child);

				if (!pathExistMappedChild)
				{
					*failedNode = child;
					break;
				}

				dest_child_with_cost dcwc(child,childDP, childDestPortLat, destPortLat, cost);
				alreadyMappedChilds.push(dcwc);
			}
			if (!pathExistMappedChild)
			{
				if (detailedDebug)
					LOG(ROUTE)<< "already child Estimating Path Failed!\n";
				continue; //if it cannot be mapped to child abort the estimation for this dest
			}

			assert(pathFromParentExist);
			assert(pathExistMappedChild);
			dest_with_cost dest_with_cost_ins(parentStartLocs, alreadyMappedChilds, dest, minLatDestVal_prime, node, 0, this->dfg->unmappedMemOps, this);

			if (minLatDestVal_prime < minLatSucc)
			{
				minLatSucc = minLatDestVal_prime;
			}

			estimatedRoutesTemp.push(dest_with_cost_ins);
		}
		//printf("Estimation Done....\n");
		if (pathFromParentExist & pathExistMappedChild)
			break;
	}

	while (!estimatedRoutesTemp.empty())
	{
		dest_with_cost top = estimatedRoutesTemp.top();
		estimatedRoutesTemp.pop();
		if (minLatDests[top.dest] == minLatSucc || !changed)
			estimatedRoutes.push(top);
	}

	//	std::cout << "EstimateEouting end!\n";
	//	if(estimatedRoutes.empty()) assert(*failedNode!=NULL);
	return !estimatedRoutes.empty();
}

bool CGRAXMLCompile::PathFinderMapper::estimateRouting(DFGNode *node,
													   std::priority_queue<dest_with_cost> &estimatedRoutes,
													   DFGNode **failedNode)
{

	std::map<DFGNode *, std::vector<Port *>> possibleStarts;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	bool detailedDebug = false;

		std::cout<< "EstimateEouting begin...\n";

	for (DFGNode *parent : node->parents)
	{

		//std::cout << "parent = " << parent->idx << "\n";
		if(parent->idx ==0)
		{
			for (DataPath* rootDP: parent->rootDP_vec)
			{

				//std::cout << "add parent to starts = " << parent->idx << "\n";
				assert(rootDP->getOutputDP()->getOutPort("T"));
				if(std::find(possibleStarts[parent].begin(), possibleStarts[parent].end(), rootDP->getOutputDP()->getOutPort("T")) == possibleStarts[parent].end())
					possibleStarts[parent].push_back(rootDP->getOutputDP()->getOutPort("T"));
				//printf("Possible starts : %s\n", rootDP->getOutputDP()->getOutPort("T")->getFullName().c_str());
				for (std::pair<Port *, int> pair : parent->routingPorts)
				{
					Port *p = pair.first;
				//	std::cout << p->getFullName() << "\n";
					//assert(p->getLat() != -1);
				}
			}
		}
		else
		{
			if (parent->rootDP != NULL)
			{
				LOG(ROUTE) << "add parent to starts = " << parent->idx << "\n";
				assert(parent->rootDP->getOutputDP()->getOutPort("T"));
				possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

				for (std::pair<Port *, int> pair : parent->routingPorts)
				{
					Port *p = pair.first;
				//	std::cout << p->getFullName() << "\n";
				//	assert(p->getLat() != -1);
				}
			}
		}
	}

	for (DFGNode *child : node->children)
	{
		//std::cout << "child = " << child->idx << "\n";
		if (child->rootDP != NULL)
		{
			LOG(ROUTE)<< "child=" << child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
			assert(child->rootDP->getLat() != -1);
			if (node->childrenOPType[child] == "PS")
			{
				LOG(ROUTE)<< "Skipping.....\n";
				continue;
			}
			assert(child->rootDP->getInPort(node->childrenOPType[child]));
			alreadyMappedChildPorts[child] = child->rootDP->getInPort(node->childrenOPType[child]);

			int ii = child->rootDP->getCGRA()->get_t_max();
			assert(child->rootDP->getLat() != -1);

			if(node->childNextIter[child] == 1){
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii);
			}else if (node->childNextIter[child] == 0){
				if(mapping_method_name.find("SA") != std::string::npos){
					assert(false);
				}
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() );
			}else {
				assert(false);
			}
			
		}
		else if(child->idx == node->idx){
			alreadyMappedChildPorts[child] == NULL;
		}
	}

	std::vector<DataPath *> candidateDests;
	int penalty = 0;
	std::map<DataPath *, int> dpPenaltyMap;

	unordered_set<PE *> allPEs = cgra->getAllPEList();

	//std::cout << "selecting candidate dests\n";
	for (PE *currPE : allPEs)
	{

		for (Module *submod : currPE->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(submod))
			{

				if (fu->supportedOPs.find(node->op) == fu->supportedOPs.end())
				{
					continue;
				}

				string str1="IterationCounter";
				if(((node->op.compare("SELECT") == 0) and node->idx == 0) && (strstr(submod->getFullName().c_str(),str1.c_str())))
				{
						//printf("Iteration counter found !");
						for (Module *submodFU : fu->subModules)
						{
							if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
							{
								//if(((node->idx == 38) && (currPE->Y == 1)) || (node->idx != 38))
									candidateDests.push_back(dp);

								continue;
							}
						}
				}

				if ((fu->currOP.compare(node->op) == 0)  && !((node->op.compare("SELECT") == 0) and node->idx == 0))
				{

					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							if(cgra->is_spm_modelled){
								if(!node->base_pointer_name.empty()){
									//base pointer name is not empty
									if(dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end()){
										//this dp does not support the variable
										continue;
									}
								}
							}
							
							if (checkDPFree(dp, node, penalty))
							{

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
#ifdef FLEX
									if (dp->isReserved)
									{
										//printf("[RESERVED DP] Module Name = %s\n",submod->getFullName().c_str());
										continue;
									}else{
										//printf("[NOT RESERVED] Module Name = %s\n",submod->getFullName().c_str());
										if(checkVectorDP(dp)){
											//if(((node->idx == 38) && (currPE->Y == 1)) || (node->idx != 38))
											candidateDests.push_back(dp);
										}
									}
#else
									candidateDests.push_back(dp);
#endif
									dpPenaltyMap[dp] = penalty;
								}
							}
						}
					}
				}
				else if ((fu->currOP.compare("NOP") == 0) && !((node->op.compare("SELECT") == 0) and node->idx == 0) )
				{
					//printf("Inside else if\n");
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							bool is_mem_op = node->op.find("LOAD") != string::npos || node->op.find("STORE") != string::npos;
							if(cgra->is_spm_modelled){
								if(!node->base_pointer_name.empty() && is_mem_op){
									//base pointer name is not empty
									if(dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end()){
										//this dp does not support the variable
										//cout << "memvar=" << node->base_pointer_name <<  " is not supported in " << dp->getFullName() << "\n";
										continue;
									}
								}
							}

							if (checkDPFree(dp, node, penalty))
							{
								//printf("Inside else if if\n");
								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									//printf("Inside else if if if\n");
#ifdef FLEX
									//printf("Inside else if if if\n");
									if (dp->isReserved)
									{
										//printf("[RESERVED DP] Module Name = %s\n",submod->getFullName().c_str());
										continue;
									}else{
										//printf("[NOT RESERVED] Module Name = %s reserved=%d\n",submod->getFullName().c_str(),dp->isReserved);
										//printf("Inside else if if if else\n");
										if(checkVectorDP(dp))
										{
											//if(((node->idx == 38) && (currPE->Y == 1)) || (node->idx != 38))
											   candidateDests.push_back(dp);
										}
									}
#else
									candidateDests.push_back(dp);
#endif
									dpPenaltyMap[dp] = penalty;
								}
							}
						}
					}
				}
			}
		}
	}

	LOG(ROUTE)<< "Candidate Dests = " << candidateDests.size() << "\n";
	printf("Candidate destination size = %d\n",int(candidateDests.size()));
	//for (DataPath *dest : candidateDests)
	//{
	//	std::cout << "Candidate Dest = " << dest->getFullName() << "\n";
	//}
	if (candidateDests.empty())
		return false;

	std::cout << "getlatMinStartsPHI\n";
	int minLat = getlatMinStartsPHI(node, possibleStarts);
	std::cout << "minLat=" << minLat << "\n";

#ifdef FLEX
	//Redefine MaxLat based on the ASAP value
	if (minLat < node->ASAP)
		minLat = node->ASAP;
#endif
	std::cout << "getLatCandDests\n";
	std::map<DataPath *, int> minLatDests = getLatCandDests(candidateDests, minLat);
	bool changed = false;
	printf("Candidate destination size = %d\n",int(minLatDests.size()));
	std::cout << "modifyMaxLatCandDest\n";
	candidateDests = modifyMaxLatCandDest(minLatDests, node, changed);
	std::cout << "Candidate Dests = " << candidateDests.size() << "\n";
	int ii = this->cgra->get_t_max();

	int minLatSucc = 1000000000;
	std::priority_queue<dest_with_cost> estimatedRoutesTemp;

	int allowed_time_steps_for_connection = 3;
	int iterations = allowed_time_steps_for_connection;


	for (int i = 0; i < iterations; ++i)
	{
		bool pathFromParentExist = false;
		bool pathExistMappedChild = false;

		for (DataPath *dest : candidateDests)
		{

			int minLatDestVal_prime = minLatDests[dest] + ii * i;
			//printf("MinLatDests=%d\n",minLatDests[dest]);
			//std::stringstream output_stream;
			//std::cout << "Candidate Dest =" ;
			//std::cout << dest->getPE()->getName() << ".";
			//std::cout << dest->getFU()->getName() << ".";
			//std::cout << dest->getName() << "\n";
			//LOG(ROUTE)<<output_stream.str();

			//		std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
			std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
			int minLatDestVal = minLatDestVal_prime;
			pathFromParentExist = true;

			//if the parent is the phi node, it needs to have spreaded across time and start point can be any of those
			std::map<DFGNode *, std::vector<Port *>> possibleStarts_modi = selectClosestIterationCounter(possibleStarts,minLatDestVal,dest);

			for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts_modi)
			{
				//printf("Possible start points..\n");
			    //std::cout <<"Candidate= "<<dest->getFullName()<<" Start parent:"<<pair.first->idx<<" \n";
				DFGNode *parent = pair.first;

				//Skip parent if the edge is pseudo
				if (parent->getOPtype(node) == "PS")
					continue;

				Port *destPort = dest->getInPort(parent->getOPtype(node));
				minLatDestVal = minLatDestVal_prime + parent->childNextIter[node] * ii;

				std::priority_queue<cand_src_with_cost> res;
				//printf("startCand path..\n");
				for (Port *startCand : pair.second)
				{
					int cost;
					//printf("startCand path..\n");
					//printf("DFGNODE=%d StarCand=%s\n",parent->idx,startCand->getFullName().c_str());
					std::vector<LatPort> path;
					std::map<Port *, std::set<DFGNode *>> mutexPaths;
					LOG(ROUTE)<< "par Estimating Path" << startCand->getFullName() << "," << startCand->getLat() << ","
								  << "--->" << destPort->getFullName() << "," << minLatDestVal << "," << ",parent_node = " << parent->idx
								  << "\n";

					LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
					assert(startCand->getLat() != -1);
					LatPort destPortLat = std::make_pair(minLatDestVal, destPort);

					LOG(ROUTE) << "lat = " << destPortLat.first << ",PE=" << destPort->getMod()->getPE()->getName() << ",t=" <<  destPort->getMod()->getPE()->T << "\n";
					assert((minLatDestVal) % destPort->getMod()->getCGRA()->get_t_max() == destPort->getMod()->getPE()->T);

					bool pathExist = false;
					{
						FU *parentFU = dest->getFU();
						assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
						int latency = parentFU->supportedOPs[node->op];
						Port *destPort = dest->getOutputPort(latency);
						LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

						if (canExitCurrPE(destPortLat))
						{
							pathExist = true;
						}
						else
						{
							LOG(ROUTE)<< "Cannot exit from :" << destPortLat.second->getFullName() << "\n";
						}
					}
					//printf("Before leastcost path..\n");
					pathExist = pathExist & LeastCostPathAstar(startCandLat, destPortLat, dest, path, cost, parent, mutexPaths, node);
					//printf("After leastcost path..\n");
					//std::cout << "Dest= " << dest->getFullName() << " Cost= " << cost << "\n";
					path.clear();
					if (!pathExist)
					{
						LOG(ROUTE)<< "par Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
						continue;
					}
					//printf("setting cost..\n");
					cost += dpPenaltyMap[dest];
					res.push(cand_src_with_cost(startCandLat, destPortLat, cost));
					//std::cout << "start candidate = " << startCand -> getFullName() << " cost= " << cost << "\n";
					//printf("after setting cost..\n");

				}
				//printf("Outside startcand loop..\n");
				if (res.empty())
				{
					pathFromParentExist = false;
					*failedNode = parent;
					//std::cout <<"Candidate= "<<dest->getFullName()<<" Start parent:"<<parent->idx<<" \n";
					break;
				}
				//printf("setup cost ports..\n");
				parent_cand_src_with_cost pcswc(parent, res);
				parentStartLocs.push(pcswc);
				//printf("setup cost ports done..\n");
			}

			if (!pathFromParentExist)
			{

				continue;
			}
			//printf("Near children..\n");

			pathExistMappedChild = true;
			std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
			for (std::pair<DFGNode *, Port *> pair : alreadyMappedChildPorts)
			{
				DFGNode *child = pair.first;
				Port *childDestPort = pair.second;
				DataPath* childDP = child->rootDP;

				if (child->idx == node->idx)
				{
					childDestPort = dest->getInPort(node->childrenOPType[child]);
					LOG(ROUTE) << "setting latency = " << minLatDestVal + ii << "\n";
					childDestPort->setLat(minLatDestVal + ii);
					childDP = dest;
				}

				std::vector<LatPort> path;
				int cost;

				FU *parentFU = dest->getFU();
				assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
				int latency = parentFU->supportedOPs[node->op];
				Port *destPort = dest->getOutputPort(latency);

				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				LOG(ROUTE)<< "already child Estimating Path" << destPort->getFullName() << "," << minLatDestVal + latency << ","
							  << "--->" << childDestPort->getFullName() << "," << childDestPort->getLat() << "," << "exist_child = " << child->idx  
							  << "\n";
				LOG(ROUTE)<< "lat = " << childDestPort->getLat() << ",PE=" << childDestPort->getMod()->getPE()->getName() << ",t=" << childDestPort->getMod()->getPE()->T << "\n";

				LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
				assert(childDestPort->getLat() != -1);
				LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

				pathExistMappedChild = pathExistMappedChild & LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, cost, node, mutexPaths, child);
				//printf("After leastcost path child ..\n");
				//std::cout << "Dest= " << dest->getFullName() << " Cost= " << cost << "\n";
				if (!pathExistMappedChild)
				{
					*failedNode = child;
					break;
				}

				dest_child_with_cost dcwc(child,childDP, childDestPortLat, destPortLat, cost);
				alreadyMappedChilds.push(dcwc);
			}
			//printf("After children..\n");
			if (!pathExistMappedChild)
			{
				if (detailedDebug)
					LOG(ROUTE)<< "already child Estimating Path Failed!\n";
				continue; //if it cannot be mapped to child abort the estimation for this dest
			}

			assert(pathFromParentExist);
			assert(pathExistMappedChild);
			//printf("Setting up destination\n");
			//printf("[THILINI]:: parentStartLocs=%d alreadyMappedChilds=%d dest=%s minLatDestVal_prime=%d\n",parentStartLocs.size(),alreadyMappedChilds.size(),dest->getFullName().c_str(),minLatDestVal_prime);
			dest_with_cost dest_with_cost_ins(parentStartLocs, alreadyMappedChilds, dest, minLatDestVal_prime, node, 0, this->dfg->unmappedMemOps, this);
			//printf("Setting up destination done\n");
			if (minLatDestVal_prime < minLatSucc)
			{
				minLatSucc = minLatDestVal_prime;
			}

			estimatedRoutesTemp.push(dest_with_cost_ins);


		}
		//printf("Estimation Done....\n");
		if (pathFromParentExist & pathExistMappedChild)
			break;
	}

	while (!estimatedRoutesTemp.empty())
	{
		dest_with_cost top = estimatedRoutesTemp.top();
		estimatedRoutesTemp.pop();
		if (minLatDests[top.dest] == minLatSucc || !changed)
			estimatedRoutes.push(top);
	}

		//std::cout << "EstimateEouting end!\n";
	//	if(estimatedRoutes.empty()) assert(*failedNode!=NULL);
	return !estimatedRoutes.empty();
}

bool CGRAXMLCompile::PathFinderMapper::Route(DFGNode *node,
											 std::priority_queue<dest_with_cost> &estimatedRoutes,
											 DFGNode **failedNode)
{

	std::cout << "Route begin...\n";

	int parentRoutingPortCount = 0;
	int routedParents = 0;

	for (DFGNode *parent : node->parents)
	{
		int thisParentNodeCount = 0;
		if (parent->rootDP != NULL)
		{
			thisParentNodeCount = parent->routingPorts.size();
		}

		//		if(thisParentNodeCount>0){
		//			routedParents++;
		//			thisParentNodeCount--; //remove the T port in the cout
		//		}
		parentRoutingPortCount += thisParentNodeCount;
	}
	//	if(parentRoutingPortCount>0){
	//		parentRoutingPortCount-=1; //remove the T port in the cout
	//	}

	int addedRoutingParentPorts = 0;

	bool routeSucc = false;
	dest_with_cost currDest;
	while (!estimatedRoutes.empty())
	{
		currDest = estimatedRoutes.top();
		estimatedRoutes.pop();

		if (currDest.dest->getMappedNode() != NULL)
		{
			std::cout << "currDest is not NULL \n";
			std::cout << "currDP:" << currDest.dest->getName() << ",currPE:" << currDest.dest->getPE()->getName() << "\n";
			std::cout << "currNode:" << currDest.dest->getMappedNode()->idx << "\n";
		}
		assert(currDest.dest->getMappedNode() == NULL);
		std::cout << "alreadyMappedChilds = " << currDest.alreadyMappedChilds.size() << "\n";

		bool alreadMappedChildRouteSucc = true; //this will change to false if failure in alreadyMappedChilds
		std::map<DFGNode *, std::vector<LatPort>> mappedChildPaths;
		std::map<DFGNode *, std::map<Port *, std::set<DFGNode *>>> mappedChildMutexPaths;
		while (!currDest.alreadyMappedChilds.empty())
		{
			dest_child_with_cost dest_child_with_cost_ins = currDest.alreadyMappedChilds.top();
			currDest.alreadyMappedChilds.pop();

			std::vector<LatPort> possibleStarts;
			possibleStarts.clear();
			possibleStarts.push_back(dest_child_with_cost_ins.startPort);
			for (std::pair<Port *, int> pair : node->routingPorts)
			{
				possibleStarts.push_back(std::make_pair(pair.first->getLat(), pair.first));
				assert(pair.first->getLat() != -1);
			}

			std::priority_queue<cand_src_with_cost> q;
			std::map<Port *, std::set<DFGNode *>> mutexPathsTmp;
			std::vector<LatPort> pathTmp;
			for (LatPort p : possibleStarts)
			{
				int cost;
				//std::cout << "Before least cost...\n";
				//std::cout << "Start Port-1 = " << p.second->getFullName() << "\n";
				if (LeastCostPathAstar(p, dest_child_with_cost_ins.childDest, dest_child_with_cost_ins.childDP, pathTmp, cost, node, mutexPathsTmp, dest_child_with_cost_ins.child))
				{
					pathTmp.clear();
					q.push(cand_src_with_cost(p, dest_child_with_cost_ins.childDest, cost));
				}
			}

			int cost;
			std::vector<LatPort> path;
			LatPort src = dest_child_with_cost_ins.startPort;
			LatPort dest = dest_child_with_cost_ins.childDest;

			while (!q.empty())
			{
				cand_src_with_cost head = q.top();
				q.pop();
				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				//std::cout << "Before least cost 2...\n";
				//std::cout << "Start Port-2 = " << head.src.second->getFullName() << "\n";
				alreadMappedChildRouteSucc = LeastCostPathAstar(head.src, dest, dest_child_with_cost_ins.childDP, path, cost, node, mutexPaths, dest_child_with_cost_ins.child);
				if (alreadMappedChildRouteSucc)
				{
					assignPath(node, dest_child_with_cost_ins.child, path);
					mappedChildPaths[dest_child_with_cost_ins.child] = path;
					mappedChildMutexPaths[dest_child_with_cost_ins.child] = mutexPaths;
					std::cout << "Route success :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << "\n";
					break;
				}
				else
				{
					std::cout << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
					for (LatPort p : path)
					{
						if (p.second->getMod()->getPE())
						{
							std::cout << p.second->getMod()->getPE()->getName() << "-->";
						}
					}
					std::cout << "\n";

					for (LatPort p : path)
					{
						std::cout << p.second->getFullName() << "\n";
					}
				}
				path.clear();
			}
			if (!alreadMappedChildRouteSucc)
			{
				*failedNode = dest_child_with_cost_ins.child;
				break;
			}
		}

		if (alreadMappedChildRouteSucc)
		{
			for (std::pair<Port *, int> pair : node->routingPorts)
			{
				Port *p = pair.first;
				int destIdx = pair.second;
				std::cout << "to:" << destIdx << "," << p->getFullName() << "\n";
			}
		}

		if (!alreadMappedChildRouteSucc)
		{
			node->clear(this->dfg);
			continue; //try the next dest
		}
		else
		{
			std::cout << "Already Mapped child Routes....\n";
			for (std::pair<DFGNode *, std::vector<LatPort>> pair : mappedChildPaths)
			{
				DFGNode *child = pair.first;
				for (LatPort lp : pair.second)
				{
					Port *p = lp.second;
					std::cout << "to:" << child->idx << " :: ";
					std::cout << p->getFullName();
					if (mappedChildMutexPaths[child].find(p) != mappedChildMutexPaths[child].end())
					{
						std::cout << "|mutex(";
						for (DFGNode *mutexnode : mappedChildMutexPaths[child][p])
						{
							std::cout << mutexnode->idx << ",";
						}
						std::cout << ")";
					}
					std::cout << "\n";
				}
				std::cout << "\n";
			}
			std::cout << "\n";
		}

		bool parentRoutSucc = true;
		addedRoutingParentPorts = 0;
		std::map<DFGNode *, std::map<Port *, std::set<DFGNode *>>> mappedParentMutexPaths;
		while (!currDest.parentStartLocs.empty())
		{
			parent_cand_src_with_cost pcswc = currDest.parentStartLocs.top();
			currDest.parentStartLocs.pop();
			DFGNode *parent = pcswc.parent;
			std::priority_queue<cand_src_with_cost> &q = pcswc.cswc;

			bool succ = false;
			while (!q.empty())
			{
				cand_src_with_cost cand_src_with_cost_ins = q.top();
				q.pop();
				LatPort src = cand_src_with_cost_ins.src;
				LatPort dest = cand_src_with_cost_ins.dest;
				std::vector<LatPort> path;
				std::map<Port *, std::set<DFGNode *>> mutexPath;
				int cost;
				//std::cout << "Before least cost 3...\n";
				//std::cout << "Start Port-3 = " << src.second->getFullName() << "\n";
				succ = LeastCostPathAstar(src, dest, currDest.dest, path, cost, parent, mutexPath, node);
				//std::cout << "done least cost 3...\n";
				if (succ)
				{

					//					bool routedParent=true;
					//					if(parent->routingPorts.size()==0){ //unrouted parent
					//						routedParent=false;
					//					}
					//std::cout << "before assign path...\n";
					assignPath(parent, node, path);
					//std::cout << "after assign path...\n";
					mappedParentMutexPaths[parent] = mutexPath;
					addedRoutingParentPorts += path.size();
					//					if(routedParent){
					addedRoutingParentPorts -= 1;
					//					}
					//					for(Port* p : path){
					//						std::cout << p->getFullName() << ",\n";
					//					}
					//					std::cout << "\n";
					break;
				}
				else
				{
					addedRoutingParentPorts = 0;
					node->clear(this->dfg);
					std::cout << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
				}
				path.clear();
			}
			if (!succ)
			{
				*failedNode = parent;
				node->clear(this->dfg);
				addedRoutingParentPorts = 0;
				parentRoutSucc = false; // at least one parent failed to route, try a new dest
				break;
			}
		}

		if (parentRoutSucc)
		{ //all parents routed succesfull + all mapped childs are connected
			routeSucc = true;
			std::cout << "node=" << node->idx << ",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << ",lat=" << currDest.destLat << "\n";
			std::cout << "routing info ::\n";
			for (DFGNode *parent : node->parents)
			{
				printf("Parent=%d\n",parent->idx);
				std::cout << "parent routing port size = " << parent->routingPorts.size() << "\n";
				int prev_lat = -1;
				for (std::pair<Port *, int> pair : parent->routingPorts)
				{
					Port *p = pair.first;
					//					if(node.routingPortDestMap[p]==&node){
					std::cout << "fr:" << parent->idx << " :: ";
					std::cout << ",dest=" << pair.second << " :: ";
					std::cout << p->getFullName();
					std::cout << ",lat=" << p->getLat();

					if (mappedParentMutexPaths[parent].find(p) != mappedParentMutexPaths[parent].end())
					{
						std::cout << "|mutex(";
						for (DFGNode *mutexnode : mappedParentMutexPaths[parent][p])
						{
							std::cout << mutexnode->idx << ",";
						}
						std::cout << ")";
					}
					std::cout << std::endl;
					//					}
					if (prev_lat != -1)
					{
						//							assert(p->getLat() - prev_lat <= 1);
					}
					prev_lat = p->getLat();
				}
			}
			std::cout << "routing info done.\n";
			currDest.dest->assignNode(node, currDest.destLat, this->dfg);

#ifdef FLEX
			if(node->idx != 0)
				reserveVectorNodes(currDest.dest,node,currDest.destLat);
#endif
			mappingLog4 << node->idx << "," << currDest.dest->getPE()->X << ","<< currDest.dest->getPE()->Y << "," << currDest.destLat << "\n";
			std::cout << "mappingLog4=" << node->idx << "," << currDest.dest->getPE()->X << ","<< currDest.dest->getPE()->Y << "," << currDest.destLat << "\n";
			node->rootDP = currDest.dest;

			if(node->idx==0)
			{
					setIterDP(node, currDest.dest, currDest.destLat);
			}

			break;
		}
		node->clear(this->dfg);
	}

	if (routeSucc)
	{
		std::cout << "Route success...\n";

		int parentRoutingPortCountEnd = 0;
		//		int mappedParentCount=0;
		for (DFGNode *parent : node->parents)
		{
			if (parent->rootDP != NULL)
			{
				//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd = std::max(0, parentRoutingPortCountEnd - routedParents);
		if (parentRoutingPortCountEnd != parentRoutingPortCount + addedRoutingParentPorts)
		{
			std::cout << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			std::cout << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			std::cout << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}

		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount+addedRoutingParentPorts);
		return true;
	}
	else
	{
		currDest.dest->assignNode(node, currDest.destLat, this->dfg);
#ifdef FLEX
		reserveVectorNodes(currDest.dest,node,currDest.destLat);
#endif
		node->rootDP = currDest.dest;

		if(node->idx==0)
		{
				setIterDP(node, currDest.dest, currDest.destLat);
		}

		//if(node->idx==0)
		//	printf("PAMU\n");
		node->clear(this->dfg);
		std::cout << "Route failed...\n";

		int parentRoutingPortCountEnd = 0;
		//		int mappedParentCount=0;
		for (DFGNode *parent : node->parents)
		{
			if (parent->rootDP != NULL)
			{
				//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd = std::max(0, parentRoutingPortCountEnd - routedParents);
		if (parentRoutingPortCountEnd != parentRoutingPortCount + addedRoutingParentPorts)
		{
			std::cout << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			std::cout << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			std::cout << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}
		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount);
		assert(*failedNode != NULL);
		return false;
	}
}

int CGRAXMLCompile::PathFinderMapper::calculateCost(LatPort src,
													LatPort next_to_src, LatPort dest)
{

	std::string srcName = src.second->getName();
	//std::cout << src.second->getName() << ",";

	std::string next_to_srcName = next_to_src.second->getName();
	//std::cout << next_to_srcName << "\n";

	assert(src.second);
	assert(next_to_src.second);
	assert(dest.second);

	PE *srcPE = src.second->findParentPE();
	assert(srcPE);
	PE *nextPE = next_to_src.second->findParentPE();
	assert(nextPE);
	//int distance = abs(nextPE->Y - srcPE->Y) + abs(nextPE->X - srcPE->X);
	int distance = abs(nextPE->Y - srcPE->Y) + abs(nextPE->X - srcPE->X) + regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());
	//int distance = regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());
	//std::cout << "Distance1 = " <<  distance << "\n";

	//if(src.second->getName().compare("OUT_O") != 0)
		distance = distance * PETransitionCostFactor + next_to_src.second->getCongCost() + PortTransitionCost;
	//else
	//	distance = distance * PETransitionCostFactor + PortTransitionCost;
	//std::cout << "Distance2 = " <<  distance  << " first = " << distance * PETransitionCostFactor  << " second= "<< next_to_src.second->getCongCost() << " third=" << PortTransitionCost << "\n";
	assert(distance > 0);

	if (srcPE != nextPE)
	{
		int freePorts = 0;

		for (Port *p : nextPE->outputPorts)
		{
			Module *parent = nextPE->getParent();
			if (parent->getNextPorts(std::make_pair(next_to_src.first, p), this).empty())
				continue;
			if (p->getNode() == NULL)
			{
				freePorts++;
			}
		}

		//		for(Port &p : nextPE->inputPorts){
		//			Module* parent = nextPE->getParent();
		//			if(parent->getFromPorts(&p,this).empty()) continue;
		//			if(p.getNode()==NULL){
		//				freePorts++;
		//			}
		//		}

		//		distance = distance + (nextPE->outputPorts.size() + nextPE->inputPorts.size() - freePorts)*UOPCostFactor;
		//		distance = distance + (nextPE->outputPorts.size() - freePorts)*UOPCostFactor;
		//		distance = distance + (1 + nextPE->outputPorts.size() - freePorts)*UOPCostFactor;
		//distance = distance + ((nextPE->outputPorts.size() * 2 - (freePorts))/nextPE->outputPorts.size()) * UOPCostFactor;
		distance = distance + ((nextPE->outputPorts.size() * 2 - (freePorts))) * UOPCostFactor;
		//std::cout << "Distance3 = " <<  distance << "\n";

		if (nextPE->outputPorts.size() * 2 < freePorts)
		{
			std::cout << "outportsize = " << nextPE->outputPorts.size() << "\n";
			std::cout << "freePorts = " << freePorts << "\n";
		}
	}

	//	int unmappedMemNodeCount=0;
	//	for(DFGNode* node : this->sortedNodeList){
	//		if(node->isMemOp()){
	//			if(node->rootDP==NULL){
	//				unmappedMemNodeCount++;
	//			}
	//		}
	//	}
	//	dfg->unmappedMemOps = unmappedMemNodeCount;
	assert(distance > 0);

	if ((next_to_src.second->getName().compare("P") == 0) || (next_to_src.second->getName().compare("I1") == 0) || (next_to_src.second->getName().compare("I2") == 0))
	{

		FU *fu = next_to_src.second->getMod()->getFU();
		if ((fu->supportedOPs.find("LOAD") != fu->supportedOPs.end()) && (dest == next_to_src))
		{
			int freeMemNodes = cgra->freeMemNodes;
			freeMemNodes = freeMemNodes>0? freeMemNodes:1;
			double memrescost_dbl = (double)this->dfg->unmappedMemOps / (double)freeMemNodes;
			memrescost_dbl = memrescost_dbl * (double)MEMResourceCost;
			
			distance = distance + (int)memrescost_dbl;
			if (this->dfg->unmappedMemOps == cgra->freeMemNodes)
			{
				distance = distance + MRC * 10;
				//std::cout << "Distance4 = " <<  distance << "\n";
				
			}

		}
	}

	assert(distance > 0);
	return distance;
}

bool CGRAXMLCompile::PathFinderMapper::Map(CGRA *cgra, DFG *dfg)
{
	//std::cout << "Here00\n";
	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	int backTrackCredits = this->backTrackLimit;

	//Disable mutex paths to test pathfinder
	this->enableMutexPaths = true;

	this->cgra = cgra;
	this->dfg = dfg;

	Check_DFG_CGRA_Compatibility();
	//std::cout << "Here0\n";

	if(cgra->is_spm_modelled){
		UpdateVariableBaseAddr();
	}
	//std::cout << "Base addr updated\n";
	//Testing 1 2 3
	//getLongestDFGPath(dfg->findNode(1093),dfg->findNode(82));

	//	SortSCCDFG();
	//	SortTopoGraphicalDFG();
	//sortBackEdgePriorityASAP();
	sortBackEdgePriorityALAP();
	//std::vector<int> nodeOrder {0,25,38,29,40,39,33,30,35,32,10,22,19,13,23,20,11,14,24,21,15,12};
	std::vector<int> nodeOrder {0,35,32,25,29,38,33,40,39,30,14,20,23,11,24,21,15,12,10,22,19,13};
	//sortGivenOrder(nodeOrder);
	//std::cout << "Sort Done\n";

	std::string mappingLogFileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths);  // + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths); // + ".routeInfo.log";
	
	
	//std::cout << "Here\n";
	bool mapSuccess = false;

	std::string congestionInfoFileName = mappingLogFileName + ".congestion.info";
	cout << "Opening congestion file : " << congestionInfoFileName << "!\n";
	congestionInfoFile.open(congestionInfoFileName.c_str());
	assert(congestionInfoFile.is_open());

	for (int i = 0; i < this->maxIter; ++i)
	{

		std::string mappingLogFileName_withIter = mappingLogFileName + "_Iter=" + std::to_string(i) + ".mapping.csv";
		std::string mappingLog2FileName_withIter = mappingLog2FileName + "_Iter=" + std::to_string(i) + ".routeInfo.log";
		std::string mappingLog4FileName_withIter = mappingLogFileName + "_II=" + std::to_string(cgra->get_t_max())+ "_Iter=" + std::to_string(i) + ".mappingwithlatency.txt";

		mappingLog.open(mappingLogFileName_withIter.c_str());
		mappingLog2.open(mappingLog2FileName_withIter.c_str());
		mappingLog4.open(mappingLog4FileName_withIter.c_str());

		cout << "Opening mapping csv file : " << mappingLogFileName_withIter << "\n";
		cout << "Opening routeInfo log file : " << mappingLog2FileName_withIter << "\n";

		assert(mappingLog.is_open());
		assert(mappingLog2.is_open());
		assert(mappingLog4.is_open());

		while (!mappedNodes.empty())
		{
			mappedNodes.pop();
		}
		while (!unmappedNodes.empty())
		{
			unmappedNodes.pop();
		}

		for (DFGNode *node : sortedNodeList)
		{
			unmappedNodes.push(node);
		}

		//std::cout << "MAP begin...\n";

		while (!unmappedNodes.empty())
		{

			DFGNode *node = unmappedNodes.top();
			unmappedNodes.pop();

			std::stringstream MapHeader;
			MapHeader << "current node = " << node->idx;
			MapHeader << ",op = " << node->op;
			MapHeader << ",unmapped nodes = " << unmappedNodes.size();
			MapHeader << ",mapped nodes = " << mappedNodes.size();
			MapHeader << ",freeMemNodes = " << cgra->freeMemNodes;
			MapHeader << ",unmappedMemNodes = " << dfg->unmappedMemOps;
			MapHeader << ",II = " << cgra->get_t_max();
			MapHeader << ",btCredits = " << backTrackCredits;

			// MapHeader << ",PEType = " << this->cgra->peType;
			// MapHeader << ",XDim = " << this->cgra->get_x_max();
			// MapHeader << ",YDim = " << this->cgra->get_y_max();
			// MapHeader << ",DPs = " << this->cgra->numberofDPs;

			MapHeader << ",CGRA=" << this->cgra->getCGRAName();
			MapHeader << ",MaxHops=" << this->cgra->max_hops;

			MapHeader << ",BB = " << node->BB;
			MapHeader << ",mutexPathEn = " << this->enableMutexPaths;
			MapHeader << ",Iter = " << i;
			MapHeader << "\n";

			std::cout << MapHeader.str();
			mappingLog << MapHeader.str();

			bool isEstRouteSucc = false;

			//fill the routing information
			if (estimatedRouteInfo.find(node) == estimatedRouteInfo.end())
			{
				//the routes are not estimated.
				std::priority_queue<dest_with_cost> estimatedRoutes;
				DFGNode *failedNode;
				isEstRouteSucc = estimateRouting(node, estimatedRoutes, &failedNode);

				if (!isEstRouteSucc)
				{
					printMappingLog();
					printMappingLog2();
					if (enableBackTracking)
					{
						if (backTrackCredits == 0 || failedNode == NULL)
						{
							std::cout << "route estimation failed...\n";
							std::cout << "Map Failed!.\n";
							mappingLog << "route estimation failed...\n";
							mappingLog << "Map Failed!.\n";

							mappingLog.close();
							mappingLog2.close();
							mappingLog4.close();
							return false;
						}
						backTrackCredits--;

						//					DFGNode* prevNode = mappedNodes.top();
						//					mappedNodes.pop();
						//					unmappedNodes.push(node);
						//					unmappedNodes.push(prevNode);
						//					prevNode->clear(this->dfg);
						//					std::cout << "route estimation failed...\n";
						//					mappingLog << "route estimation failed...\n";
						//					continue;

						DFGNode *prevNode = mappedNodes.top();
						mappedNodes.pop();
						unmappedNodes.push(node);
						unmappedNodes.push(prevNode);

						prevNode->clear(this->dfg);
						estimatedRouteInfo.erase(node);

						//										assert(failedNode!=NULL);
						//										unmappedNodes.push(node);
						//										removeFailedNode(mappedNodes,unmappedNodes,failedNode);
						//										failedNode->blacklistDest.insert(failedNode->rootDP);
						//										(failedNode)->clear(this->dfg);
						//										estimatedRouteInfo.erase(node);
						//										estimatedRouteInfo.erase(failedNode);

						continue;
					}
					else
					{
						while (!mappedNodes.empty())
						{
							DFGNode *prevNode = mappedNodes.top();
							mappedNodes.pop();
							prevNode->clear(this->dfg);
						}
						std::cout << "Map Failed!.\n";
						mappingLog << "Map Failed!.\n";
						mappingLog.close();
						mappingLog2.close();
						mappingLog4.close();
						return false;
					}
				}
				estimatedRouteInfo[node] = estimatedRoutes;
			}

			bool isRouteSucc = false;
			DFGNode *failedNode = NULL;

			std::cout << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			if (!estimatedRouteInfo[node].empty())
			{
				isRouteSucc = Route(node, estimatedRouteInfo[node], &failedNode);
				if (!isRouteSucc)
					std::cout << "BLAAAAAAAAAAA!\n";
			}
			else
			{
				if (mappedNodes.empty())
				{
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					mappingLog.close();
					mappingLog2.close();
					mappingLog4.close();
					return false;
				}
			}

			if (!isRouteSucc)
			{
				this->printMappingLog();
				this->printMappingLog2();
				if (mappedNodes.empty())
				{
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					mappingLog.close();
					mappingLog2.close();
					mappingLog4.close();
					return false;
				}

				if (enableBackTracking)
				{
					if (backTrackCredits == 0)
					{
						mappingLog << "Map Failed!.\n";
						std::cout << "Map Failed!.\n";
						mappingLog.close();
						mappingLog2.close();
						mappingLog4.close();
						return false;
					}
					//					assert(failedNode!=NULL);
					backTrackCredits--;

					DFGNode *prevNode = mappedNodes.top();
					mappedNodes.pop();
					unmappedNodes.push(node);
					unmappedNodes.push(prevNode);

					prevNode->clear(this->dfg);
					estimatedRouteInfo.erase(node);

					//					unmappedNodes.push(node);
					//					removeFailedNode(mappedNodes,unmappedNodes,failedNode);
					//					failedNode->blacklistDest.insert(failedNode->rootDP);
					//					(failedNode)->clear(this->dfg);
					//					estimatedRouteInfo.erase(node);
					//					estimatedRouteInfo.erase(failedNode);
					continue;
				}
				else
				{
					while (!mappedNodes.empty())
					{
						DFGNode *prevNode = mappedNodes.top();
						mappedNodes.pop();
						prevNode->clear(this->dfg);
					}
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					mappingLog.close();
					mappingLog2.close();
					mappingLog4.close();
					return false;
				}
			}

			//		this->printMappingLog();
			//		this->printMappingLog2();
			backTrackCredits = std::min(this->backTrackLimit, backTrackCredits + 1);
			mappedNodes.push(node);
		}
		mapSuccess = updateCongestionCosts(i);
		if (mapSuccess)
		{
			break;
		}
		clearCurrMapping();
		estimatedRouteInfo.clear();
		mappingLog.close();
		mappingLog2.close();
		mappingLog4.close();
	}

	//	congestionInfoFile.close();

	if (mapSuccess)
	{
		mappingLog << "Map Success!.\n";
		mappingLog2 << "Map Success!.\n";
		this->printMappingLog();
		this->printMappingLog2();

		// by Yujie
		// cgra->PrintMappedJSON(fNameLog1 + cgra->getCGRAName() + "mapping.json");
		cgra->PrintMappingForPillars(fNameLog1 + cgra->getCGRAName() + "mapping_i.txt", fNameLog1 + cgra->getCGRAName() + "mapping_r.txt");

		std::cout << "Map Success!.\n";
		mappingLog.close();
		mappingLog2.close();

		std::cout << "Checking conflict compatibility!\n";
		checkConflictedPortCompatibility();

		if (this->cgra->peType == "STDNOC_4REGF_1P")
		{
			checkRegALUConflicts();
		}
		return true;
	}
	else
	{
		while (!mappedNodes.empty())
		{
			DFGNode *prevNode = mappedNodes.top();
			mappedNodes.pop();
			prevNode->clear(this->dfg);
		}
		mappingLog << "Map Failed!.\n";
		std::cout << "Map Failed!.\n";
		mappingLog.close();
		mappingLog2.close();
		return false;
	}
}

void CGRAXMLCompile::PathFinderMapper::assignPath(DFGNode *src, DFGNode *dest,
												  std::vector<LatPort> path)
{

	LOG(ROUTE) << "assigning path from:" << src->idx << " to:" << dest->idx << "\n";

	int srcPortCount = 0;

	int prevLat = -1;
	LatPort prevPort;
	for (LatPort p : path)
	{

		if (prevLat != -1)
		{
			if (p.first - prevLat > 1)
			{
				std::cout << prevPort.second->getFullName() << ",Lat = " << prevPort.first << "\n";
				std::cout << p.second->getFullName() << ",Lat = " << p.first << "\n";
			}
			assert(p.first - prevLat <= 1);
		}

		prevLat = p.first;
		prevPort = p;

		
		if (p.second->getNode() == src)
		{
			srcPortCount++;
			LOG(ROUTE)<<"src port";
			// continue;
		}
		

		
		p.second->setNode(src, p.first, dest->idx, this);

#ifdef FLEX
		if(src->idx != 0)
			reserveVectorPorts(p.second,src,p.first, dest->idx);
#endif
		congestedPorts[p.second].insert(src);
		p.second->increaseConflictedUse(src, this);

		if (std::find(src->routingPorts.begin(), src->routingPorts.end(), std::make_pair(p.second, dest->idx)) == src->routingPorts.end())
		{
			if (std::find(src->routingPorts.begin(), src->routingPorts.end(), std::make_pair(p.second, src->idx)) == src->routingPorts.end())
			{
				src->routingPorts.push_back(std::make_pair(p.second, dest->idx));
				LOG(ROUTE)<<"push back route:" << p.second->getFullName() ;
			}
			else
			{
				LOG(ROUTE) << p.second->getFullName() ;
				assert(p.second->getName().compare("T") == 0);
			}
		}
		//		src->routingPortDestMap[p]=dest->idx;
	}
	LOG(ROUTE) << "srcPortCount = " << srcPortCount << "\n";
}

bool CGRAXMLCompile::PathFinderMapper::updateCongestionCosts(int iter)
{
	bool noCongestion = true;

	std::set<int> conflictedTimeSteps;

	congestionInfoFile << "**********************************\n";
	congestionInfoFile << "II = " << this->cgra->get_t_max() << ",iter = " << iter << "\n";
	congestionInfoFile << "**********************************\n";

	for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts)
	{
		Port *p = pair.first;
		if (pair.second.size() > 1)
		{
			for (DFGNode *node1 : pair.second)
			{
				for (DFGNode *node2 : pair.second)
				{
					if (node1 == node2)
					{
						continue;
					}
					if (this->dfg->isMutexNodes(node1, node2, p))
						continue;
					std::cout << "CONGESTION:" << p->getFullName();
					congestionInfoFile << "CONGESTION:" << p->getFullName();
					for (DFGNode *node : pair.second)
					{
						std::cout << "," << node->idx << "|BB=" << node->BB;
						congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
					}
					std::cout << "\n";
					congestionInfoFile << "\n";
					p->increastCongCost();
					noCongestion = false;
					conflictedTimeSteps.insert(p->getMod()->getPE()->T);
					//					break;
				}
				if (!noCongestion)
				{
					//					break;
				}
			}
		}
		if (p->getHistoryCost() > 0)
		{
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
			congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
		}
	}

	bool noConflicts = true;
	for (std::pair<Port *, std::set<DFGNode *>> pair : conflictedPorts)
	{
		Port *p = pair.first;

		if (p->getNode() != NULL)
		{
			for (DFGNode *node : pair.second)
			{
				//				if(node == p->getNode()){
				//					bool isRDP = p->getName().find("RDP") != std::string::npos;
				//					bool isINT = p->getName().find("INT") != std::string::npos;
				//					bool isT = p->getName().find("_T") != std::string::npos;
				//					if((isRDP&isINT) || isT) continue;
				//				}
				noConflicts = false;
			}

			if (noConflicts)
				continue;

			std::cout << "CONFLICT :" << p->getFullName();
			congestionInfoFile << "CONFLICT :" << p->getFullName();
			for (DFGNode *node : pair.second)
			{
				std::cout << "," << node->idx << "|BB=" << node->BB;
				congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
			}
			std::cout << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			std::cout << "\n";

			congestionInfoFile << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			congestionInfoFile << "\n";

			for (int i = 0; i < pair.second.size(); ++i)
			{
				p->increastCongCost();
			}
			conflictedTimeSteps.insert(p->getMod()->getPE()->T);
		}

		if (p->getHistoryCost() > 0)
		{
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
			congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
		}
	}

	if (this->upperboundII > conflictedTimeSteps.size() + this->cgra->get_t_max())
	{
		this->upperboundII = conflictedTimeSteps.size() + this->cgra->get_t_max();
		this->upperboundIter = iter;
		this->upperboundFoundBy = this->cgra->get_t_max();
		std::cout << "****************************************\n";
		std::cout << "Upperbound II = " << this->upperboundII << "\n";
		std::cout << "On iter = " << iter << "\n";
		std::cout << "****************************************\n";

		congestionInfoFile << "****************************************\n";
		congestionInfoFile << "Upperbound II = " << this->upperboundII << "\n";
		congestionInfoFile << "On iter = " << iter << "\n";
		congestionInfoFile << "****************************************\n";
	}

	congestionInfoFile << std::endl;

	congestedPorts.clear();
	conflictedPorts.clear();
	conflictedTimeStepMap.clear();
	if (noCongestion)
		std::cout << "noCongestion!\n";
	if (noConflicts)
		std::cout << "noConflicts!\n";

	if (noCongestion)
		congestionInfoFile << "noCongestion!\n";
	if (noConflicts)
		congestionInfoFile << "noConflicts!\n";

	return noCongestion & noConflicts;
}

bool CGRAXMLCompile::PathFinderMapper::clearCurrMapping()
{
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "CLEARING :: node=" << node->idx << ",destDP=" << node->rootDP->getName() << ",destPE=" << node->rootDP->getPE()->getName() << "\n";
		node->clear(this->dfg);
	}

	std::stack<Module *> searchStack;
	searchStack.push(this->cgra);

	while (!searchStack.empty())
	{
		Module *top = searchStack.top();
		searchStack.pop();
		for (Port *p : top->inputPorts)
		{
			assert(p->getNode() == NULL);
		}
		for (Port *p : top->internalPorts)
		{
			assert(p->getNode() == NULL);
		}
		for (Port *p : top->outputPorts)
		{
			assert(p->getNode() == NULL);
		}
		for (Module *submod : top->subModules)
		{
			searchStack.push(submod);
		}
	}
	return true;
}

bool CGRAXMLCompile::PathFinderMapper::checkConflictedPortCompatibility()
{

	std::stack<Module *> searchStack;
	searchStack.push(this->cgra);

	while (!searchStack.empty())
	{
		Module *top = searchStack.top();
		searchStack.pop();
		for (Port *p : top->inputPorts)
		{
			if (p->getNode() != NULL)
			{
				for (Port *cp : top->getConflictPorts(p))
				{
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if (cp->getNode() != NULL)
					{
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode() == NULL);
				}
			}
		}
		for (Port *p : top->internalPorts)
		{
			if (p->getNode() != NULL)
			{
				for (Port *cp : top->getConflictPorts(p))
				{
					//					if(cp->)
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if (cp->getNode() != NULL)
					{
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode() == NULL);
				}
			}
		}
		for (Port *p : top->outputPorts)
		{
			if (p->getNode() != NULL)
			{
				for (Port *cp : top->getConflictPorts(p))
				{
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if (cp->getNode() != NULL)
					{
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode() == NULL);
				}
			}
		}
		for (Module *submod : top->subModules)
		{
			searchStack.push(submod);
		}
	}
}

bool CGRAXMLCompile::PathFinderMapper::checkRegALUConflicts()
{
	for (int t = 0; t < this->cgra->get_t_max(); ++t)
	{
		int timeslice_count = 0;
		vector<PE *> PEList = this->cgra->getSpatialPEList(t);
		// for (int y = 0; y < this->cgra->get_y_max(); ++y)
		// {
		// 	for (int x = 0; x < this->cgra->get_x_max(); ++x)
		// 	{
		for (PE *currPE : PEList)
		{

			// PE *currPE = this->cgra->getPE(t, y, x);
			int usage = 0;

			for (Module *submod_fu : currPE->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(submod_fu))
				{
					for (Module *submod_dp : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submod_dp))
						{
							if (dp->getMappedNode() != NULL)
							{
								std::cout << dp->getFullName() << ":" << dp->getMappedNode()->idx << ",";
								usage++;
								break;
							}
						}
					}
				}
			}

			for (RegFile *RF : currPE->allRegs)
			{
				for (int i = 0; i < RF->get_nWRPs(); ++i)
				{
					std::string wrpName = "WRP" + std::to_string(i);
					Port *wrp = RF->getInPort(wrpName);
					if (wrp->getNode() != NULL)
					{
						std::cout << wrp->getFullName() << ":" << wrp->getNode()->idx << ",";
						usage++;
					}
				}

				for (int i = 0; i < RF->get_nRDPs(); ++i)
				{
					std::string rdpName = "RDP" + std::to_string(i);
					Port *rdp = RF->getOutPort(rdpName);
					if (rdp->getNode() != NULL)
					{
						std::cout << rdp->getFullName() << ":" << rdp->getNode()->idx << ",";
						usage++;
					}
				}
			}

			if (timeslice_count <= usage - 1)
			{
				timeslice_count = usage - 1;
			}

			std::cout << "\n";
		}
		std::cout << "t=" << t << ","
			  << "timeslice=" << timeslice_count << "\n";
	}
}

bool CGRAXMLCompile::PathFinderMapper::checkDPFree(DataPath *dp, DFGNode *node, int &penalty)
{
	PE *currPE = dp->getPE();
	FU *currFU = dp->getFU();

	int numberFUs = 0;
	int numberUsedFUs = 0;
	int numberConstants = 0;
	bool memfu_found = false;
	bool memop_found = false;
	for (Module *submod_fu : currPE->subModules)
	{
		if (FU *fu = dynamic_cast<FU *>(submod_fu))
		{
			int dp_used = 0;
			if (!memfu_found)
				memfu_found = fu->isMEMFU();
			for (Module *submod_dp : fu->subModules)
			{
				if (DataPath *dp = dynamic_cast<DataPath *>(submod_dp))
				{
					if (dp->getMappedNode() != NULL)
					{
						dp_used = 1;
						if (!memop_found)
							memop_found = dp->getMappedNode()->isMemOp();
						if (dp->getMappedNode()->hasConst)
						{
							numberConstants++;
						}
					}
				}
			}
			numberUsedFUs += dp_used;
			numberFUs += 1;
		}
	}

	//increment for the current node
	numberUsedFUs++;
	if (node->hasConst)
	{
		numberConstants++;
	}

	assert(this->dfg->unmappedMemOps == this->dfg->unmappedMemOpSet.size());
	assert(this->cgra->freeMemNodes == this->cgra->freeMemNodeSet.size());

	penalty = 0;
	if (memfu_found)
	{
		int memnode_const_count = 0;
		for (DFGNode *memnode : this->dfg->unmappedMemOpSet)
		{
			if (memnode->hasConst)
			{
				memnode_const_count++;
			}
		}

		int freeMemPEs_const = 0;
		for (DataPath *memdp : this->cgra->freeMemNodeSet)
		{
			int memPEConstants = 0;
			int memUsedFUs = 0;
			PE *memPE = memdp->getPE();
			for (Module *submod_fu : memPE->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(submod_fu))
				{
					int dp_used = 0;
					for (Module *submod_dp : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submod_dp))
						{
							if (dp->getMappedNode() != NULL)
							{
								dp_used = 1;
								if (!memop_found)
									memop_found = dp->getMappedNode()->isMemOp();
								if (dp->getMappedNode()->hasConst)
								{
									memPEConstants++;
								}
							}
						}
					}
					memUsedFUs += dp_used;
				}
			}
			if (memUsedFUs + memPEConstants <= 1)
			{
				freeMemPEs_const++;
			}
		}

		if ((!node->isMemOp()) && (!memop_found))
		{
			double penalty_ratio_dbl = (double)memnode_const_count / (double)freeMemPEs_const;
			double penalty_dbl = penalty_ratio_dbl * (double)MRC;
			penalty = (int)penalty_dbl;
		}
	}

	//with current node it should be less than or equal to number of FUs
	if (numberConstants + numberUsedFUs <= numberFUs || numberFUs == 1)
	{
		if (dp->getMappedNode() == NULL)
		{
			return true;
		}
	}
	return false;
}

bool CGRAXMLCompile::PathFinderMapper::updateConflictedTimeSteps(int timeStep,
																 int conflicts)
{

	int presentConflicts = conflictedTimeStepMap[timeStep];
	if (conflicts > presentConflicts)
	{
		conflictedTimeStepMap[timeStep] = conflicts;
		return true;
	}
	return false;
}

int CGRAXMLCompile::PathFinderMapper::getTimeStepConflicts(int timeStep)
{
	return conflictedTimeStepMap[timeStep];
}

void CGRAXMLCompile::PathFinderMapper::sortBackEdgePriorityASAP()
{
	sortedNodeList.clear();

	std::stringstream output_ss;
	printf("Here0\n");
	struct BEDist
	{
		DFGNode *parent;
		DFGNode *child;
		int dist;
		BEDist(DFGNode *parent, DFGNode *child, int dist) : parent(parent), child(child), dist(dist) {}
		bool operator<(const BEDist &other) const
		{
			if (dist == other.dist)
			{
				return true;
			}
			return dist > other.dist;
		}
		//		bool operator==(const BEDist& other) const{
		//			return parent==other.parent & child==other.child;
		//		}
	};

	std::set<BEDist> backedges;
	//printf("Here1\n");
	for (DFGNode &node : dfg->nodeList)
	{

		if (node.idx == 97)
		{
			output_ss << "node_idx:97,node_ASAP:" << node.ASAP << "\n";
		}
		for (DFGNode *child : node.children)
		{

			if (node.idx == 97)
			{
				output_ss << "child_idx:" << child->idx << "child_ASAP:" << child->ASAP << "\n";
			}

			if (child->ASAP <= node.ASAP)
			{
				std::cout << "inserting for : node=" << node.idx << ",child:" << child->idx << "\n";
				backedges.insert(BEDist(&node, child, node.ASAP - child->ASAP));
				printf("Backedge on node->%d\n",node.idx);
			}
		}
	}
	//printf("Here2\n");
	//populate reccycles
	output_ss << "Populate Rec Cycles!\n";
	RecCycles.clear();
	//exit(0);
	for (BEDist be : backedges)
	{
		//		std::set<DFGNode*> backedgePath;
		std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be.parent);

		output_ss << "REC_CYCLE :: BE_Parent = " << be.parent->idx << "\n";
		output_ss << "REC_CYCLE :: BE_Child = " << be.child->idx << "\n";
		output_ss<< "REC_CYCLE :: BE_Parent's ancesotry : \n";
		for (DFGNode *n : backedgePathVec)
		{
			if (RecCycles[BackEdge(be.parent, be.child)].find(n) == RecCycles[BackEdge(be.parent, be.child)].end())
			{
				output_ss << n->idx << ",";
			}
			RecCycles[BackEdge(be.parent, be.child)].insert(n);
		}
		output_ss << "REC_CYCLE :: Done!\n";
		//= dfg->getAncestoryASAP(be.parent);
		//		if(dfg->getAncestoryASAPUntil(be.parent,be.child,backedgePath)){
		//			backedgePath.insert(be.parent);
		//			RecCycles[BackEdge(be.parent,be.child)]=backedgePath;
		//		}
	}
	//printf("Here3\n");
	RecCyclesLS.clear();
	for (DFGNode &node : dfg->nodeList)
	{
		for (DFGNode *recParent : node.recParents)
		{
			BEDist be_temp(&node, recParent, node.ASAP - recParent->ASAP);

			std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be_temp.parent);

			output_ss << "REC_CYCLELS :: BE_Parent = " << be_temp.parent->idx << "\n";
			output_ss << "REC_CYCLELS :: BE_Child = " << be_temp.child->idx << "\n";
			output_ss << "REC_CYCLELS :: BE_Parent's ancesotry : \n";
			for (DFGNode *n : backedgePathVec)
			{
				if (n == be_temp.parent)
					continue;
				if (RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].find(n) == RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].end())
				{
					output_ss << n->idx << ",";
				}
				RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].insert(n);
			}
			output_ss << "REC_CYCLELS :: Done!\n";

			backedges.insert(be_temp);
		}
	}
	//printf("Here4\n");
	std::map<DFGNode *, std::vector<DFGNode *>> beparentAncestors;
	std::map<DFGNode *, std::vector<DFGNode *>> bechildAncestors;
	//	std::map<DFGNode*,std::vector<DFGNode*>> bechildAncestors;
	std::map<std::pair<DFGNode *, DFGNode *>, bool> trueBackedges;

	for (BEDist be : backedges)
	{
		printf("Inside backedge loop\n");
		output_ss << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		output_ss << "BE CHILD = " << be.child->idx << "\n";

		output_ss << "Ancestory : "
				  << "\n";
		beparentAncestors[be.parent] = dfg->getAncestoryASAP(be.parent);
		bechildAncestors[be.child] = dfg->getAncestoryASAP(be.child);
		output_ss << "\n";

		if (std::find(beparentAncestors[be.parent].begin(),
					  beparentAncestors[be.parent].end(),
					  be.child) == beparentAncestors[be.parent].end())
		{
			output_ss << "BE CHILD does not belong BE Parent's Ancestory\n";

			//Hack to force all backedges to be true backedges
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
		}
		else
		{
			//change this be.parent if PHI nodes are not removed
			output_ss << "RecPHI inserted : " << be.child->idx << "\n";
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
			//			RecPHIs.insert(be.child);
		}

		//		bechildAncestors[be.child]=dfg->getAncestory(be.child);
	}
	//printf("Here5\n");
	//	{ //true backedges children are placed high priority :MERGED
	//		std::vector<DFGNode*> mergedAncestoriesChild;
	//		std::map<DFGNode*,DFGNode*> mergedKeysChild;
	//
	//		for(BEDist be : backedges){
	//			if(trueBackedges[std::make_pair(be.parent,be.child)] == true){
	//				mergedAncestoriesChild = dfg->mergeAncestoryASAP(mergedAncestoriesChild,bechildAncestors[be.child],RecCycles);
	//			}
	//		}
	//
	//		for(DFGNode* ancestorNode : mergedAncestoriesChild){
	//			if(std::find(sortedNodeList.begin(),sortedNodeList.end(),ancestorNode) == sortedNodeList.end()){
	//				sortedNodeList.push_back(ancestorNode);
	//			}
	//		}
	//
	//	}

	std::map<DFGNode *, std::set<DFGNode *>> superiorChildren;

	//	std::vector<DFGNode*> mergedAncestory;
	std::map<DFGNode *, std::vector<DFGNode *>> mergedAncestories;
	mergedAncestories.clear();
	std::map<DFGNode *, DFGNode *> mergedKeys;
	for (BEDist be : backedges)
	{
		//		write a logic to merge ancestories where if one be's child is present in some other be's parent's ancesotory'
		bool merged = false;

		//		if(trueBackedges[std::make_pair(be.parent,be.child)] == true){
		//			// if true backede place the child first so that the parent's path will
		//			// be adjusted accordingly
		////			for(DFGNode* ancestorNode : bechildAncestors[be.child]){
		////				if(std::find(sortedNodeList.begin(),sortedNodeList.end(),ancestorNode) == sortedNodeList.end()){
		////					sortedNodeList.push_back(ancestorNode);
		////				}
		////			}
		//			superiorChildren[be.parent].insert(be.child);
		//		}

		for (std::pair<DFGNode *, std::vector<DFGNode *>> pair : mergedAncestories)
		{
			DFGNode *key = pair.first;
			//			if(trueBackedges[std::make_pair(be.parent,be.child)] == false) continue;
			if (std::find(mergedAncestories[key].begin(), mergedAncestories[key].end(), be.child) != mergedAncestories[key].end())
			{

				if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
				{
					superiorChildren[key].insert(be.child);
				}

				output_ss << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedAncestories[key] = dfg->mergeAncestoryASAP(mergedAncestories[key], beparentAncestors[be.parent], RecCycles);
				merged = true;
				output_ss << "Merging Done :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedKeys[be.parent] = key;
				//				break;
			}
		}
		if (!merged)
		{
			mergedAncestories[be.parent] = dfg->getAncestoryASAP(be.parent);
			mergedKeys[be.parent] = be.parent;

			if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
			{
				superiorChildren[be.parent].insert(be.child);
			}
		}
	}
	//printf("Here6\n");
	for (BEDist be : backedges)
	{
		std::vector<DFGNode *> mergedSuperiorChildren;
		for (DFGNode *sChild : superiorChildren[mergedKeys[be.parent]])
		{
			mergedSuperiorChildren = dfg->mergeAncestoryASAP(mergedSuperiorChildren, bechildAncestors[sChild], RecCycles);
		}

		//		for(DFGNode* sChild : superiorChildren[mergedKeys[be.parent]]){
		//			for(DFGNode* ancestorNode : bechildAncestors[sChild]){
		for (DFGNode *ancestorNode : mergedSuperiorChildren)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		//		}

		for (DFGNode *ancestorNode : mergedAncestories[mergedKeys[be.parent]])
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
	}
	//printf("Here7\n");
	for (BEDist be : backedges)
	{
		assert(mergedKeys.find(be.parent) != mergedKeys.end());
		std::vector<DFGNode *> ancestoryNodes = mergedAncestories[mergedKeys[be.parent]];
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		output_ss << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.parent) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.parent);
		}

		ancestoryNodes = dfg->getAncestoryASAP(be.child);
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		output_ss << "BE CHILD = " << be.child->idx << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.child) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.child);
		}
	}
	//printf("Here8\n");
	std::map<int, std::vector<DFGNode *>> asapLevelNodeList;
	for (DFGNode &node : dfg->nodeList)
	{
		asapLevelNodeList[node.ASAP].push_back(&node);
	}

	int maxASAPlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : asapLevelNodeList)
	{
		if (pair.first > maxASAPlevel)
		{
			maxASAPlevel = pair.first;
		}
	}

	for (int i = 0; i <= maxASAPlevel; ++i)
	{
		for (DFGNode *node : asapLevelNodeList[i])
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), node) == sortedNodeList.end())
			{
				sortedNodeList.push_back(node);
				//printf("Node->%d\n",node->idx);
			}
		}
	}
	//printf("Here9\n");
	output_ss<< "***********SORTED LIST*******************\n";
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "Node=" << node->idx << ",ASAP=" << node->ASAP <<"\n";
	}
	//	assert(false);
	//printf("Here91\n");
	std::reverse(sortedNodeList.begin(), sortedNodeList.end());

	int unmappedMemNodeCount = 0;
	for (DFGNode *node : this->sortedNodeList)
	{
		//printf("Node->%d\n",node->idx);
		if (node->isMemOp())
		{
			if (node->rootDP == NULL)
			{
				unmappedMemNodeCount++;
				dfg->unmappedMemOpSet.insert(node);
			}
		}
	}
	dfg->unmappedMemOps = unmappedMemNodeCount;
	//printf("Here10\n");
	LOG(DFG)<<output_ss.str();
}


void CGRAXMLCompile::PathFinderMapper::sortGivenOrder(std::vector<int> nodeOrder)
{

	sortedNodeList.clear();

	for(int index : nodeOrder)
	{
		for (DFGNode &node : dfg->nodeList)
		{
			if(node.idx == index)
			{
				sortedNodeList.push_back(&node);
				break;
			}
		}
	}
	std::cout << "***********SORTED LIST*******************\n";
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "Node=" << node->idx << ",ALAP=" << node->ALAP ;
	}
	//	assert(false);

	std::reverse(sortedNodeList.begin(), sortedNodeList.end());

	int unmappedMemNodeCount = 0;
	for (DFGNode *node : this->sortedNodeList)
	{
		if (node->isMemOp())
		{
			if (node->rootDP == NULL)
			{
				unmappedMemNodeCount++;
				dfg->unmappedMemOpSet.insert(node);
			}
		}
	}
	dfg->unmappedMemOps = unmappedMemNodeCount;

}
void CGRAXMLCompile::PathFinderMapper::sortBackEdgePriorityALAP()
{

	sortedNodeList.clear();

	struct BEDist
	{
		DFGNode *parent;
		DFGNode *child;
		int dist;
		BEDist(DFGNode *parent, DFGNode *child, int dist) : parent(parent), child(child), dist(dist) {}
		bool operator<(const BEDist &other) const
		{
			if (dist == other.dist)
			{
				return true;
			}
			return dist > other.dist;
		}
		//		bool operator==(const BEDist& other) const{
		//			return parent==other.parent & child==other.child;
		//		}
	};

	std::set<BEDist> backedges;

	for (DFGNode &node : dfg->nodeList)
	{

		if (node.idx == 97)
		{
			std::cout << "node_idx:97,node_ALAP:" << node.ALAP << "\n";
		}
		for (DFGNode *child : node.children)
		{

			if (node.idx == 97)
			{
				std::cout << "child_idx:" << child->idx << "child_ALAP:" << child->ALAP << "\n";
			}

			if (child->ALAP <= node.ALAP)
			{
				std::cout << "inserting for : node=" << node.idx << ",child:" << child->idx << "\n";
				backedges.insert(BEDist(&node, child, node.ALAP - child->ALAP));
			}
		}
	}

	for (DFGNode &node : dfg->nodeList)
	{
		for (DFGNode *recParent : node.recParents)
		{
			backedges.insert(BEDist(&node, recParent, node.ALAP - recParent->ALAP));
		}
	}

	std::map<DFGNode *, std::vector<DFGNode *>> beparentAncestors;
	//	std::map<DFGNode*,std::vector<DFGNode*>> bechildAncestors;

	for (BEDist be : backedges)
	{
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		std::cout << "BE CHILD = " << be.child->idx << "\n";

		std::cout << "Ancestory : "
				  << "\n";
		beparentAncestors[be.parent] = dfg->getAncestoryALAP(be.parent);
		std::cout << "\n";

		if (std::find(beparentAncestors[be.parent].begin(),
					  beparentAncestors[be.parent].end(),
					  be.child) == beparentAncestors[be.parent].end())
		{
			std::cout << "BE CHILD does not belong BE Parent's Ancestory\n";
		}
		else
		{
			//change this be.parent if PHI nodes are not removed
			//			RecPHIs.insert(be.parent);
		}

		//		bechildAncestors[be.child]=dfg->getAncestory(be.child);
	}

	//	std::vector<DFGNode*> mergedAncestory;
	std::map<DFGNode *, std::vector<DFGNode *>> mergedAncestories;
	mergedAncestories.clear();
	std::map<DFGNode *, DFGNode *> mergedKeys;
	for (BEDist be : backedges)
	{
		//		write a logic to merge ancestories where if one be's child is present in some other be's parent's ancesotory'
		bool merged = false;
		for (std::pair<DFGNode *, std::vector<DFGNode *>> pair : mergedAncestories)
		{
			DFGNode *key = pair.first;
			if (std::find(mergedAncestories[key].begin(), mergedAncestories[key].end(), be.child) != mergedAncestories[key].end())
			{
				std::cout << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedAncestories[key] = dfg->mergeAncestoryALAP(mergedAncestories[key], beparentAncestors[be.parent]);
				merged = true;
				mergedKeys[be.parent] = key;
			}
		}
		if (!merged)
		{
			mergedAncestories[be.parent] = beparentAncestors[be.parent];
			mergedKeys[be.parent] = be.parent;
		}
	}

	for (BEDist be : backedges)
	{
		assert(mergedKeys.find(be.parent) != mergedKeys.end());
		std::vector<DFGNode *> ancestoryNodes = mergedAncestories[mergedKeys[be.parent]];
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.parent) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.parent);
		}

		ancestoryNodes = dfg->getAncestoryALAP(be.child);
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE CHILD = " << be.child->idx << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.child) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.child);
		}
	}

	std::map<int, std::vector<DFGNode *>> alapLevelNodeList;
	for (DFGNode &node : dfg->nodeList)
	{
		alapLevelNodeList[node.ALAP].push_back(&node);
	}

	int maxALAPlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : alapLevelNodeList)
	{
		if (pair.first > maxALAPlevel)
		{
			maxALAPlevel = pair.first;
		}
	}

	for (int i = 0; i <= maxALAPlevel; ++i)
	{
		for (DFGNode *node : alapLevelNodeList[i])
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), node) == sortedNodeList.end())
			{
				sortedNodeList.push_back(node);
			}
		}
	}

	LOG(ROUTE) << "***********SORTED LIST*******************\n";
	for (DFGNode *node : sortedNodeList)
	{
		LOG(ROUTE) << "Node=" << node->idx << ",ALAP=" << node->ALAP ;
	}
	//	assert(false);

	std::reverse(sortedNodeList.begin(), sortedNodeList.end());

	int unmappedMemNodeCount = 0;
	for (DFGNode *node : this->sortedNodeList)
	{
		if (node->isMemOp())
		{
			if (node->rootDP == NULL)
			{
				unmappedMemNodeCount++;
				dfg->unmappedMemOpSet.insert(node);
			}
		}
	}
	dfg->unmappedMemOps = unmappedMemNodeCount;
}

int CGRAXMLCompile::PathFinderMapper::getlatMinStartsPHI(const DFGNode *currNode,
														 const std::map<DFGNode *, std::vector<Port *>> &possibleStarts)
{

	int min;
	std::map<DFGNode *, int> minLat;

	//get minimal latency for each start
	for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
	{
		int latm = 100000000;

		for (Port *p : pair.second)
		{
			if (p->getLat() < latm)
			{
				latm = p->getLat();
			}
		}
		assert(latm != 100000000);
		minLat[pair.first] = latm;
	}

	int max = 0;
	//get the max latency among  all the candidates
	for (std::pair<DFGNode *, int> pair : minLat)
	{
		if (max < pair.second)
		{
			max = pair.second;
		}
		LOG(ROUTE) << "getlatMinStartsPHI :: minLat = " << max << "\n";
	}

	//	std::map<std::string,int> oplatencyMap;
	//	cgra->PEArr[0][0][0]->getMEMIns(oplatencyMap);

	std::unordered_map<std::string, int> oplatencyMap = cgra->getGlobalOPMinLatencyMap();

	int recphi_lat = 0;
	if (RecPHIs.find((DFGNode *)currNode) != RecPHIs.end())
	{
		std::cout << "RecPHI found!!!! : " << currNode->idx << "\n";

		for (DFGNode *child : currNode->children)
		{
			for (DFGNode *childparent : child->parents)
			{
				if (childparent == currNode)
					continue;

				int oplatency = oplatencyMap[childparent->op];
				for (DFGNode *parentchildparent : childparent->parents)
				{
					if (parentchildparent->rootDP != NULL)
					{
						int newlat = parentchildparent->rootDP->getLat() + oplatency;
						if (newlat > recphi_lat)
						{
							std::cout << "RecPhi Lat = " << newlat << "\n";
							recphi_lat = newlat;
						}
					}
				}
			}
		}
	}

	if (recphi_lat > max)
		max = recphi_lat;

	//	assert(max!=-1);
	return max;
}

std::set<CGRAXMLCompile::DFGNode *> CGRAXMLCompile::PathFinderMapper::getElders(DFGNode *node)
{
	//	std::set<DFGNode*> res;
	//
	//	std::map<int,DFGNode*> descendents;
	//	std::queue<std::set<DFGNode*>> q;
	//	std::set<DFGNode*> initLevel; initLevel.insert(node);
	//	q.push(initLevel);
	//	int level_ctr=0;
	//
	//	while(!q.empty()){
	//		std::set<DFGNode*> level = q.front(); q.pop();
	//		level_ctr++;
	//		for(DFGNode* levelnode : level){
	//			for(DFGNode* child : levelnode->children){
	//				if(levelnode->childNextIter[child] == 1) continue; //ignore backedges;
	//				if(descendents.find(child) == descendents.end()){
	//					descendents.insert(child);
	//					q.push(child);
	//				}
	//			}
	//
	//
	//		}
	//
	//
	//		for(DFGNode* child : top->children){
	//			if(top->childNextIter[child] == 1) continue; //ignore backedges
	//			if(descendents.find(child) == descendents.end()){
	//				descendents.insert(child);
	//				q.push(child);
	//			}
	//		}
	//	}
	//
	//
	//
	//	return res;
}

// I guess this is to satisfy recurrent data dependency.
int CGRAXMLCompile::PathFinderMapper::getMaxLatencyBE(DFGNode *node, std::map<DataPath *, beParentInfo> &beParentDests, int &downSteamOps)
{

	std::set<BackEdge> setBackEdges;
	//std::cout << "getMaxLatencyBE started!\n";

	//std::cout << "NODE ASAP = " << node->ASAP << "\n";

	for (std::pair<BackEdge, std::set<DFGNode *>> pair : RecCycles)
	{
		BackEdge be = pair.first;
		std::set<DFGNode *> rec_nodes = pair.second;
		if (rec_nodes.find(node) != rec_nodes.end())
		{
			if (be.second->rootDP != NULL)
			{
				std::stringstream ss;
				ss<< "RecSet(" << be.first->idx << "," << be.second->idx << ")"
						  << " : ";
				for (DFGNode *n : rec_nodes)
				{
					ss << n->idx << ",";
				}
				ss << "\n";
				LOG(ROUTE)<<ss.str();
				setBackEdges.insert(pair.first);
			}
		}
	}

	for (std::pair<BackEdge, std::set<DFGNode *>> pair : RecCyclesLS)
	{
		BackEdge be = pair.first;
		std::set<DFGNode *> rec_nodes = pair.second;
		if (rec_nodes.find(node) != rec_nodes.end())
		{
			if (be.second->rootDP != NULL)
			{
				std::stringstream ss;
				ss << "RecSet(" << be.first->idx << "," << be.second->idx << ")"
						  << " : ";
				for (DFGNode *n : rec_nodes)
				{
					ss << n->idx << ",";
				}
				ss << "\n";
				LOG(ROUTE)<<ss.str();
				setBackEdges.insert(pair.first);
			}
		}
	}

	// PE *samplePE = cgra->PEArr[0][0][0];
	std::unordered_map<std::string, int> OpLatency = cgra->getGlobalOPMinLatencyMap();
	// samplePE->getNonMEMIns(OpLatency);
	// samplePE->getMemOnlyIns(OpLatency);

	int maxLat = LARGE_VALUE;

	for (BackEdge be : setBackEdges)
	{
		int maxLatency = be.second->rootDP->getLat() + cgra->get_t_max();
		int noDownStreamOps = 0;
		//		maxLatency = maxLatency - OpLatency[be.first->op];
		//std::cout << "maxLatency = " << maxLatency << "\n";
		std::map<int, std::set<DFGNode *>> asapOrder;
		std::map<int, int> asapMaxOpLat;
		std::map<int, int> asapMaxLat;

		if (RecCycles.find(be) != RecCycles.end())
		{
			//			for(DFGNode* n : RecCycles[be]){
			//				asapOrder[n->ASAP].insert(n);
			//			}
			std::vector<DFGNode *> longPath = getLongestDFGPath(node, be.first);
			for (int i = 0; i < longPath.size(); ++i)
			{
				asapOrder[longPath[i]->ASAP].insert(longPath[i]);
			}
		}

		beParentInfo bpi;
		bpi.dsMEMfound = false;

		if (RecCyclesLS.find(be) != RecCyclesLS.end())
		{
			//			for(DFGNode* n : RecCyclesLS[be]){
			//				asapOrder[n->ASAP].insert(n);
			//			}
			std::vector<DFGNode *> longPath = getLongestDFGPath(node, be.first);
			for (int i = 0; i < longPath.size(); ++i)
			{
				asapOrder[longPath[i]->ASAP].insert(longPath[i]);
			}
			maxLatency = maxLatency + 2; //the store need not to finish
			bpi.isLDST = true;
		}

		int upstreamOPs = 0;
		for (std::pair<int, std::set<DFGNode *>> pair : asapOrder)
		{
			int maxOplatency = 0;
			std::stringstream output_ss;
			output_ss<< "ops : ";
			for (DFGNode *n : pair.second)
			{
				output_ss<< "idx=" << n->idx << "[" << n->op << "]"
						  << "(" << OpLatency[n->op] << ")"
						  << ",";
				int new_lat = OpLatency[n->op];
				if (new_lat > maxOplatency)
					maxOplatency = new_lat;
			}
			output_ss << "\n";
			output_ss << "ASAP=" << pair.first << ",OPLAT=" << maxOplatency << "\n";

			if ((bpi.dsMEMfound == false) && (node->ASAP < pair.first))
			{
				if (maxOplatency == 2)
				{
					output_ss << "MEM FOUND SET TRUE!\n";
					bpi.dsMEMfound = true;
					bpi.uptoMEMops = upstreamOPs;
				}
			}

			if (node->ASAP < pair.first)
			{
				upstreamOPs++;
			}

			asapMaxOpLat[pair.first] = maxOplatency;

			std::cout <<output_ss.str();
		}

		std::map<int, std::set<DFGNode *>>::reverse_iterator rit = asapOrder.rbegin();

		int prevLat = maxLatency;
		//std::cout << "prevLat=" << prevLat << "\n";
		while (rit != asapOrder.rend())
		{
			int asap = (*rit).first;
			asapMaxLat[asap] = prevLat - asapMaxOpLat[asap];
			prevLat = asapMaxLat[asap];

			if (asap > node->ASAP)
			{
				noDownStreamOps++;
			}

			rit++;
		}

		//		beParentInfo bpi;
		bpi.beParent = be.first;
		bpi.lat = asapMaxLat[node->ASAP];
		bpi.downStreamOps = noDownStreamOps;
		beParentDests[be.second->rootDP] = bpi;

		//std::cout << "asapMaxLat[node->ASAP]=" << asapMaxLat[node->ASAP] << "\n";
		if (asapMaxLat[node->ASAP] < maxLat)
		{
			maxLat = asapMaxLat[node->ASAP];
			downSteamOps = noDownStreamOps;
		}
	}

	if (maxLat != LARGE_VALUE)
	{
		std::cout << "getMaxLatencyBE :: node=" << node->idx << " maxLat = " << maxLat << "\n";
		//		assert(false);
	}
	//std::cout << "getMaxLatencyBE done!\n";
	return maxLat;
}

void CGRAXMLCompile::PathFinderMapper::addPseudoEdgesOrphans(DFG *dfg)
{

	std::set<int> orphanNodes;

	for (DFGNode &node : dfg->nodeList)
	{
		if (node.parents.empty())
		{
			orphanNodes.insert(node.idx);
		}
	}

	for (int nodeIdx : orphanNodes)
	{
		DFGNode *node = dfg->findNode(nodeIdx);

		std::map<int, DFGNode *> asapchild;
		for (DFGNode *child : node->children)
		{
			if (node->childNextIter[child])
				continue;
			asapchild[child->ASAP] = child;
		}

		assert(!asapchild.empty());
		DFGNode *earliestChild = (*asapchild.begin()).second;

		std::map<int, DFGNode *> asapcousin;
		for (DFGNode *parent : earliestChild->parents)
		{
			if (parent == node)
				continue;
			if (parent->childNextIter[earliestChild])
				continue;
			asapcousin[parent->ASAP] = parent;
		}

		if (!asapcousin.empty())
		{
			DFGNode *latestCousin = (*asapcousin.rbegin()).second;

			//std::cout << "[THILINI] Adding Pseudo Connection :: parent=" << latestCousin->idx << ",to" << node->idx << "\n";
			latestCousin->children.push_back(node);
			latestCousin->childNextIter[node] = 0;
			latestCousin->childrenOPType[node] = "P";
			node->parents.push_back(latestCousin);
		}
	}

	assert(false);
}

std::vector<CGRAXMLCompile::DFGNode *> CGRAXMLCompile::PathFinderMapper::getLongestDFGPath(
	DFGNode *src, DFGNode *dest)
{

	std::vector<DFGNode *> result;
	if (src == dest)
	{
		result.push_back(src);
		return result;
	}

	std::set<std::pair<DFGNode *, int>> q_init;
	std::queue<std::set<std::pair<DFGNode *, int>>> q;

	// PE *samplePE = cgra->PEArr[0][0][0];
	std::unordered_map<std::string, int> oplatencyMap = cgra->getGlobalOPMinLatencyMap();
	// samplePE->getNonMEMIns(oplatencyMap);
	// samplePE->getMemOnlyIns(oplatencyMap);

	q_init.insert(std::make_pair(src, oplatencyMap[src->op]));
	std::map<DFGNode *, std::map<int, DFGNode *>> cameFrom;
	q.push(q_init);
	std::stringstream output_ss;
	while (!q.empty())
	{
		std::set<std::pair<DFGNode *, int>> curr = q.front();
		q.pop();
		std::set<std::pair<DFGNode *, int>> next;
		for (std::pair<DFGNode *, int> p1 : curr)
		{
			DFGNode *node = p1.first;
			output_ss<< node->idx << ",";
			for (DFGNode *child : node->children)
			{
				if (node->childNextIter[child] == 1)
					continue;
				int nextLat = p1.second + oplatencyMap[child->op];
				next.insert(std::make_pair(child, nextLat));
				cameFrom[child][nextLat] = node;
			}
		}
		output_ss << "\n";
		if (!next.empty())
			q.push(next);
	}

	assert(cameFrom.find(dest) != cameFrom.end());

	DFGNode *temp = dest;
	while (temp != src)
	{
		output_ss<< temp->idx << " <-- ";
		result.push_back(temp);
		temp = (*cameFrom[temp].rbegin()).second;
	}
	result.push_back(src);
	output_ss << "\n";
	//	assert(false);

	std::reverse(result.begin(), result.end());
	LOG(ROUTE)<<output_ss.str();
	return result;
}

int CGRAXMLCompile::PathFinderMapper::getFreeMEMPeDist(PE *currPE)
{
	int currT = currPE->T;

	// for (int y = 0; y < this->cgra->get_y_max(); ++y)
	// {
	// 	//		int tdiff = std::abs()
	// 	//		PE* destPE = this->cgra->PEArr
	// }
}

std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::modifyMaxLatCandDest(
	std::map<DataPath *, int> candDestIn, DFGNode *node, bool &changed)
{

	std::vector<DataPath *> res;
	vector<pair<DataPath *, int> > candDestIn_before_trunc;

	std::map<DataPath *, beParentInfo> beParentDests;
	int downStreamOps = 0;
	int maxLat = getMaxLatencyBE(node, beParentDests, downStreamOps);

	if (maxLat != LARGE_VALUE)
		assert(!beParentDests.empty());

	changed = false;

	bool isMeMOp = checkMEMOp(node->op);

	LOG(ROUTE) << "MaxLat = " << maxLat << "\n";
	LOG(ROUTE) << "IsMEMOp = " << isMeMOp << "\n";
	LOG(ROUTE) << "candDestIn size = " << candDestIn.size() << "\n";

	for (std::pair<DataPath *, int> pair : candDestIn)
	{

		DataPath *dp = pair.first;
		FU* fu = dp->getFU();
		PE *pe = dp->getPE();
		int offset = 0;

		if ((fu->isMEMFU()) && (isMeMOp == false))
		{
			offset = 1;
		}

		if (cgra->minLatBetweenPEs > 0)
		{
			assert(cgra->minLatBetweenPEs == 1);
			int max_dist = 0;
			//be means backedge
			for (std::pair<DataPath *, beParentInfo> pair : beParentDests)
			{
				//				if(pair.second.isLDST == false){
				PE *bePE = pair.first->getPE();
				// int dx = std::abs(bePE->X - pe->X);
				// int dy = std::abs(bePE->Y - pe->Y);
				// int dist = dx + dy;
				int dist = cgra->getQuickTimeDistBetweenPEs(bePE,pe);

				if (pair.second.isLDST == true)
				{
					dist = 0;
				}

				int dsOps = pair.second.downStreamOps;
				if (pair.second.dsMEMfound)
				{
					// dist = pe->X;
					dist = cgra->getTimeClosestMEMPE(pe);
					dsOps = pair.second.uptoMEMops;
					LOG(ROUTE) << "**MEM FOUND DOWN**\n";
				}

				if (maxLat != LARGE_VALUE)
				{
					LOG(ROUTE)<< "pe=" << pe->getName() << ",";
					LOG(ROUTE) << "dist=" << dist << ",";
					LOG(ROUTE) << "slack=" << pair.second.lat - maxLat << ",";
					LOG(ROUTE)<< "downstreamOps=" << dsOps << "\n";
				}

				int lat_slack = pair.second.lat - maxLat;
				assert(lat_slack >= 0);
				dist = dist - lat_slack - dsOps;

				if (dist > max_dist)
					max_dist = dist;
				//				}
			}

			if (max_dist > 0)
				max_dist = max_dist - 1; // can reach the neighbours in the same cycle
			offset += max_dist;
		}

		if (pair.second <= maxLat - offset)
		{
			candDestIn_before_trunc.push_back(pair);
			res.push_back(pair.first);
		}
		else
		{
			//printf("[ModifyMaxLatency] pair.second=%d maxLat=%d offset=%d\n",pair.second, maxLat, offset);
			changed = true;
		}
	}
    //std::cout << "[SIZES] :: Res = " << res.size() << " before_trunc = " << candDestIn_before_trunc.size() <<"\n";
	res = selectLeastLat(candDestIn_before_trunc);

	return res;
}


std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::selectLeastLat(
		vector<pair<DataPath *, int> > candDestIn)
{
	std::vector<DataPath *> res;
	//sort(candDestIn.begin(), candDestIn.end(), CGRAXMLCompile::PathFinderMapper::cmp);

	int count = 0;
	for(int i=0 ; i < 100;i++)
	{
		for (auto& it : candDestIn) {
			//std::cout << "[DEST LAT] latency = " << it.second << "\n";
			if(it.second == i)
			{
				res.push_back(it.first);
				count ++;
				if (count==50)
				{
					break;
				}
			}
		}
		if (count==50)
		{
			break;
		}
	}
	return res;
}

bool CGRAXMLCompile::PathFinderMapper::canExitCurrPE(LatPort p)
{
	//lzy: as the name shows, this is to check the  whether the port can lead a path which can exit the current PE. 
	std::set<LatPort> alreadyVisited;

	std::stack<LatPort> st;
	st.push(p);

	//Todo check currDP can execute the operation
	DataPath *dp = static_cast<DataPath *>(p.second->getMod());
	if (dp->getMappedNode() == NULL)
		return true;

	PE *srcPE = p.second->getMod()->getPE();
	assert(srcPE);

	while (!st.empty())
	{
		LatPort currPort = st.top();
		st.pop();
		PE *currPE = currPort.second->getMod()->getPE();
		assert(currPE);
		if (currPE != srcPE)
			return true;
		alreadyVisited.insert(currPort);
		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);
		for (LatPort lp : nextPorts)
		{
			if (alreadyVisited.find(lp) != alreadyVisited.end())
				continue;
			st.push(lp);
		}
	}
	return false;
}

bool CGRAXMLCompile::PathFinderMapper::checkMEMOp(string op)
{
	if (op.find("OLOAD") != string::npos || op.find("OSTORE") != string::npos)
	{
		return false;
	}

	if (op.find("LOAD") != string::npos || op.find("STORE") != string::npos)
	{
		return true;
	}

	return false;
}

void CGRAXMLCompile::PathFinderMapper::GetAllSupportedOPs(Module* currmod, unordered_set<string>& supp_ops, unordered_set<string>& supp_pointers){
	//std::cout << "GetAllSupportedOPs :: currmod = " << currmod->getFullName() << "\n";

	if(FU* fu = dynamic_cast<FU*>(currmod)){
		for(auto it = fu->supportedOPs.begin(); it != fu->supportedOPs.end(); it++){
			supp_ops.insert(it->first);
		}
	}

	if(DataPath* dp = dynamic_cast<DataPath*>(currmod)){
		for(string s : dp->accesible_memvars){
			supp_pointers.insert(s);
		}
	}

	if(CGRA* cgra_ins = dynamic_cast<CGRA*>(currmod)){
		for(Module* submod : cgra_ins->subModArr[0]){
			GetAllSupportedOPs(submod,supp_ops,supp_pointers);
		}
	}
	else{
		for(Module* submod : currmod->subModules){
			GetAllSupportedOPs(submod,supp_ops,supp_pointers);
		}
	}

}

bool CGRAXMLCompile::PathFinderMapper::Check_DFG_CGRA_Compatibility(){

	unordered_set<string> all_supp_ops;
	unordered_set<string> all_supp_pointers;
	std::cout << "Start checking compatibility\n";
	GetAllSupportedOPs(cgra,all_supp_ops,all_supp_pointers);
	std::cout << "Supported_ops_there\n";
	unordered_set<string> base_pointers;

	std::stringstream output_ss;
	std::cout << "all supported pointers : \n";
	output_ss << "all supported pointers : \n";
	for(string ptr : all_supp_pointers){
		output_ss << "\t" << ptr << "\n";
	}
	//std::cout << "all required pointers : \n";
	output_ss << "all required pointers : \n";
	std::cout << "all required pointers : \n";
	for(auto it = dfg->pointer_sizes.begin(); it != dfg->pointer_sizes.end(); it++){
		output_ss << "\t" << it->first << ",size = " << it->second << "\n";
	}
	std::cout << "all required pointers : \n";
	for(DFGNode& node : dfg->nodeList){
		//std::cout << "Supported_ops\n";
		string op = node.op;
		std::cout << "Supported_ops op->"<< op << "\n";
		if(all_supp_ops.find(op) == all_supp_ops.end()){
			output_ss << "op=" << op << " is not supported in this CGRA, exiting....\n";
			exit(EXIT_FAILURE);
			std::cout << "Supported_ops_false\n";
			return false;
		}
		//std::cout << "Supported_ops_true\n";

	}
	std::cout << "Supported_ops_true\n";
	if(cgra->is_spm_modelled){
		for(auto it = dfg->ldst_pointer_sizes.begin(); it != dfg->ldst_pointer_sizes.end(); it++){
			string pointer = it->first;
			if(all_supp_pointers.find(pointer) == all_supp_pointers.end()){
				std::cout << "pointer=" << pointer << " is not present in the CGRA, exiting....\n";
				exit(EXIT_FAILURE);
				std::cout << "Supported_pointer_false\n";
				return false;
			}
		}
	}
	else{
		output_ss << "SPMs are not modelled, therefore ignoring supported pointers check.\n";
		std::cout << "SPM Not modled\n";
	}
	std::cout << "Supported_pointer_true\n";

	LOG(ARCH)<<output_ss.str();
	std::cout << "TRUE\n";
	return true;
}

void CGRAXMLCompile::PathFinderMapper::UpdateVariableBaseAddr(){

	assert(cgra);
	assert(dfg);

	// unordered_map<Module*,int> spm_addr;
	// for(auto it = cgra->Variable2SPM.begin(); it != cgra->Variable2SPM.end(); it++){
	// 	string var = it->first;
	// 	Module* spm = it->second;

	// 	if(spm_addr.find(spm) == spm_addr.end()){
	// 		spm_addr[spm] = 0;
	// 	}

	// 	int size = dfg->pointer_sizes[var];
	// 	cout << "UpdateVariableBaseAddr :: var = " << var << ", spm = " << spm->getFullName() << ", base_addr = " << spm_addr[spm] << "\n";
	// 	cgra->Variable2SPMAddr[var] = spm_addr[spm];

	// 	spm_addr[spm] = spm_addr[spm] + size;
	// }


	for(DFGNode& node : dfg->nodeList){
		if(node.gep_offset != -1){
			assert(!node.base_pointer_name.empty());
			cout << "base_pointer name = " << node.base_pointer_name << "\n";
			assert(cgra->Variable2SPMAddr.find(node.base_pointer_name) != cgra->Variable2SPMAddr.end());
			node.constant = node.gep_offset + cgra->Variable2SPMAddr[node.base_pointer_name];
		}
		else if(node.op.find("OLOAD") != string::npos || node.op.find("OSTORE") != string::npos){
			//for outer loop load and stores, set the constant to the base address
			assert(!node.base_pointer_name.empty());
			assert(cgra->Variable2SPMAddr.find(node.base_pointer_name) != cgra->Variable2SPMAddr.end());
			node.constant = cgra->Variable2SPMAddr[node.base_pointer_name];
		}
	}
	// exit(EXIT_SUCCESS);
}

void CGRAXMLCompile::PathFinderMapper::printHyCUBEBinary(CGRA* cgra) {
	//maybe using 1D vector of InsFarr. And use (x,y,t) index

	std::vector<InsFormat> InsFArr;
	InsFormatIter insFIter;
	
	int max_index = (cgra->get_t_max() + 1) * cgra->get_x_max() * cgra->get_y_max();
	InsFArr.reserve(max_index + cgra->get_x_max() * cgra->get_y_max());
	std::string binFName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs)  + "_XDim=" + std::to_string(this->cgra->get_x_max()) + "_YDim=" + std::to_string(this->cgra->get_y_max()) + "_II=" + std::to_string(cgra->get_t_max()/this->cgra->vec_size) + "_V=" + std::to_string(this->cgra->vec_size)+ "_MTP=" + std::to_string(enableMutexPaths) + "_binary.bin";

	for(int i = 0; i < max_index; i++){
		InsFArr.push_back(InsFormat{});
	}

	for (int t = 0; t < cgra->get_t_max(); ++t) {
		//printf("[T] T=%d\n",t);
		vector<PE *> peList = this->cgra->getSpatialPEList(t);

				int iter=0;
				for (PE *pe : peList)
				{
					//printf("[PE] pe_name=%s\n",pe->getFullName().c_str());
					FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

					DFGNode* currentMappedOP = dp->getMappedNode();

					int prev_t;
					int X = 0;
					int Y = 0;
					X = pe->X;
					Y = pe->Y;

					PE* prevPE;
					FU* prevFU;
					DataPath* prevDP;
					DFGNode* mappedOP;

					string str1 = "IterationCounter";

					if((strstr(pe->getFullName().c_str(),str1.c_str())))
					{
						insFIter.stride = std::bitset<2>(this->dfg->stride).to_string();
						insFIter.maxCount = std::bitset<16>(this->dfg->max_count).to_string();
						continue;
					}

					if(cgra->get_t_max()==1)
					{
						prev_t = t;
						prevPE = pe;
						prevFU = fu;
						prevDP = dp;
						mappedOP = currentMappedOP;
					}
					else
					{
						prev_t = (t + 2*cgra->get_t_max() - 1)%cgra->get_t_max();
						vector<PE *> prevPEList = this->cgra->getSpatialPEList(prev_t);

						for (PE *prevPEx : prevPEList)
						{
							if((prevPEx->X == pe->X) && (prevPEx->Y == pe->Y))
								prevPE = prevPEx;
						}
						//prevPE = prevPEList.at(iter);
						prevFU = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevFU);
						prevDP = static_cast<DataPath*>(prevFU->getSubMod("DP0"));
						//mappedOP = prevDP->getMappedNode();
						mappedOP = dp->getMappedNode();
					}


					//printf("Here1\n");
/*					string str1 = "IterationCounter";
					if(dp->getMappedNode())
					{
						if(((dp->getMappedNode()->op.compare("SELECT") == 0) and mappedOP->idx == 0) && (strstr(pe->getFullName().c_str(),str1.c_str())))
						{
						  //printf("Here\n");
						  insFIter.stride = std::bitset<2>(this->dfg->stride).to_string();
						  insFIter.maxCount = std::bitset<16>(this->dfg->max_count).to_string();
						  continue;
						}
					}else
					{
						if((strstr(pe->getFullName().c_str(),str1.c_str())))
						{
							continue;
						}
					}*/
					//printf("[PE_LIST]  T=%d Y=%d X=%d pe_name=%s\n",t,Y,X,pe->getFullName().c_str());
					//printf("[PE] pe_name=%s\n",pe->getFullName().c_str());
					if(mappedOP)
					{
						if((mappedOP->op.compare("SELECT") == 0))
						{
							printf("t=%d X=%d,Y=%d IDX=%s IDX_DP=%s IDX_CurrDP=%s\n",t,X,Y,mappedOP->op.c_str(),dp->getMappedNode()->op.c_str(),prevDP->getMappedNode()->op.c_str());
							continue;
						}
					}
					//printf("Herex\n");
					Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
					Port* easto = pe->getOutPort("EAST_O"); assert(easto);
					Port* westo = pe->getOutPort("WEST_O"); assert(westo);
					Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

					DFGNode* north_o_node = northo->getNode();
					DFGNode* east_o_node = easto->getNode();
					DFGNode* west_o_node = westo->getNode();
					DFGNode* south_o_node = southo->getNode();

					iter++;

					Module* mod =  westo->getMod();
					DataPath *mod_dp = static_cast<DataPath *>(mod);


					Port* i1_ip = dp->getInPort("I1"); assert(i1_ip);
					Port* i2_ip = dp->getInPort("I2"); assert(i2_ip);
					Port* p_ip = dp->getInPort("P"); assert(p_ip);
					//setting bypass bits
					Port* northi = pe->getInPort("NORTH_I"); assert(northi);
					Port* easti = pe->getInPort("EAST_I"); assert(easti);
					Port* westi = pe->getInPort("WEST_I"); assert(westi);
					Port* southi = pe->getInPort("SOUTH_I"); assert(southi);
					DFGNode* northi_node = northi->getNode();
					DFGNode* easti_node = easti->getNode();
					DFGNode* westi_node = westi->getNode();
					DFGNode* southi_node = southi->getNode();



					InsFormat insF;
					//PE* prevPE = pe;
					//printf("Heren\n");
					//XBar
					if(north_o_node){

						if(north_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

							if(northi_node && northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
#ifdef FLEX2
								insF.northo = "00011";
#else
								insF.northo = "0011";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.northo = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.northo = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.northo = "11011";
								}
								else
								{
									insF.northo = "00011";
								}
#else
								insF.northo = "1011";
#endif
							}
							//insF.northo = "011";
						}
						else if(north_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							if(easti_node && easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
#ifdef FLEX2
								insF.northo = "00000";
#else
								insF.northo = "0000";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.northo = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.northo = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.northo = "11000";
								}
								else
								{
									insF.northo = "00000";
								}
#else
								insF.northo = "1000";
#endif
							}
							//insF.northo = "000";
						}
						else if(north_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							if(westi_node &&  westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
#ifdef FLEX2
								insF.northo = "000010";
#else
								insF.northo = "0010";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.northo = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.northo = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.northo = "11010";
								}
								else
								{
									insF.northo = "00010";
								}
#else
								insF.northo = "1010";
#endif
							}
							//insF.northo = "010";
						}
						else if(north_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							if(southi_node &&	southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
#ifdef FLEX2
									insF.northo = "00001";
#else
									insF.northo = "0001";
#endif
								}
								else{
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.northo = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.northo = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.northo = "11001";
								}
								else
								{
									insF.northo = "00001";
								}
#else
									insF.northo = "1001";
#endif
								}
							//insF.northo = "001";
						}
						else if(north_o_node == pe->getSingleRegPort("TREG_RI")->getNode() &&
								northo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
#ifdef FLEX2
							insF.northo = "00101";
#else
							insF.northo = "1101";
#endif
						}

						else if(north_o_node == fu->getOutPort("DP0_T")->getNode() && northo->getLat() == fu->getOutPort("DP0_T")->getLat()){
#ifdef FLEX2
							insF.northo = "00100";
#else
							insF.northo = "1100";
#endif
						}
						else{
							//std::cout << "Port : " << northo->getFullName() << ",node = " << northo->getNode()->idx << ", source not found!\n";
							//assert(false);
#ifdef FLEX2
							insF.northo = "00111";
#else
							insF.northo = "1111";
#endif
						}
					}


					else{
#ifdef FLEX2
							insF.northo = "00111";
#else
						insF.northo = "1111";
#endif
					}
					//printf("Heree\n");
					if(east_o_node){

						if(east_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

							if(northi_node &&	northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
#ifdef FLEX2
								insF.easto = "00011";
#else
								insF.easto = "0011";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.easto = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.easto = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.easto = "11011";
								}
								else
								{
									insF.easto = "00011";
								}
#else
								insF.easto = "1011";
#endif
							}

						}
						else if(east_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							if(easti_node &&	easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
#ifdef FLEX2
								insF.easto = "00000";
#else
								insF.easto = "0000";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.easto = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.easto = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.easto = "11000";
								}
								else
								{
									insF.easto = "00000";
								}
#else
								insF.easto = "1000";
#endif
							}
							//insF.easto = "000";
						}
						else if(east_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							if(westi_node &&	westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
#ifdef FLEX2
								insF.easto = "00010";
#else
								insF.easto = "0010";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.easto = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.easto = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.easto = "11010";
								}
								else
								{
									insF.easto = "00010";
								}
#else
								insF.easto = "1010";
#endif
							}
							//insF.easto = "010";
						}
						else if(east_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							if(southi_node &&	southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
#ifdef FLEX2
								insF.easto = "00001";
#else
								insF.easto = "0001";
#endif
								}
								else{
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.easto = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.easto = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.easto = "11001";
								}
								else
								{
									insF.easto = "00001";
								}
#else
									insF.easto = "1001";
#endif
								}
							//insF.easto = "001";
						}
						else if(east_o_node == pe->getSingleRegPort("TREG_RI")->getNode() &&
								easto->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
#ifdef FLEX2
							insF.easto = "00101";
#else
							insF.easto = "1101";
#endif
						}
						else if(east_o_node == fu->getOutPort("DP0_T")->getNode() && easto->getLat() == fu->getOutPort("DP0_T")->getLat()){
#ifdef FLEX2
							insF.easto = "00100";
#else
							insF.easto = "1100";
#endif
						}
						else{
#ifdef FLEX2
							insF.easto = "00111";
#else
							insF.easto = "1111";
#endif
						}
					}
					else{
#ifdef FLEX2
							insF.easto = "00111";
#else
							insF.easto = "1111";
#endif
					}
					//printf("Herew\n");
					if(west_o_node){

						if(west_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							if(northi_node &&	northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
#ifdef FLEX2
								insF.westo = "00011";
#else
								insF.westo = "0011";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.westo = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.westo = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.westo = "11011";
								}
								else
								{
									insF.westo = "00011";
								}
#else
								insF.westo = "1011";
#endif
							}
							//insF.westo = "011";
						}
						else if(west_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							if(easti_node &&	easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
#ifdef FLEX2
								insF.westo = "00000";
#else
									insF.westo = "0000";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.westo = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.westo = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.westo = "11000";
								}
								else
								{
									insF.westo = "00000";
								}
#else
								insF.westo = "1000";
#endif
							}
							//insF.westo = "000";
						}
						else if(west_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							if(westi_node &&
									westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
#ifdef FLEX2
								insF.westo = "00010";
#else
									insF.westo = "0010";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.westo = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.westo = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.westo = "11010";
								}
								else
								{
									insF.westo = "00010";
								}
#else
								insF.westo = "1010";
#endif
							}
							//insF.westo = "010";
						}
						else if(west_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							if(southi_node &&
									southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
#ifdef FLEX2
									insF.westo = "00001";
#else
									insF.westo = "0001";
#endif
								}
								else{
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.westo = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.westo = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.westo = "11001";
								}
								else
								{
									insF.westo = "00001";
								}
#else
									insF.westo = "1001";
#endif
								}
							//insF.westo = "001";
						}
						else if(west_o_node ==  pe->getSingleRegPort("TREG_RI")->getNode() &&
								westo->getLat() ==  pe->getSingleRegPort("TREG_RI")->getLat()){
#ifdef FLEX2
							insF.westo = "00101";
#else
							insF.westo = "1101";
#endif
						}

						else if(west_o_node == fu->getOutPort("DP0_T")->getNode() && westo->getLat() == fu->getOutPort("DP0_T")->getLat()){
#ifdef FLEX2
							insF.westo = "00100";
#else
							insF.westo = "1100";
#endif
						}
						else{
							//std::cout << "Port : " << westo->getFullName() << ",node = " << westo->getNode()->idx << ", source not found!\n";
							//assert(false);
#ifdef FLEX2
							insF.westo = "00111";
#else
							insF.westo = "1111";
#endif
						}
					}
					else{
#ifdef FLEX2
							insF.westo = "00111";
#else
							insF.westo = "1111";
#endif
					}
					//printf("Heres\n");
					if(south_o_node){

						if(south_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							if(northi_node &&
								northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
#ifdef FLEX2
								insF.southo = "00011";
#else
								insF.southo = "0011";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.southo = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.southo = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.southo = "11011";
								}
								else
								{
									insF.southo = "00011";
								}
#else
									insF.southo = "1011";
#endif
							}
							//insF.southo = "011";
						}
						else if(south_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							if(easti_node &&
									easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
								if(pe->getSingleRegPort("ER_RI")->getNode()){
									//assert(pe->getSingleRegPort("ER_RI")->getNode() != pe->getInternalPort("EAST_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.southo = "00000";
#else
								insF.southo = "0000";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.southo = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.southo = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.southo = "11000";
								}
								else
								{
									insF.southo = "00000";
								}
#else
								insF.southo = "1000";
#endif
							}
							//insF.southo = "000";
						}
						else if(south_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							if(westi_node &&
									westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
								if(pe->getSingleRegPort("WR_RI")->getNode()){
									//assert(pe->getSingleRegPort("WR_RI")->getNode() != pe->getInternalPort("WEST_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.southo = "00010";
#else
								insF.southo = "0010";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.southo = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.southo = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.southo = "11010";
								}
								else
								{
									insF.southo = "00010";
								}
#else
								insF.southo = "1010";
#endif
							}
							//insF.southo = "010";
						}
						else if(south_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							if(southi_node &&
									southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
									if(pe->getSingleRegPort("SR_RI")->getNode()){
										//assert(pe->getSingleRegPort("SR_RI")->getNode() != pe->getInternalPort("SOUTH_XBARI")->getNode());
									}
#ifdef FLEX2
								insF.southo = "00001";
#else
								insF.southo = "0001";
#endif
								}
								else{
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.southo = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.southo = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.southo = "11001";
								}
								else
								{
									insF.southo = "00001";
								}
#else
									insF.southo = "1001";
#endif
								}
							//insF.southo = "001";
						}
						else if(south_o_node == pe->getSingleRegPort("TREG_RI")->getNode() &&
								southo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
#ifdef FLEX2
								insF.southo = "00101";
#else
								insF.southo = "1101";
#endif
						}
						else if(south_o_node == fu->getOutPort("DP0_T")->getNode() && southo->getLat() == fu->getOutPort("DP0_T")->getLat()){
#ifdef FLEX2
								insF.southo = "00100";
#else
								insF.southo = "1100";
#endif
						}
						else{
							//std::cout << "Port : " << southo->getFullName() << ",node = " << southo->getNode()->idx << ", source not found!\n";
							//assert(false);
#ifdef FLEX2
								insF.southo = "00111";
#else
								insF.southo = "1111";
#endif
						}
					}
					else{
#ifdef FLEX2
								insF.southo = "00111";
#else
								insF.southo = "1111";
#endif
					}

					//printf("Herep\n");
					if(p_ip->getNode()){

						if(p_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							if(northi_node &&
								northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
								if(pe->getSingleRegPort("NR_RI")->getNode()){
									//assert(pe->getSingleRegPort("NR_RI")->getNode() != pe->getInternalPort("NORTH_XBARI")->getNode());
								}
#ifdef FLEX
								insF.alu_p = "00011";
#else
								insF.alu_p = "0011";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_p = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_p = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_p = "11011";
								}
								else
								{
									insF.alu_p = "00011";
								}
#else
								insF.alu_p = "1011";
#endif
							}
							//insF.alu_p = "011";
						}
						else if(p_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							if(easti_node &&
									easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
								if(pe->getSingleRegPort("ER_RI")->getNode()){
									//assert(pe->getSingleRegPort("ER_RI")->getNode() != pe->getInternalPort("EAST_XBARI")->getNode());
								}
#ifdef FLEX
								insF.alu_p = "00000";
#else
								insF.alu_p = "0000";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_p = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_p = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_p = "11000";
								}
								else
								{
									insF.alu_p = "00000";
								}
#else
								insF.alu_p = "1000";
#endif
							}
							//insF.alu_p = "000";
						}
						else if(p_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							if(westi_node &&
									westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
								if(pe->getSingleRegPort("WR_RI")->getNode()){
									//assert(pe->getSingleRegPort("WR_RI")->getNode() != pe->getInternalPort("WEST_XBARI")->getNode());
								}
#ifdef FLEX
								insF.alu_p = "00010";
#else
								insF.alu_p = "0010";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_p = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_p = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_p = "11010";
								}
								else
								{
									insF.alu_p = "00010";
								}
#else
								insF.alu_p = "1010";
#endif
							}
							//insF.alu_p = "010";
						}
						else if(p_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							if(southi_node &&
									southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
									if(pe->getSingleRegPort("SR_RI")->getNode()){
										//assert(pe->getSingleRegPort("SR_RI")->getNode() != pe->getInternalPort("SOUTH_XBARI")->getNode());
									}
#ifdef FLEX
								insF.alu_p = "00001";
#else
								insF.alu_p = "0001";
#endif
								}
								else{
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_p = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_p = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_p = "11001";
								}
								else
								{
									insF.alu_p = "00001";
								}
#else
									insF.alu_p = "1001";
#endif
								}
							//insF.alu_p = "001";
						}
						else if(p_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() &&
								p_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
#ifdef FLEX2
								insF.alu_p = "00101";
#else
								insF.alu_p = "1101";
#endif
						}
						else if(p_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && p_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
#ifdef FLEX2
								insF.alu_p = "00100";
#else
								insF.alu_p = "1100";
#endif
						}
						else if(p_ip->getNode()  == dp->getOutPort("T")->getNode() && p_ip->getLat() == dp->getOutPort("T")->getLat()){
#ifdef FLEX2
								insF.alu_p = "00100";
#else
								insF.alu_p = "1100";
#endif
						}
						else{
							//std::cout << "Port : " << p_ip->getFullName() << ",node = " << p_ip->getNode()->idx << ", source not found!\n";
							//assert(false);
#ifdef FLEX2
								insF.alu_p = "00111";
#else
								insF.alu_p = "1111";
#endif
						}
					}
					else{
#ifdef FLEX2
								insF.alu_p = "00111";
#else
								insF.alu_p = "1111";
#endif
					}
					//printf("Herei1\n");
					if(i1_ip->getNode()){

						if(i1_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							if(northi_node &&
								northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
								if(pe->getSingleRegPort("NR_RI")->getNode()){
									//assert(pe->getSingleRegPort("NR_RI")->getNode() != pe->getInternalPort("NORTH_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.alu_i1 = "00011";
#else
								insF.alu_i1 = "0011";
#endif
								if(currentMappedOP &&currentMappedOP->type_i1i2){
#ifdef FLEX2
									insF.alu_i2 = "00011";
#else
									insF.alu_i2 = "0011";
#endif
								}
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i1 = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i1 = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i1 = "11011";
								}
								else
								{
									insF.alu_i1 = "00011";
								}
#else
								insF.alu_i1 = "1011";
#endif
								if(currentMappedOP &&currentMappedOP->type_i1i2){
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i2 = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i2 = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i2 = "11011";
								}
								else
								{
									insF.alu_i2 = "00011";
								}
#else
									insF.alu_i2 = "1011";
#endif
								}
							}

						}
						else if(i1_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							if(easti_node &&
									easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
								if(pe->getSingleRegPort("ER_RI")->getNode()){
									//assert(pe->getSingleRegPort("ER_RI")->getNode() != pe->getInternalPort("EAST_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.alu_i1 = "00000";
#else
								insF.alu_i1 = "0000";
#endif
								if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
									insF.alu_i2 = "00000";
#else
									insF.alu_i2 = "0000";
#endif
								}
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i1 = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i1 = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i1 = "11000";
								}
								else
								{
									insF.alu_i1 = "00000";
								}
#else
								insF.alu_i1 = "1000";
#endif
									if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i2 = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i2 = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i2 = "11000";
								}
								else
								{
									insF.alu_i2 = "00000";
								}
#else
										insF.alu_i2 = "1000";
#endif
									}
							}

						}
						else if(i1_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							if(westi_node &&
									westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
								if(pe->getSingleRegPort("WR_RI")->getNode()){
									//assert(pe->getSingleRegPort("WR_RI")->getNode() != pe->getInternalPort("WEST_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.alu_i1 = "00010";
#else
								insF.alu_i1 = "0010";
#endif
								if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
									insF.alu_i2 = "00010";
#else
									insF.alu_i2 = "0010";
#endif
								}
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i1 = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i1 = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i1 = "11010";
								}
								else
								{
									insF.alu_i1 = "00010";
								}
#else
								insF.alu_i1 = "1010";
#endif
								if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i2 = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i2 = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i2 = "11010";
								}
								else
								{
									insF.alu_i2 = "00010";
								}
#else
									insF.alu_i2 = "1010";
#endif
								}
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							if(southi_node &&
									southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
									if(pe->getSingleRegPort("SR_RI")->getNode()){
										//assert(pe->getSingleRegPort("SR_RI")->getNode() != pe->getInternalPort("SOUTH_XBARI")->getNode());
									}
#ifdef FLEX2
									insF.alu_i1 = "00001";
#else
									insF.alu_i1 = "0001";
#endif
									if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
										insF.alu_i2 = "00001";
#else
										insF.alu_i2 = "0001";
#endif
									}
								}
								else{
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i1 = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i1 = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i1 = "11001";
								}
								else
								{
									insF.alu_i1 = "00001";
								}
#else
									insF.alu_i1 = "1001";
#endif
									if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i2 = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i2 = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i2 = "11001";
								}
								else
								{
									insF.alu_i2 = "00001";
								}
#else
										insF.alu_i2 = "1001";
#endif
									}
								}
						}
						else if(i1_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() &&
								i1_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
#ifdef FLEX2
							insF.alu_i2 = "00101";
#else
							insF.alu_i1 = "1101";
#endif
							if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
								insF.alu_i2 = "00101";
#else
								insF.alu_i2 = "1101";
#endif
							}
						}
						else if(i1_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && i1_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
#ifdef FLEX2
							insF.alu_i2 = "00100";
#else
							insF.alu_i1 = "1100";
#endif
							if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
								insF.alu_i2 = "00100";
#else
								insF.alu_i2 = "1100";
#endif
							}
						}


						else if(i1_ip->getNode() == dp->getOutPort("T")->getNode() && i1_ip->getLat() == dp->getOutPort("T")->getLat()){
#ifdef FLEX2
							insF.alu_i2 = "00100";
#else
							insF.alu_i1 = "1100";
#endif
							if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
								insF.alu_i2 = "00100";
#else
								insF.alu_i2 = "1100";
#endif
							}
						}
						else{
							//std::cout << "Port : " << i1_ip->getFullName() << ",node = " << i1_ip->getNode()->idx << ", source not found!\n";
							//assert(false);
#ifdef FLEX2
							insF.alu_i1 = "00111";
#else
							insF.alu_i1 = "1111";
#endif
							if(currentMappedOP && currentMappedOP->type_i1i2){
#ifdef FLEX2
								insF.alu_i1 = "00111";
#else
								insF.alu_i2 = "1111";
#endif
							}
						}
					}
					else{
#ifdef FLEX2
						insF.alu_i1 = "00111";
#else
						insF.alu_i1 = "1111";
#endif
					}

					//printf("Herei2\n");
					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
					if(i2_ip->getNode()){

						if(i2_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							if(northi_node &&
								northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
								if(pe->getSingleRegPort("NR_RI")->getNode()){
									//assert(pe->getSingleRegPort("NR_RI")->getNode() != pe->getInternalPort("NORTH_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.alu_i2 = "00011";
#else
								insF.alu_i2 = "0011";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("NR_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i2 = "01011";
								}
								else if(pe->getSingleRegPort("NR1_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i2 = "10011";
								}
								else if(pe->getSingleRegPort("NR2_RI")->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode())
								{
									insF.alu_i2 = "11011";
								}
								else
								{
									insF.alu_i2 = "00011";
								}
#else
								insF.alu_i2 = "1011";
#endif
							}
							//insF.alu_i2 = "011";
						}
						else if(i2_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							if(easti_node &&
									easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
								if(pe->getSingleRegPort("ER_RI")->getNode()){
									//assert(pe->getSingleRegPort("ER_RI")->getNode() != pe->getInternalPort("EAST_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.alu_i2 = "00000";
#else
								insF.alu_i2 = "0000";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("ER_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i2 = "01000";
								}
								else if(pe->getSingleRegPort("ER1_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i2 = "10000";
								}
								else if(pe->getSingleRegPort("ER2_RI")->getNode() == pe->getInternalPort("EAST_XBARI")->getNode())
								{
									insF.alu_i2 = "11000";
								}
								else
								{
									insF.alu_i2 = "00000";
								}
#else
								insF.alu_i2 = "1000";
#endif
							}
							//insF.alu_i2 = "000";
						}
						else if(i2_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							//std::cout << pe->getSingleRegPort("WR_RI")->getNode()->idx << "\n";
							//std::cout << pe->getInternalPort("WEST_XBARI")->getNode()->idx << "\n";
							if(westi_node &&
									westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
								if(pe->getSingleRegPort("WR_RI")->getNode()){
									//assert(pe->getSingleRegPort("WR_RI")->getNode() != pe->getInternalPort("WEST_XBARI")->getNode());
								}
#ifdef FLEX2
								insF.alu_i2 = "00010";
#else
								insF.alu_i2 = "0010";
#endif
							}
							else{
#ifdef FLEX2
								if(pe->getSingleRegPort("WR_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i2 = "01010";
								}
								else if(pe->getSingleRegPort("WR1_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i2 = "10010";
								}
								else if(pe->getSingleRegPort("WR2_RI")->getNode() == pe->getInternalPort("WEST_XBARI")->getNode())
								{
									insF.alu_i2 = "11010";
								}
								else
								{
									insF.alu_i2 = "00010";
								}
#else
								insF.alu_i2 = "1010";
#endif
							}
							//insF.alu_i2 = "010";
						}
						else if(i2_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							if(southi_node &&
									southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
									if(pe->getSingleRegPort("SR_RI")->getNode()){
										//assert(pe->getSingleRegPort("SR_RI")->getNode() != pe->getInternalPort("SOUTH_XBARI")->getNode());
									}
#ifdef FLEX2
									insF.alu_i2 = "00001";
#else
									insF.alu_i2 = "0001";
#endif
								}
								else{
#ifdef FLEX2
								if(pe->getSingleRegPort("SR_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i2 = "01001";
								}
								else if(pe->getSingleRegPort("SR1_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i2 = "10001";
								}
								else if(pe->getSingleRegPort("SR2_RI")->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode())
								{
									insF.alu_i2 = "11001";
								}
								else
								{
									insF.alu_i2 = "00001";
								}
#else
									insF.alu_i2 = "1001";
#endif
								}
							//insF.alu_i2 = "001";
						}
						else if(i2_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() &&
								i2_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
#ifdef FLEX2
							insF.alu_i2 = "00101";
#else
							insF.alu_i2 = "1101";
#endif
						}
						else if(i2_ip->getNode() == fu->getOutPort("DP0_T")->getNode() &&
								i2_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
#ifdef FLEX2
							insF.alu_i2 = "00100";
#else
							insF.alu_i2 = "1100";
#endif
						}

						else if(i2_ip->getNode() == dp->getOutPort("T")->getNode() && i2_ip->getLat() == dp->getOutPort("T")->getLat()){
#ifdef FLEX2
							insF.alu_i2 = "00100";
#else
							insF.alu_i2 = "1100";
#endif
						}
						else{
							//std::cout << "Port : " << i2_ip->getFullName() << ",node = " << i2_ip->getNode()->idx << ", source not found!\n";
							//assert(false);
#ifdef FLEX2
							insF.alu_i2 = "00111";
#else
							insF.alu_i2 = "1111";
#endif
						}
					}
					else{
#ifdef FLEX2
						insF.alu_i2 = "00111";
#else
						insF.alu_i2 = "1111";
#endif
					}
					}
					//printf("t=%d X=%d,Y=%d  east=%s south=%s west=%s north=%s \n",t,X,Y,insF.easto.c_str(),insF.southo.c_str(),insF.westo.c_str(),insF.northo.c_str());
					//TREG WE
					if( pe->getSingleRegPort("TREG_RO")->getNode() &&
							fu->getOutPort("DP0_T")->getNode() &&
							pe->getSingleRegPort("TREG_RO")->getNode() == fu->getOutPort("DP0_T")->getNode() ){
						insF.treg_we = "1";
					}
					else{
						insF.treg_we = "0";
					}

					// Register write enables


					if(pe->getSingleRegPort("NR_RO")->getNode() &&
							northi->getNode() &&
							pe->getSingleRegPort("NR_RO")->getNode() == northi->getNode()){
						insF.north_reg_we = "1";
						//printf("[REGS NORTH] :: PE_name = %s Mapped Node=%d\n",pe->getFullName().c_str(),pe->getSingleRegPort("NR_RO")->getNode()->idx);
					}
					else{
						insF.north_reg_we = "0";
					}


					if(pe->getSingleRegPort("ER_RO")->getNode() &&
							easti->getNode() &&
							pe->getSingleRegPort("ER_RO")->getNode() == easti->getNode()){
						insF.east_reg_we = "1";
						//printf("[REGS EAST] :: PE_name = %s Mapped Node=%d\n",pe->getFullName().c_str(),pe->getSingleRegPort("ER_RO")->getNode()->idx);
					}
					else{
						insF.east_reg_we = "0";
					}

					if(pe->getSingleRegPort("WR_RO")->getNode() &&
							westi->getNode() &&
							pe->getSingleRegPort("WR_RO")->getNode() == westi->getNode()){
						insF.west_reg_we = "1";
						//printf("[REGS WEST] :: PE_name = %s Mapped Node=%d\n",pe->getFullName().c_str(),pe->getSingleRegPort("WR_RO")->getNode()->idx);
					}
					else{
						insF.west_reg_we = "0";
					}

					if(pe->getSingleRegPort("SR_RO")->getNode() &&
							southi->getNode() &&
							pe->getSingleRegPort("SR_RO")->getNode() == southi->getNode()){
						insF.south_reg_we = "1";
						//printf("[REGS SOUTH] :: PE_name = %s Mapped Node=%d\n",pe->getFullName().c_str(),pe->getSingleRegPort("SR_RO")->getNode()->idx);

					}
					else{
						insF.south_reg_we = "0";
					}

					//printf("Herexx\n");

					if(northi_node &&
							northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
						if(pe->getSingleRegPort("NR_RI")->getNode()){
							std::cout << "pe=" << pe->getName() << ",node=" << pe->getSingleRegPort("NR_RI")->getNode()->idx << ",portnode=" << northi_node->idx << "\n";
							//assert(pe->getSingleRegPort("NR_RI")->getNode() != pe->getInternalPort("NORTH_XBARI")->getNode());
						}
						insF.north_reg_bypass = "0";
					}
					else{
						insF.north_reg_bypass = "1";
					}

					if(easti_node &&
							easti_node == pe->getInternalPort("EAST_XBARI")->getNode() && easti->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
						if(pe->getSingleRegPort("ER_RI")->getNode()){
							std::cout << "pe=" << pe->getName() << ",node=" << pe->getSingleRegPort("ER_RI")->getNode()->idx << ",portnode=" << easti_node->idx << "\n";
							//assert(pe->getSingleRegPort("ER_RI")->getNode() != pe->getInternalPort("EAST_XBARI")->getNode());
						}
						insF.east_reg_bypass = "0";
					}
					else{
						insF.east_reg_bypass = "1";
					}

					if(westi_node &&
							westi_node == pe->getInternalPort("WEST_XBARI")->getNode() && westi->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
						if(pe->getSingleRegPort("WR_RI")->getNode()){

						}
						insF.west_reg_bypass = "0";
					}
					else{
						insF.west_reg_bypass = "1";
					}

					if(southi_node &&
							southi_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southi->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
						if(pe->getSingleRegPort("SR_RI")->getNode()){
							std::cout << "pe=" << pe->getName() << ",node=" << pe->getSingleRegPort("SR_RI")->getNode()->idx << ",portnode=" << southi_node->idx << "\n";
							//assert(pe->getSingleRegPort("SR_RI")->getNode() != pe->getInternalPort("SOUTH_XBARI")->getNode());
						}
						insF.south_reg_bypass = "0";
					}
					else{
						insF.south_reg_bypass = "1";
					}


					if(mappedOP){
						insF.opcode = mappedOP->getBinaryString();
						//printf("t=%d X=%d,Y=%d  OP_string=%s IDX=%s\n",t,X,Y,insF.opcode.c_str(),mappedOP->op.c_str());
						if(mappedOP->npb){
							insF.negated_predicate = "1";
						}
					}
					else{
#ifdef FLEX
						if(pe->isMemPE)
							insF.opcode = "000";
						else
							insF.opcode = "00000";
#else

						insF.opcode = "00000";
#endif
					}

					if(mappedOP && mappedOP->hasConst){
						insF.constant_valid = "1";

#ifdef FLEX
						insF.constant = std::bitset<3>(pe->local_reg.size()).to_string();
						pe->local_reg.push_back(mappedOP->constant);
					}
					else if(mappedOP && mappedOP->hasOffset)
					{
							insF.constant_valid = "1";
							insF.constant = std::bitset<3>(pe->local_reg.size()).to_string();
							pe->local_reg.push_back(mappedOP->offsetVal);
					}
#else
						insF.constant = mappedOP->get27bitConstantBinaryString();
					}
#endif
					/*else if(insF.opcode == "00000"){
						// if nop, select constant 1 to enable power gating.
					    insF.constant_valid = "1";
						insF.constant = "000000000000000000000000001";
					}
					else if(mappedOP && (mappedOP->hasShift || mappedOP->hasOffset) && !mappedOP->hasConst)
					{

						insF.constant_valid = "0";
						std::string shiftStr;
						std::string maxCountStr;
						if(mappedOP->hasShift && mappedOP->hasOffset)
						{

							shiftStr = std::bitset<2>(mappedOP->shiftVal).to_string();
							maxCountStr = std::bitset<16>(mappedOP->offsetVal).to_string();

						}else if(mappedOP->hasShift)
						{

							shiftStr = std::bitset<2>(mappedOP->shiftVal).to_string();
							maxCountStr = std::bitset<16>(0).to_string();

						}else if(mappedOP->hasOffset)
						{

							maxCountStr = std::bitset<16>(mappedOP->offsetVal).to_string();
							shiftStr = std::bitset<2>(0).to_string();
						}else
						{
							maxCountStr = std::bitset<16>(0).to_string();
							shiftStr = std::bitset<2>(0).to_string();
						}
						std::string additionalStr = std::bitset<9>(0).to_string();
						insF.constant = additionalStr + shiftStr + maxCountStr;

					}*/
					else{
						insF.constant_valid = "0";
						//					insF.constant = "123456789012345678901234567";
#ifdef FLEX
						insF.constant = "000";
#else
						insF.constant = "000000000000000000000000000";
#endif
					}


					if( mappedOP && mappedOP->npb){
						insF.negated_predicate = "1";
						//				assert(false);
					}
					else{
						insF.negated_predicate = "0";
					}

#ifdef FLEX
					if(pe->isMemPE)
						insF.rem_zeros =  std::bitset<8>(0).to_string();
					else
						insF.rem_zeros =  std::bitset<6>(0).to_string();
					//if(!prevDP->isReserved)
					//{
						InsFArr[getIndexOfBin(int((t)), Y, X)] = insF;
						//if(mappedOP)
							//std::cout << "T=" << t << " Y=" << Y << " X=" << X <<  " offset=" << t%this->cgra->vec_size << "\n";
						//	printf("t=%d X=%d,Y=%d mapped_t =%d offset=%d IDX=%d OP=%s\n",t,X,Y,int((t)/this->cgra->vec_size),t%this->cgra->vec_size,mappedOP->idx,mappedOP->op.c_str());
	//				}
#else
					InsFArr[getIndexOfBin(t+1, Y, X)] = insF;
#endif
 					// InsFArr[t+1][Y][X] = insF;

					//		}
					//	}
				}
	}

	//printf("[JUMPL]\n");
	InsFormat jumpl;
	jumpl.negated_predicate = "0";
#ifdef FLEX
	jumpl.constant_valid = "0";
	jumpl.constant = 	"000";

#else
	jumpl.constant_valid = "1";
	jumpl.constant = "000000000000000" + std::bitset<4>(1).to_string() + std::bitset<4>(cgra->get_t_max()).to_string() + std::bitset<4>(1).to_string();
	jumpl.opcode = "11110";
#endif
	//assert(jumpl.constant.size() == 27);

	jumpl.north_reg_we = "0";
	jumpl.east_reg_we = "0";
	jumpl.west_reg_we = "0";
	jumpl.south_reg_we = "0";
	jumpl.treg_we = "0";
	jumpl.north_reg_bypass = "0";
	jumpl.east_reg_bypass = "0";
	jumpl.west_reg_bypass = "0";
	jumpl.south_reg_bypass = "0";
	jumpl.alu_p = "111";
	jumpl.alu_i1 = "111";
	jumpl.alu_i2 = "111";
	jumpl.northo = "111";
	jumpl.easto = "111";
	jumpl.westo = "111";
	jumpl.southo = "111";

	int x;
	int y;

	vector<PE *> peList = this->cgra->getSpatialPEList(0);

	for (PE *pe : peList)
	{

		x = pe->X;
		y = pe->Y;
#ifdef FLEX
		jumpl.alu_p = InsFArr[getIndexOfBin(int(cgra->get_t_max()), y, x)].alu_p;
		jumpl.alu_i1 = InsFArr[getIndexOfBin(int(cgra->get_t_max()), y, x)].alu_i1;
		jumpl.alu_i2 = InsFArr[getIndexOfBin(int(cgra->get_t_max()), y, x)].alu_i2;
		jumpl.northo = InsFArr[getIndexOfBin(int(cgra->get_t_max()), y, x)].northo;
		jumpl.easto = InsFArr[getIndexOfBin(int(cgra->get_t_max()), y, x)].easto;
		jumpl.westo = InsFArr[getIndexOfBin(int(cgra->get_t_max()), y, x)].westo;
		jumpl.southo = InsFArr[getIndexOfBin(int(cgra->get_t_max()), y, x)].southo;
		if(pe->isMemPE)
		{
			jumpl.rem_zeros =  std::bitset<8>(0).to_string();
			jumpl.opcode = "111";
		}
		else
		{
			jumpl.rem_zeros =  std::bitset<6>(0).to_string();
			jumpl.opcode = "11110";
		}
#else
		jumpl.alu_p = InsFArr[getIndexOfBin(cgra->get_t_max(), y, x)].alu_p;
		jumpl.alu_i1 = InsFArr[getIndexOfBin(cgra->get_t_max(), y, x)].alu_i1;
		jumpl.alu_i2 = InsFArr[getIndexOfBin(cgra->get_t_max(), y, x)].alu_i2;
		jumpl.northo = InsFArr[getIndexOfBin(cgra->get_t_max(), y, x)].northo;
		jumpl.easto = InsFArr[getIndexOfBin(cgra->get_t_max(), y, x)].easto;
		jumpl.westo = InsFArr[getIndexOfBin(cgra->get_t_max(), y, x)].westo;
		jumpl.southo = InsFArr[getIndexOfBin(cgra->get_t_max(), y, x)].southo;
#endif
		//InsFArr[getIndexOfBin(0, y, x)] = jumpl;
		// InsFArr[0][y][x] = jumpl;
	}
	//printf("[JUMPL DONE]\n");
	std::map<int,std::map<int,bool>> peEnable;
	std::map<int,std::map<int,bool>> configOPEnable;
	std::map<int,std::map<int,std::map<int,bool>>> configOPEnableCycle;
	std::map<int,std::map<int,std::map<int,bool>>> configRouteEnable;

	for(int y = 0 ; y < cgra->get_y_max() ; y++){
		for(int x = 0 ; x < cgra->get_x_max() ; x++){
			bool active=false;
			bool active_op =false;
			bool active_r = false;
			for(int t = 0 ; t < cgra->get_t_max() ; t++){
				//std::cout << "X=" << x << " Y=" << y << " t=" << t << "\n";
				if(((InsFArr[getIndexOfBin(t, y, x)].opcode != "000") && ((x==0) or (x==cgra->get_x_max()-1))) or ((InsFArr[getIndexOfBin(t, y, x)].opcode != "00000") && !((x==0) or (x==cgra->get_x_max()-1))))
				{
					active = true;
					active_op = true;
				}
#ifdef FLEX2
				else if(InsFArr[getIndexOfBin(t, y, x)].alu_p != "00111")
				{
					active = true;
					active_op = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].alu_i1 != "00111")
				{
					active = true;
					active_op = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].alu_i2 != "00111")
				{
					active = true;
					active_op = true;
				}
				if(InsFArr[getIndexOfBin(t, y, x)].northo != "00111")
				{
					active = true;
					active_r = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].easto != "00111")
				{
					active = true;
					active_r = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].westo != "00111")
				{
					//std::cout << "X=" << x << " Y=" << y << "\n";
					active = true;
					active_r = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].southo != "00111")
				{
					active = true;
					active_r = true;
				}
#else
				else if(InsFArr[getIndexOfBin(t, y, x)].alu_p != "1111")
				{
					active = true;
					active_op = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].alu_i1 != "1111")
				{
					active = true;
					active_op = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].alu_i2 != "1111")
				{
					active = true;
					active_op = true;
				}
				if(InsFArr[getIndexOfBin(t, y, x)].northo != "1111")
				{
					active = true;
					active_r = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].easto != "1111")
				{
					active = true;
					active_r = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].westo != "1111")
				{
					//std::cout << "X=" << x << " Y=" << y << "\n";
					active = true;
					active_r = true;
				}
				else if(InsFArr[getIndexOfBin(t, y, x)].southo != "1111")
				{
					active = true;
					active_r = true;
				}
#endif
				configRouteEnable[t][y][x] = active_r;
				configOPEnableCycle[t][y][x] = active_op;
			}
			peEnable[y][x] = active;
			configOPEnable[y][x] = active_op;
			//configRouteEnable[y][x] = active_r;
		}
	}
	//printf("[CLKEN]\n");
#ifdef FLEX
	for (int t = 0; t < cgra->get_t_max(); ++t) {
		vector<PE *> peList = this->cgra->getSpatialPEList(t);
		for (PE *pe : peList)
		{
			string str1 = "IterationCounter";

			if((strstr(pe->getFullName().c_str(),str1.c_str())))
			{
				continue;
			}
			Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
			Port* easto = pe->getOutPort("EAST_O"); assert(easto);
			Port* westo = pe->getOutPort("WEST_O"); assert(westo);
			Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

			FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
			DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
			Port* i1_ip = dp->getInPort("I1"); assert(i1_ip);
			Port* i2_ip = dp->getInPort("I2"); assert(i2_ip);
			Port* p_ip = dp->getInPort("P"); assert(p_ip);

			Port* northi = pe->getInPort("NORTH_I"); assert(northi);
			Port* easti = pe->getInPort("EAST_I"); assert(easti);
			Port* westi = pe->getInPort("WEST_I"); assert(westi);
			Port* southi = pe->getInPort("SOUTH_I"); assert(southi);

			int x = pe->X;
			int y = pe->Y;
			//std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t <<"\n";
			//std::cout << pe->getFullName() << " t=" << t << "\n";
			if(dp->getMappedNode())
			{

					for (int i=1;i<(this->cgra->vec_size);i++)
					{
						//std::cout << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].opcode << "\n";
						if(!pe->isMemPE)
						{
							if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].opcode == "00000")
							{
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].opcode = InsFArr[getIndexOfBin(t, y, x)].opcode;
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].negated_predicate = InsFArr[getIndexOfBin(t, y, x)].negated_predicate;
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].constant = InsFArr[getIndexOfBin(t, y, x)].constant;
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].constant_valid = InsFArr[getIndexOfBin(t, y, x)].constant_valid;
								//std::cout << "[non_MEM]Assigning the opcodes\n";
							}else
							{
								std::cout << "[Error_nonMEM] :: Not properly reserved node = "<< dp->getMappedNode()->idx <<"\n";
								std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t << " config=" << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].opcode<<"\n";
							}
						}
						else
						{
							if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].opcode == "000")
							{
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].opcode = InsFArr[getIndexOfBin(t, y, x)].opcode;
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].negated_predicate = InsFArr[getIndexOfBin(t, y, x)].negated_predicate;
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].constant = InsFArr[getIndexOfBin(t, y, x)].constant;
								InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].constant_valid = InsFArr[getIndexOfBin(t, y, x)].constant_valid;
								//std::cout << "Assigning the opcodes\n";
							}else
							{
								std::cout << "[Error] :: Not properly reserved node = "<< dp->getMappedNode()->idx <<"\n";
								std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t << " config=" << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].opcode<<"\n";
							}
						}
#ifdef FLEX2
						if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_i1 == "00111" && InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_i2 == "00111" && InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_p == "00111")

#else
						if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_i1 == "1111" && InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_i2 == "1111" && InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_p == "1111")
#endif
						{
							InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_i1 = InsFArr[getIndexOfBin(t, y, x)].alu_i1;
							InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_i2 = InsFArr[getIndexOfBin(t, y, x)].alu_i2;
							InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_p = InsFArr[getIndexOfBin(t, y, x)].alu_p;
						}else
						{
							std::cout << "[Error] :: Not properly reserved node = "<< dp->getMappedNode()->idx <<"\n";
							//std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t << " config=" << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].alu_i1<<"\n";
						}
					}
			}
			//printf("[OPCODE]\n");
#ifdef FLEX2
			if(InsFArr[getIndexOfBin(t, y, x)].easto != "00111" && easto->getNode())
#else
			if(InsFArr[getIndexOfBin(t, y, x)].easto != "1111" && easto->getNode())
#endif
			{
				//std::cout << "Inside East\n";
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					//std::cout << "vec="<< i << " "<< InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].easto << "\n";
#ifdef FLEX2
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].easto == "00111")
#else
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].easto == "1111")
#endif
					{
						//std::cout << "vec="<< i << " "<< t<< "\n";
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].easto = InsFArr[getIndexOfBin(t, y, x)].easto;
					}
					else
					{
						std::cout << "[Error EAST] :: Not properly reserved node = "<< easto->getNode()->idx <<"\n";
						std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t << " config=" << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].easto<<"\n";
					}
				}
			}
			//printf("[EAST]\n");
#ifdef FLEX2
			if(InsFArr[getIndexOfBin(t, y, x)].westo != "00111" && westo->getNode())
#else
			if(InsFArr[getIndexOfBin(t, y, x)].westo != "1111" && westo->getNode())
#endif
			{
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
#ifdef FLEX2
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].westo == "00111")
#else
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].westo == "1111")
#endif
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].westo = InsFArr[getIndexOfBin(t, y, x)].westo;
					else
					{
						std::cout << "[Error WEST] :: Not properly reserved node = "<< westo->getNode()->idx <<"\n";
						std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t << " config=" << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].westo<<"\n";
					}
				}
			}
			//printf("[WEST]\n");
#ifdef FLEX2
			if(InsFArr[getIndexOfBin(t, y, x)].northo != "00111" && northo->getNode())
#else
			if(InsFArr[getIndexOfBin(t, y, x)].northo != "1111" && northo->getNode())
#endif
			{
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
#ifdef FLEX2
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].northo == "00111")
#else
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].northo == "1111")
#endif
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].northo = InsFArr[getIndexOfBin(t, y, x)].northo;
					else
					{
						std::cout << "[Error NORTH] :: Not properly reserved node = "<< northo->getNode()->idx <<"\n";
						std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t << " config=" << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].northo<<"\n";
					}
				}
			}
			//printf("[NORTH]\n");
#ifdef FLEX2
			if(InsFArr[getIndexOfBin(t, y, x)].southo != "00111" && southo->getNode())
#else
			if(InsFArr[getIndexOfBin(t, y, x)].southo != "1111" && southo->getNode())
#endif
			{
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
#ifdef FLEX2
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].southo == "00111")
#else
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].southo == "1111")
#endif
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].southo = InsFArr[getIndexOfBin(t, y, x)].southo;
					else
					{
						std::cout << "[Error SOUTH] :: Not properly reserved node = "<< southo->getNode()->idx <<"\n";
						std::cout << "[Error] :: X = "<< x << " Y=" << y << " T=" << t << " config=" << InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].southo<<"\n";
					}
				}
			}
			//printf("[SOUTH]\n");
/*			if(InsFArr[getIndexOfBin(t, y, x)].east_reg_bypass != "1" && easti->getNode())
			{
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].east_reg_bypass == "1")
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].east_reg_bypass = InsFArr[getIndexOfBin(t, y, x)].east_reg_bypass;
					else
						std::cout << "[Error EAST] :: Not properly reserved node = \n";
				}
			}

			if(InsFArr[getIndexOfBin(t, y, x)].west_reg_bypass != "1" && westi->getNode())
			{
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].west_reg_bypass == "1")
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].west_reg_bypass = InsFArr[getIndexOfBin(t, y, x)].west_reg_bypass;
					else
						std::cout << "[Error WEST] :: Not properly reserved node = \n";
				}
			}

			if(InsFArr[getIndexOfBin(t, y, x)].north_reg_bypass != "1" && northi->getNode())
			{
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].north_reg_bypass == "1")
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].north_reg_bypass = InsFArr[getIndexOfBin(t, y, x)].north_reg_bypass;
					else
						std::cout << "[Error NORTH] :: Not properly reserved node = \n";
				}
			}

			if(InsFArr[getIndexOfBin(t, y, x)].south_reg_bypass != "1" && southi->getNode())
			{
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].south_reg_bypass == "1")
						InsFArr[getIndexOfBin((t+i)%cgra->get_t_max(), y, x)].south_reg_bypass = InsFArr[getIndexOfBin(t, y, x)].south_reg_bypass;
					else
						std::cout << "[Error SOUTH] :: Not properly reserved node = \n";
				}
			}*/
		}
	}
#endif
	std::cout << "Printing Iter\n";
	//printBinFileIter(insFIter,peEnable,configOPEnable,configRouteEnable,configOPEnableCycle,binFName);
	printBinFileIter(insFIter,peEnable,configOPEnable,binFName);
	//std::cout << "Printing the binary\n";
	printBinFile(InsFArr,binFName,cgra);

}

void CGRAXMLCompile::PathFinderMapper::printBinFile(
		const std::vector<InsFormat >& insFArr,
		std::string fName, CGRA* cgra) {

	std::ofstream binFile;

	binFile.open(fName, std::ios_base::app);

	std::ofstream binFile1;

	std::string binFName = "Older_format_XDim=" + std::to_string(this->cgra->get_x_max()) + "_YDim=" + std::to_string(this->cgra->get_y_max()) + "_II=" + std::to_string(cgra->get_t_max()/this->cgra->vec_size) + "_V=" + std::to_string(this->cgra->vec_size)+ "_binary.bin";


	binFile1.open(binFName, std::ios_base::app);
#ifdef FLEX
	int t_max = int(cgra->get_t_max()) ;
#else
	int t_max = cgra->get_t_max() ;
#endif
	int y_max = cgra->get_y_max();
	int x_max = cgra->get_x_max();

	//binFile << "NPB,CONSTVALID,CONST,OPCODE,REGWEN,TREGWEN,REGBYPASS,PRED,OP1,OP2,NORTH,WEST,SOUTH,EAST\n";

#ifdef FLEX
	int operation_trigger[y_max][x_max];
	int E_trigger[y_max][x_max];
	int W_trigger[y_max][x_max];
	int S_trigger[y_max][x_max];
	int N_trigger[y_max][x_max];
	std::map<int,std::map<int,std::map<int,bool>>> configOPEnableCycle;
	std::map<int,std::map<int,std::map<int,bool>>> configRouteEnable;

	binFile << "\nCONFIGURATION : ";
	for(int y = 0 ; y < y_max ; y++){
		for(int x = 0 ; x < x_max ; x++){

			binFile << "\n" << "[CONFIGURATION] Y=" << y << " X=" << x << "\n";
			binFile1 << "\n" << "Y=" << y << " X=" << x << "\n";
			std::cout << "\n" << "Y=" << y << " X=" << x << "\n";
			std::string insString_operation[t_max+1];
			std::string insString_operation_r[t_max+1];
			std::string insString_bypass[t_max+1];
			std::string insString_we[t_max+1];
			std::string insString_routing[t_max+1];
			std::string insString_routing_E[t_max+1];
			std::string insString_routing_W[t_max+1];
			std::string insString_routing_N[t_max+1];
			std::string insString_routing_S[t_max+1];

			std::string insString_operation_vec[t_max/this->cgra->vec_size+1];
			std::string insString_operation_r_vec[t_max/this->cgra->vec_size+1];
			std::string insString_bypass_vec[t_max/this->cgra->vec_size];
			std::string insString_routing_E_vec[t_max/this->cgra->vec_size+1];
			std::string insString_routing_W_vec[t_max/this->cgra->vec_size+1];
			std::string insString_routing_N_vec[t_max/this->cgra->vec_size+1];
			std::string insString_routing_S_vec[t_max/this->cgra->vec_size+1];

			if(this->cgra->vec_size > 1)
			{
			for(int t = 0 ; t < t_max ; t++){
				//	binFile << t << ":";

				insString_operation[t]  += insFArr[getIndexOfBin(t, y, x)].negated_predicate;
				insString_operation[t]  += insFArr[getIndexOfBin(t, y, x)].constant_valid;
				insString_operation[t]  += insFArr[getIndexOfBin(t, y, x)].opcode;
				//insString_operation[t]  += insFArr[getIndexOfBin(t, y, x)].treg_we;
				insString_operation[t]  += insFArr[getIndexOfBin(t, y, x)].constant;
				insString_operation_r[t]  += insFArr[getIndexOfBin(t, y, x)].alu_p;
				insString_operation_r[t]  += insFArr[getIndexOfBin(t, y, x)].alu_i2;
				insString_operation_r[t]  += insFArr[getIndexOfBin(t, y, x)].alu_i1;
				//insString[t]  += ",";

				insString_we[t]  += insFArr[getIndexOfBin(t, y, x)].north_reg_we;
				insString_we[t]  += insFArr[getIndexOfBin(t, y, x)].west_reg_we;
				insString_we[t]  += insFArr[getIndexOfBin(t, y, x)].south_reg_we;
				insString_we[t]  += insFArr[getIndexOfBin(t, y, x)].east_reg_we;
				insString_bypass[t]  += insFArr[getIndexOfBin(t, y, x)].south_reg_bypass;
				insString_bypass[t]  += insFArr[getIndexOfBin(t, y, x)].north_reg_bypass;
				insString_bypass[t]  += insFArr[getIndexOfBin(t, y, x)].west_reg_bypass;
				insString_bypass[t]  += insFArr[getIndexOfBin(t, y, x)].east_reg_bypass;
				//insString_bypass[t]  += ",";
				insString_routing[t]  += insFArr[getIndexOfBin(t, y, x)].northo;
				insString_routing[t]  += insFArr[getIndexOfBin(t, y, x)].westo;
				insString_routing[t]  += insFArr[getIndexOfBin(t, y, x)].southo;
				insString_routing[t]  += insFArr[getIndexOfBin(t, y, x)].easto;
				insString_routing_E[t]  =  insFArr[getIndexOfBin(t, y, x)].easto;
				insString_routing_W[t]  =  insFArr[getIndexOfBin(t, y, x)].westo;
				insString_routing_N[t]  =  insFArr[getIndexOfBin(t, y, x)].northo;
				insString_routing_S[t]  =  insFArr[getIndexOfBin(t, y, x)].southo;
				std::cout << "t=" << t << " operation=" << insString_operation[t] << " N=" << insString_routing_N[t] << " W=" << insString_routing_W[t] << " S=" << insString_routing_S[t] << " E=" << insString_routing_E[t] << "\n";

				//binFile << "\n";
			}

			//std::cout << "Initialize done\n";
			int count =0;


			int t=0;
			int trigger =0;
			bool trigger_set = false;
			int nop_count = 0;
			int no_op_elements = 0;
			//std::cout << "T_max=" << t_max << "\n";
			while(t < t_max){
				//std::cout << "T=" << t << "\n";
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(insString_operation[t%t_max] != insString_operation[(t+i)%t_max])
					{
						if((insString_operation[t%t_max] == "00000000000" || insString_operation[t%t_max] == "000000000") && (t > (this->cgra->vec_size-1)))
						{
							//std::cout << "[NOP operation] Detected of size = " << i << "\n";
							nop_count = i;
							t=t+i;
						}else
						{
							//std::cout << "Mismatch operation :: t= " << t << "," <<insString_operation[t] << "\n";
							t=t+1;
						}
					}else
					{
						if(i == (this->cgra->vec_size -1))
						{
							insString_operation_vec[int(t/this->cgra->vec_size)+1] = std::bitset<2>(nop_count).to_string() + insString_operation[t%t_max];
							insString_operation_r_vec[int(t/this->cgra->vec_size)+1] = insString_operation_r[t%t_max];
							no_op_elements += 1;
							if(!trigger_set)
								trigger = t;
							trigger_set = true;
							t=t+this->cgra->vec_size;
						}
						//std::cout << "Incrementing t\n";
						//t=t+this->cgra->vec_size;
					}
				}
			}
			//std::cout << "Outside loop\n";
			insString_operation_vec[0] = "0000000"+ std::bitset<3>(no_op_elements).to_string() ;

			operation_trigger[y][x] = trigger;
			//std::cout << "Operation set\n";

			t=0;
			trigger =0;
			trigger_set = false;
			while(t < t_max){

				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(insString_routing_E[t%t_max] != insString_routing_E[(t+i)%t_max])
					{
#ifdef FLEX2
						if((insString_routing_E[t%t_max] == "00111") && (t > (this->cgra->vec_size-1)))
#else
						if((insString_routing_E[t%t_max] == "1111") && (t > (this->cgra->vec_size-1)))
#endif
						{
							t=t+i;
						}else
						{
							t=t+1;
						}
					}else
					{
						if(i == (this->cgra->vec_size -1)){

							if(!trigger_set)
							{
								trigger = t;
							    trigger_set = true;
							}
							insString_routing_E_vec[int(t/this->cgra->vec_size)+1] = std::bitset<2>((t%this->cgra->vec_size)).to_string()+insString_routing_E[t%t_max];
							t=t+this->cgra->vec_size;
						}
					}
				}
			}
#ifdef FLEX2
			insString_routing_E_vec[0]="0000111";
#else
			insString_routing_E_vec[0]="001111";
#endif
			E_trigger[y][x] = trigger;
			//std::cout << "E done\n";

			t=0;
			trigger =0;
			trigger_set = false;
			while(t < t_max){
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(insString_routing_W[t%t_max] != insString_routing_W[(t+i)%t_max])
					{
#ifdef FLEX2
						if((insString_routing_W[t%t_max] == "00111") && (t > (this->cgra->vec_size-1)))
#else
						if((insString_routing_W[t%t_max] == "1111") && (t > (this->cgra->vec_size-1)))
#endif
						{
							std::cout << "[NOP W] Detected of size = " << i << "\n";
							t=t+i;
						}else
						{
							t=t+1;
						}
					}else
					{
						if(i == (this->cgra->vec_size -1)){
							if(!trigger_set)
							{
								trigger = t;
								trigger_set = true;
							}
							insString_routing_W_vec[int(t/this->cgra->vec_size)+1] = std::bitset<2>((t%this->cgra->vec_size)).to_string()+insString_routing_W[t%t_max];
							t=t+this->cgra->vec_size;
						}
						//t=t+this->cgra->vec_size;
					}
				}

			}
			//std::cout << "Done while\n";
#ifdef FLEX2
			insString_routing_W_vec[0]="0000111";
#else
			insString_routing_W_vec[0]="001111";
#endif
			W_trigger[y][x] = trigger;
			//std::cout << "W done\n";
			t=0;
			trigger =0;
			trigger_set = false;
			while(t < t_max){

				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(insString_routing_N[t%t_max] != insString_routing_N[(t+i)%t_max])
					{
#ifdef FLEX2
						if((insString_routing_N[t%t_max] == "00111") && (t > (this->cgra->vec_size-1)))
#else
						if((insString_routing_N[t%t_max] == "1111") && (t > (this->cgra->vec_size-1)))
#endif
						{
							std::cout << "[NOP N] Detected of size = " << i << "\n";
							t=t+i;
						}else
						{
							t=t+1;
						}
					}else
					{
						if(i == (this->cgra->vec_size -1)){

							if(!trigger_set)
							{
								trigger = t;
								trigger_set = true;
							}
							insString_routing_N_vec[int(t/this->cgra->vec_size)+1] = std::bitset<2>((t%this->cgra->vec_size)).to_string()+insString_routing_N[t%t_max];
							t=t+this->cgra->vec_size;
						}
						//t=t+this->cgra->vec_size;
					}
				}
			}
#ifdef FLEX2
			insString_routing_N_vec[0]="0000111";
#else
			insString_routing_N_vec[0]="001111";
#endif
			N_trigger[y][x] = trigger;
			//std::cout << "N done\n";
			t=0;
			trigger =0;
			trigger_set = false;
			while(t < t_max){
				for (int i=1;i<(this->cgra->vec_size);i++)
				{
					if(insString_routing_S[t%t_max] != insString_routing_S[(t+i)%t_max])
					{
#ifdef FLEX2
						if((insString_routing_S[t%t_max] == "00111") && (t > (this->cgra->vec_size-1)))
#else
						if((insString_routing_S[t%t_max] == "1111") && (t > (this->cgra->vec_size-1)))
#endif
						{
							std::cout << "[NOP S] Detected of size = " << i << "\n";
							t=t+i;
						}else
						{
							t=t+1;
						}
					}else
					{
						if(i == (this->cgra->vec_size -1)){

							if(!trigger_set)
							{
								trigger = t;
								trigger_set = true;
							}
							insString_routing_S_vec[int(t/this->cgra->vec_size)+1] = std::bitset<2>((t%this->cgra->vec_size)).to_string()+insString_routing_S[t%t_max];
							t=t+this->cgra->vec_size;
						}
					}
				}
			}
#ifdef FLEX2
			insString_routing_S_vec[0]="0000111";
#else
			insString_routing_S_vec[0]="001111";
#endif
			S_trigger[y][x] = trigger;
			//std::cout << "S done\n";
			for(int t = 0 ; t < t_max ; t++){

					binFile1 << t << ":";
					binFile1 << insString_operation[t]+","+insString_bypass[t]+","+insString_routing[t];
					//std::cout << "t=" << t << " : " << insString[t] << "\n";
					binFile1 << "\n";
			}
#ifdef FLEX2
			insString_operation_r_vec[0] = "001110011100111";
#else
			insString_operation_r_vec[0] = "111111111111";
#endif
			for(int t = 0 ; t < t_max/this->cgra->vec_size + 1 ; t++){
				if(insString_operation_vec[t].length() == 0)
				{
					if((x==0) or (x==x_max-1))
					{
#ifdef FLEX2
						insString_operation_vec[t] = std::bitset<10>(0).to_string() + "001110011100111";
#else
						insString_operation_vec[t] = std::bitset<10>(0).to_string() + "111111111111";
#endif
					}
					else
					{
#ifdef FLEX2
						insString_operation_vec[t] = std::bitset<12>(0).to_string() +  "001110011100111";
#else
						insString_operation_vec[t] = std::bitset<12>(0).to_string() +  "111111111111";
#endif
					}
				}
				if(insString_routing_N_vec[t].length() == 0)
#ifdef FLEX2
					insString_routing_N_vec[t] = "0000111";
#else
					insString_routing_N_vec[t] = "001111";
#endif
				if(insString_routing_S_vec[t].length() == 0)
#ifdef FLEX2
					insString_routing_S_vec[t] = "0000111";
#else
					insString_routing_S_vec[t] = "001111";
#endif
				if(insString_routing_E_vec[t].length() == 0)
#ifdef FLEX2
					insString_routing_E_vec[t] = "0000111";
#else
					insString_routing_E_vec[t] = "001111";
#endif
				if(insString_routing_W_vec[t].length() == 0)
#ifdef FLEX2
					insString_routing_W_vec[t] = "0000111";
#else
					insString_routing_W_vec[t] = "001111";
#endif
				insString_operation_vec[t] = insString_operation_vec[t] + insString_operation_r_vec[t];
#ifdef FLEX2
				if(insString_operation_vec[t].length() > 25)
					binFile << std::bitset<9>(0).to_string() + insString_operation_vec[t] + insString_routing_N_vec[t]+insString_routing_W_vec[t]+insString_routing_S_vec[t]+insString_routing_E_vec[t] << "\n";
				else
					binFile << std::bitset<11>(0).to_string() << insString_operation_vec[t] + insString_routing_N_vec[t]+insString_routing_W_vec[t]+insString_routing_S_vec[t]+insString_routing_E_vec[t] << "\n";

				if(((x==0) or (x==x_max-1)) && (insString_operation_vec[t]==(std::bitset<10>(0).to_string() + "001110011100111")))
				{
					configOPEnableCycle[t][y][x] = 0;
				}
				else if ((insString_operation_vec[t]==(std::bitset<12>(0).to_string() + "001110011100111")))
				{
					configOPEnableCycle[t][y][x] = 0;
				}else
				{
					configOPEnableCycle[t][y][x] = 1;
				}

				if((insString_routing_N_vec[t] == "0000111") && (insString_routing_W_vec[t] == "0000111") && (insString_routing_S_vec[t] == "0000111") && (insString_routing_E_vec[t] == "0000111"))
				{
					configRouteEnable[t][y][x] = 0;
				}else
				{
					configRouteEnable[t][y][x] = 1;
				}
#else

				if(insString_operation_vec[t].length() > 22)
					//binFile << "00000000" << insString_operation_vec[t] + insString_routing_N_vec[t]+insString_routing_W_vec[t]+insString_routing_S_vec[t]+insString_routing_E_vec[t] << "\n";
					binFile <<  insString_operation_vec[t] + insString_routing_N_vec[t]+insString_routing_W_vec[t]+insString_routing_S_vec[t]+insString_routing_E_vec[t] << "\n";
				else
					//binFile << "0000000000" << insString_operation_vec[t] + insString_routing_N_vec[t]+insString_routing_W_vec[t]+insString_routing_S_vec[t]+insString_routing_E_vec[t] << "\n";
					binFile << "00" << insString_operation_vec[t] + insString_routing_N_vec[t]+insString_routing_W_vec[t]+insString_routing_S_vec[t]+insString_routing_E_vec[t] << "\n";

			//	std::cout << "t=" << t << " operation=" << insString_operation_vec[t] << " N=" << insString_routing_N_vec[t] << " W=" << insString_routing_W_vec[t] << " S=" << insString_routing_S_vec[t] << " E=" << insString_routing_E_vec[t] << "\n";

				if(((x==0) or (x==x_max-1)) && (insString_operation_vec[t]==(std::bitset<10>(0).to_string() + "111111111111")))
				{
					configOPEnableCycle[t][y][x] = 0;
				}
				else if ((insString_operation_vec[t]==(std::bitset<12>(0).to_string() + "111111111111")))
				{
					configOPEnableCycle[t][y][x] = 0;
				}else
				{
					configOPEnableCycle[t][y][x] = 1;
				}

				if((insString_routing_N_vec[t] == "001111") && (insString_routing_W_vec[t] == "001111") && (insString_routing_S_vec[t] == "001111") && (insString_routing_E_vec[t] == "001111"))
				{
					configRouteEnable[t][y][x] = 0;
				}else
				{
					configRouteEnable[t][y][x] = 1;
				}
#endif
			}
		}
		else
		{
			//std::cout << "Printing\n";
			for(int t = 1 ; t < t_max +1 ; t++){
							//	binFile << t << ":";

							insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].negated_predicate;
							insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].constant_valid;
							insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].opcode;
							//insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].treg_we;

							insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].constant;
							insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].alu_p;
							insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].alu_i2;
							insString_operation[t]  += insFArr[getIndexOfBin(t-1, y, x)].alu_i1;
							insString_we[t]  += insFArr[getIndexOfBin(t-1, y, x)].north_reg_we;
							insString_we[t]  += insFArr[getIndexOfBin(t-1, y, x)].west_reg_we;
							insString_we[t]  += insFArr[getIndexOfBin(t-1, y, x)].south_reg_we;
							insString_we[t]  += insFArr[getIndexOfBin(t-1, y, x)].east_reg_we;

							insString_routing_E[t]  =  insFArr[getIndexOfBin(t-1, y, x)].easto;
							insString_routing_W[t]  =  insFArr[getIndexOfBin(t-1, y, x)].westo;
							insString_routing_N[t]  =  insFArr[getIndexOfBin(t-1, y, x)].northo;
							insString_routing_S[t]  =  insFArr[getIndexOfBin(t-1, y, x)].southo;

							//binFile << "\n";
			}
			insString_routing_N[0] = "1111";
			insString_routing_S[0] = "1111";
			insString_routing_E[0] = "1111";
			insString_routing_W[0] = "1111";
			insString_we[0] = "0000";

			for(int t = 0 ; t < t_max + 1 ; t++){

				if(insString_operation[t].length() == 0)
				{
					if((x==0) or (x==x_max-1))
						insString_operation[t] = std::bitset<9>(0).to_string() + "111111111111";
					else
						insString_operation[t] = std::bitset<11>(0).to_string() + "111111111111";
				}
				if(insString_routing_N[t].length() == 0)
					insString_routing_N[t] = "1111";
				if(insString_routing_S[t].length() == 0)
					insString_routing_S[t] = "1111";
				if(insString_routing_E[t].length() == 0)
					insString_routing_E[t] = "1111";
				if(insString_routing_W[t].length() == 0)
					insString_routing_W[t] = "1111";
				if(insString_we[t].length() == 0)
					insString_we[t] = "0000";

				if (t==0)
				{
					if(insString_operation[t].length() > 21)
						binFile << "00000" << insString_operation[t] + "00001111111111111111" << "\n";
					else
						binFile << "0000000" << insString_operation[t] + "00001111111111111111" << "\n";

				}else
				{
					if(insString_operation[t].length() > 21)
						binFile << "00000" << insString_operation[t] +insString_routing_N[t]+insString_routing_W[t]+insString_routing_S[t]+insString_routing_E[t] << "\n";
					else
						binFile << "0000000" << insString_operation[t] + insString_routing_N[t]+insString_routing_W[t]+insString_routing_S[t]+insString_routing_E[t] << "\n";
				}
				std::cout << "t=" << t << " operation=" << insString_operation[t] << " N=" << insString_routing_N[t] << " W=" << insString_routing_W[t] << " S=" << insString_routing_S[t] << " E=" << insString_routing_E[t] << "\n";

				//std::cout << insString_operation[t] << "\n";
				if(((x==0) or (x==x_max-1)) && (insString_operation[t]==(std::bitset<9>(0).to_string() + "111111111111")))
				{
					configOPEnableCycle[t][y][x] = 0;
				}
				else if ((insString_operation[t]==(std::bitset<11>(0).to_string() + "111111111111")))
				{
					configOPEnableCycle[t][y][x] = 0;
				}else
				{
					configOPEnableCycle[t][y][x] = 1;
				}

				if((insString_routing_N[t] == "1111") && (insString_routing_W[t] == "1111") && (insString_routing_S[t] == "1111") && (insString_routing_E[t] == "1111"))
				{
					configRouteEnable[t][y][x] = 0;
				}else
				{
					configRouteEnable[t][y][x] = 1;
				}

					//std::cout << "x=" << x << " y=" << y << " t=" << t << insString_operation[t] << insString_operation[t] << "," << insString_we[t] << "," << insString_routing_N[t] << "," << insString_routing_W[t] << "," << insString_routing_S[t] << "," << insString_routing_E[t] << "\n";
			}
		}
		}
	}
	if(this->cgra->vec_size > 1)
	{
	binFile << "\n";

	binFile << "[CMEMROUTEEN]\n";
	for(int t = 0;t< t_max/this->cgra->vec_size + 1 ;  t++){
		binFile << "[CMEMROUTEEN] T=" << t << " :";
		for(int y = cgra->get_y_max()-1 ; y >= 0 ; y--){
			for(int x = cgra->get_x_max()-1 ; x >= 0  ; x--){
				if(configRouteEnable[t][y][x])
				{
					binFile << "1";
				}
				else
				{
					binFile << "0";
				}
			}
		}
		binFile << "\n";
	}
	}
	if(this->cgra->vec_size > 1)
	{
	binFile << "\n";
	binFile << "[CLKENCMEM]\n";
	for(int t = 0;t< t_max/this->cgra->vec_size + 1 ;  t++){
		binFile << "[CLKENCMEM] T=" << t << " :";

		for(int y = cgra->get_y_max()-1 ; y >= 0 ; y--){
			for(int x = cgra->get_x_max()-1 ; x >= 0  ; x--){
				//std::cout << " T=" << t << " X=" << x <<" Y=" << y <<" EN=" << configOPEnableCycle[t][y][x]  << " " << configRouteEnable[t][y][x]  << "->"<<(configOPEnableCycle[t][y][x] && configRouteEnable[t][y][x])<< "\n";
				if(configOPEnableCycle[t][y][x])
				{
					binFile << "1";
				}
				else
				{
					binFile << "0";
				}
			}
		}
		binFile << "\n";
	}
	binFile << "\n";
	}else
	{
		binFile << "\n";
		binFile << "[CLKENCMEM]\n";
		for(int t = 0;t< t_max/this->cgra->vec_size + 1 ;  t++){
			binFile << "[CLKENCMEM] T=" << t << " :";

			for(int y = cgra->get_y_max()-1 ; y >= 0 ; y--){
				for(int x = cgra->get_x_max()-1 ; x >= 0  ; x--){
					//std::cout << " T=" << t << " X=" << x <<" Y=" << y <<" EN=" << configOPEnableCycle[t][y][x]  << " " << configRouteEnable[t][y][x]  << "->"<<(configOPEnableCycle[t][y][x] && configRouteEnable[t][y][x])<< "\n";
					if(configOPEnableCycle[t][y][x] || configRouteEnable[t][y][x])
					{
						binFile << "1";
					}
					else
					{
						binFile << "0";
					}
				}
			}
			binFile << "\n";
		}
		binFile << "\n";
	}

	if(this->cgra->vec_size > 1)
	{
	binFile << "\nTRIGGER :\n";
	for(int y = 0 ; y < y_max ; y++){
		for(int x = 0 ; x < x_max ; x++){
			binFile <<  "[TRIGGER] Y=" << y << " X=" << x << " :";
			binFile << std::bitset<4>(0).to_string() << std::bitset<3>(N_trigger[y][x]).to_string() <<  std::bitset<3>(W_trigger[y][x]).to_string() <<  std::bitset<3>(S_trigger[y][x]).to_string() <<  std::bitset<3>(E_trigger[y][x]).to_string() << "\n";
		}
	}

	binFile << "\nOPERATION TRIGGER :\n";
	for(int y = 0 ; y < y_max ; y++){
		for(int x = 0 ; x < x_max ; x++){
			binFile <<  "[OPERATION TRIGGER] Y=" << y << " X=" << x << " :";
			binFile << std::bitset<3>(operation_trigger[y][x]).to_string() << "\n";
		}
	}
	}
	binFile << "\n";

#else

	for(int t = 0 ; t < t_max ; t++){
		binFile << t << "\n";
		for(int y = 0 ; y < y_max ; y++){
			for(int x = 0 ; x < x_max ; x++){

				binFile << "Y=" << y << " X=" << x << ",";
				binFile << insFArr[getIndexOfBin(t, y, x)].rem_zeros;
				//Add NOP count
				binFile << insFArr[getIndexOfBin(t, y, x)].negated_predicate;
				binFile << insFArr[getIndexOfBin(t, y, x)].constant_valid;
				binFile << insFArr[getIndexOfBin(t, y, x)].opcode;
				binFile << insFArr[getIndexOfBin(t, y, x)].north_reg_we;
				binFile << insFArr[getIndexOfBin(t, y, x)].west_reg_we;
				binFile << insFArr[getIndexOfBin(t, y, x)].south_reg_we;
				binFile << insFArr[getIndexOfBin(t, y, x)].east_reg_we;
				binFile << insFArr[getIndexOfBin(t, y, x)].treg_we;
				binFile << insFArr[getIndexOfBin(t, y, x)].south_reg_bypass;
				binFile << insFArr[getIndexOfBin(t, y, x)].north_reg_bypass;
				binFile << insFArr[getIndexOfBin(t, y, x)].west_reg_bypass;
				binFile << insFArr[getIndexOfBin(t, y, x)].east_reg_bypass;
				binFile << insFArr[getIndexOfBin(t, y, x)].constant;
				binFile << insFArr[getIndexOfBin(t, y, x)].alu_p;
				binFile << insFArr[getIndexOfBin(t, y, x)].alu_i2;
				binFile << insFArr[getIndexOfBin(t, y, x)].alu_i1;
				binFile << insFArr[getIndexOfBin(t, y, x)].northo;
				binFile << insFArr[getIndexOfBin(t, y, x)].westo;
				binFile << insFArr[getIndexOfBin(t, y, x)].southo;
				binFile << insFArr[getIndexOfBin(t, y, x)].easto;

				binFile << "\n";
			}
		}
		binFile << "\n";
	}
#endif

	binFile.close();
	binFile1.close();
}

void CGRAXMLCompile::PathFinderMapper::printBinFileIter(
		const InsFormatIter insFIter,
		std::map<int,std::map<int,bool>> peEnable,
		std::map<int,std::map<int,bool>> configOPEnable,
		//std::map<int,std::map<int,std::map<int,bool>>> configRouteEnable,
		//std::map<int,std::map<int,std::map<int,bool>>> configOPEnableCycle,
		std::string fName) {

	std::ofstream binFile;
	binFile.open(fName);
	binFile << "ITER=";
	binFile << insFIter.stride;
	binFile << insFIter.maxCount;
	binFile << "\n";

	binFile << "LOOP_START_END=";
	binFile << "0000" + std::bitset<4>(1).to_string() + std::bitset<4>(cgra->get_t_max()).to_string() + std::bitset<4>(1).to_string();;
	binFile << "\n";

	binFile << "TILEEN=";
	for(int y = cgra->get_y_max()-1 ; y >= 0 ; y--){
		for(int x = cgra->get_x_max()-1 ; x >= 0  ; x--){
			//printf("Y=%d X=%d clken=%d\n",y,x,peEnable[y][x]);
			if(peEnable[y][x])
			{
				binFile << "1";
			}
			else
			{
				binFile << "0";
			}
		}
	}
	binFile << "\n";

	binFile << "CMEMOPEN=";
	for(int y = cgra->get_y_max()-1 ; y >= 0 ; y--){
		for(int x = cgra->get_x_max()-1 ; x >= 0  ; x--){
			//printf("Y=%d X=%d clken_op=%d\n",y,x,configOPEnable[y][x]);
			if(configOPEnable[y][x])
			{
				binFile << "1";
			}
			else
			{
				binFile << "0";
			}
		}
	}
	/*binFile << "\n";

	binFile << "[CMEMROUTEEN]\n";
	for(int t = 0;t< cgra->get_t_max() ;  t++){
		binFile << "[CMEMROUTEEN] T=" << t << " :";
		for(int y = cgra->get_y_max()-1 ; y >= 0 ; y--){
			for(int x = cgra->get_x_max()-1 ; x >= 0  ; x--){
				if(configRouteEnable[t][y][x])
				{
					binFile << "1";
				}
				else
				{
					binFile << "0";
				}
			}
		}
		binFile << "\n";
	}

	binFile << "\n";
	binFile << "[CLKENCMEM]\n";
	for(int t = 0;t< cgra->get_t_max() ;  t++){
		binFile << "[CLKENCMEM] T=" << t << " :";
		for(int y = cgra->get_y_max()-1 ; y >= 0 ; y--){
			for(int x = cgra->get_x_max()-1 ; x >= 0  ; x--){
				if(configOPEnableCycle[t][y][x])
				{
					binFile << "1";
				}
				else
				{
					binFile << "0";
				}
			}
		}
		binFile << "\n";
	}*/
	binFile << "\n";
	binFile.close();
}

void CGRAXMLCompile::PathFinderMapper::setIterDP(DFGNode* node, DataPath* currDP, int lat) {

	int X = currDP->getPE()->X;
	int Y = currDP->getPE()->Y;
	int T = currDP->getPE()->T;

	int iter = Y*this->cgra->get_x_max()+X;
	int time;

	string str1="IterationCounter";

	for (int i=1;i<this->cgra->get_t_max();i++)
	{
		time = (T+i) % this->cgra->get_t_max();
		lat = lat + 1;
		vector<PE *> PEList = this->cgra->getSpatialPEList(time);


		if((strstr(currDP->getFullName().c_str(),str1.c_str())))
		{
			printf("Current DP is iteration counter\n");
			for(PE* pe:PEList)
			{
				if((strstr(pe->getFullName().c_str(),str1.c_str())))
				{
					PE* nextPE = pe;
					FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
					DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
					if((strstr(nextDP->getFullName().c_str(),str1.c_str())))
					{
						node->rootDP_vec.push_back(nextDP);
						printf("Added DP=%s \n",nextDP->getFullName().c_str());
						nextDP->assignNode(node, lat, this->dfg);
					}
				}
			}
		}
		else
		{
			PE* nextPE = PEList.at(iter);
			FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
			DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
			node->rootDP_vec.push_back(nextDP);
			//printf("Added DP=%s \n",nextDP->getFullName().c_str());
			nextDP->assignNode(node, lat, this->dfg);
		}
	}
	node->rootDP_vec.push_back(currDP);
	//printf("Size of rootDP vec=%d\n",node->rootDP_vec.size());

	//for(DataPath* dp : node->rootDP_vec)
	//{
	//	printf("DP=%s\n",dp->getFullName().c_str());
	//}
}

std::map<CGRAXMLCompile::DFGNode *, std::vector<CGRAXMLCompile::Port *>> CGRAXMLCompile::PathFinderMapper::selectClosestIterationCounter(std::map<DFGNode *, std::vector<Port *>> possibleStarts, int lat,DataPath* dest){

	std::map<DFGNode *, std::vector<Port *>> ret;
	//printf("Inside select\n");
	string str1="IterationCounter";

	int destTime = dest->getPE()->T;
	int srcTime;
	Port * p_matching;
	DFGNode* n_matching;
	int diff = lat;
	bool phiFound=false;
	for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
	{
		if((pair.first->idx == 0))
		{
			//printf("Inside 0\n");
			phiFound= true;
			n_matching = pair.first;
			for(Port* p : pair.second)
			{
				//printf("PORT=%s\n",p->getFullName().c_str());
				if(strstr(p->getFullName().c_str(),str1.c_str()))
				{
					srcTime = p->getPE()->T;
					//printf("srcTime=%d destTime=%d\n",srcTime,destTime);
					if(srcTime <= destTime)
					{
						//printf("Inside 0\n");

						if(diff >= abs((destTime - srcTime)))
						{
							//printf("LTEQ\n");


							diff = abs((destTime - srcTime));
							p_matching = p;
							//printf("StarCand=%s diff =%d\n",p->getFullName().c_str(),diff);
						}
					}else
					{

						if(diff >= abs((destTime + this->cgra->get_t_max() - srcTime)-lat))
						{
							//printf("Lat =%d src Time=%d\n",lat ,srcTime);
							diff = abs((destTime + this->cgra->get_t_max() - srcTime)-lat);
							p_matching = p;
						}
					}
				}
				else
				{
					printf("[Error]::PHI NOT mapped to Iteration Counter\n");
				}
			}
			//printf("Inside xx\n");
		}
		else
		{
			ret[pair.first]= pair.second;
		}
	}
	//printf("Inside xxxx\n");
	if(phiFound)
	{
		if(p_matching !=  NULL)
		{
			//printf("Inside xxxxxxx\n");
			ret[n_matching].push_back(p_matching);
			//printf("lat=%d srcTime=%d destTime=%d StarCand=%s DestCand=%s\n",lat,srcTime,destTime,p_matching->getFullName().c_str(),dest->getFullName().c_str());
		}
		else
			printf("[Error]::minimum latency PHI not found\n");
	}
	return ret;
}
#ifdef FLEX
void CGRAXMLCompile::PathFinderMapper::reserveVectorNodes(DataPath* currDP, DFGNode* node, int lat) {

	//printf("Start Reserving\n");
	int T = currDP->getPE()->T;
	int X = currDP->getPE()->X;
	int Y = currDP->getPE()->Y;
    int latency = lat;
	int iter = Y*this->cgra->get_x_max()+X;

	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==this->cgra->get_t_max()-1)
			T = 0;
		else
			T += 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);

		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
        latency +=1;
		nextDP->isReserved = true;
		//nextDP->assignNode(node, latency, this->dfg);
		printf("reserving vector=%s reserved=%d\n",nextDP->getFullName().c_str(),nextDP->isReserved);

	}
}

bool CGRAXMLCompile::PathFinderMapper::checkVectorDP(DataPath* currDP) {

	//printf("Start Reserving\n");
	int T = currDP->getPE()->T;
	int X = currDP->getPE()->X;
	int Y = currDP->getPE()->Y;

	int iter = Y*this->cgra->get_x_max()+X;
	//printf("Current Datapath=%s\n",currDP->getFullName().c_str());
	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==this->cgra->get_t_max()-1)
			T = 0;
		else
			T += 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
			{
				nextPE = pe;
				//printf("CurrPE=%s  NextPE=%s X=%d Y=%d\n",currDP->getPE()->getFullName().c_str(),nextPE->getFullName().c_str(),X,Y);
			}
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));

		if(nextDP->getMappedNode() != NULL)
		{
			//printf("Next Datapath=%s Mapped node=%d\n",nextDP->getFullName().c_str(),nextDP->getMappedNode()->idx);
			return false;
		}
	}
	return true;
}
bool CGRAXMLCompile::PathFinderMapper::checkVectorPort(Port* currPort) {

	int T = currPort->findParentPE()->T;
	int X = currPort->findParentPE()->X;
	int Y = currPort->findParentPE()->Y;

	int iter = Y*this->cgra->get_x_max()+X;
	string portname = currPort->getName();

	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==this->cgra->get_t_max()-1)
			T = 0;
		else
			T += 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
		Port* nextPort ;
		string str1 = "IterationCounter";

		if((nextPE->getFullName().find("IterationCounter") != std::string::npos) or (portname.find("OUT_O") != std::string::npos))
			continue;
		else if((portname.find("_O") != std::string::npos))
		{
			nextPort = nextPE->getOutPort(portname);
		}
		else if((portname.find("_T") != std::string::npos))
		{
			nextPort = nextFU->getOutPort(portname);
		}
		else if((portname.compare("T") == 0))
		{
			nextPort = nextDP->getOutPort(portname);
		}
		else if((portname.find("_I1") != std::string::npos) or (portname.find("_I2") != std::string::npos) or (portname.find("_P") != std::string::npos) or (portname.find("_I3") != std::string::npos) or (portname.find("_I4") != std::string::npos))
		{
			//printf("Okay not at FU\n");
			nextPort = nextFU->getInPort(portname);
		}
		else if((portname.find("I1") != std::string::npos) or (portname.find("I2") != std::string::npos) or (portname.find("P") != std::string::npos) or (portname.find("I3") != std::string::npos) or (portname.find("I4") != std::string::npos))
		{
			//printf("Okay not at DP\n");
			nextPort = nextDP->getInPort(portname);
		}
		else if((portname.find("_I") != std::string::npos))
		{
			nextPort = nextPE->getInPort(portname);
		}
		else if((portname.find("_RO") != std::string::npos) or (portname.find("_RI") != std::string::npos))
		{
			nextPort = nextPE->getSingleRegPort(portname);
		}
		else if((portname.find("_XBARI") != std::string::npos))
		{
			nextPort = nextPE->getInternalPort(portname);
		}

		if(nextPort->getNode() != NULL)
			return false;
	}
	return true;
}

void CGRAXMLCompile::PathFinderMapper::checkTempReservedVectorPort(Port* currPort) {

	int T = currPort->findParentPE()->T;
	int X = currPort->findParentPE()->X;
	int Y = currPort->findParentPE()->Y;

	int iter = Y*this->cgra->get_x_max()+X;
	string portname = currPort->getName();

	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==this->cgra->get_t_max()-1)
			T = 0;
		else
			T += 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
		Port* nextPort ;
		string str1 = "IterationCounter";

		if((nextPE->getFullName().find("IterationCounter") != std::string::npos) or (portname.find("OUT_O") != std::string::npos))
			continue;
		else if((portname.find("_O") != std::string::npos))
		{
			nextPort = nextPE->getOutPort(portname);
		}
		else if((portname.find("_T") != std::string::npos))
		{
			nextPort = nextFU->getOutPort(portname);
		}
		else if((portname.compare("T") == 0))
		{
			nextPort = nextDP->getOutPort(portname);
		}
		else if((portname.find("_I1") != std::string::npos) or (portname.find("_I2") != std::string::npos) or (portname.find("_P") != std::string::npos) or (portname.find("_I3") != std::string::npos) or (portname.find("_I4") != std::string::npos))
		{
			//printf("Okay not at FU\n");
			nextPort = nextFU->getInPort(portname);
		}
		else if((portname.find("I1") != std::string::npos) or (portname.find("I2") != std::string::npos) or (portname.find("P") != std::string::npos) or (portname.find("I3") != std::string::npos) or (portname.find("I4") != std::string::npos))
		{
			//printf("Okay not at DP\n");
			nextPort = nextDP->getInPort(portname);
		}
		else if((portname.find("_I") != std::string::npos))
		{
			nextPort = nextPE->getInPort(portname);
		}
		else if((portname.find("_RO") != std::string::npos) or (portname.find("_RI") != std::string::npos))
		{
			nextPort = nextPE->getSingleRegPort(portname);
		}
		else if((portname.find("_XBARI") != std::string::npos))
		{
			nextPort = nextPE->getInternalPort(portname);
		}

		//if(nextPort->getNode() != NULL)
		//	return false;
		//if(nextPort->isTempReserved)
		//	return false;
		//if(nextPort->isReserved)
		//	return false;

		currPort->isTempReserved = true;
		nextPort->isTempReserved = true;
		//std::cout << "Reserving vector port nextport = " << nextPort->getFullName() << " currport = "<< currPort->getFullName() <<"\n";
	}

	//return true;
}

void CGRAXMLCompile::PathFinderMapper::releaseTempReservedVectorPort(Port* currPort) {

	int T = currPort->findParentPE()->T;
	int X = currPort->findParentPE()->X;
	int Y = currPort->findParentPE()->Y;

	int iter = Y*this->cgra->get_x_max()+X;
	string portname = currPort->getName();

	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==this->cgra->get_t_max()-1)
			T = 0;
		else
			T += 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
		Port* nextPort ;
		string str1 = "IterationCounter";

		if((nextPE->getFullName().find("IterationCounter") != std::string::npos) or (portname.find("OUT_O") != std::string::npos))
			continue;
		else if((portname.find("_O") != std::string::npos))
		{
			nextPort = nextPE->getOutPort(portname);
		}
		else if((portname.find("_T") != std::string::npos))
		{
			nextPort = nextFU->getOutPort(portname);
		}
		else if((portname.compare("T") == 0))
		{
			nextPort = nextDP->getOutPort(portname);
		}
		else if((portname.find("_I1") != std::string::npos) or (portname.find("_I2") != std::string::npos) or (portname.find("_P") != std::string::npos) or (portname.find("_I3") != std::string::npos) or (portname.find("_I4") != std::string::npos))
		{
			//printf("Okay not at FU\n");
			nextPort = nextFU->getInPort(portname);
		}
		else if((portname.find("I1") != std::string::npos) or (portname.find("I2") != std::string::npos) or (portname.find("P") != std::string::npos) or (portname.find("I3") != std::string::npos) or (portname.find("I4") != std::string::npos))
		{
			//printf("Okay not at DP\n");
			nextPort = nextDP->getInPort(portname);
		}
		else if((portname.find("_I") != std::string::npos))
		{
			nextPort = nextPE->getInPort(portname);
		}
		else if((portname.find("_RO") != std::string::npos) or (portname.find("_RI") != std::string::npos))
		{
			nextPort = nextPE->getSingleRegPort(portname);
		}
		else if((portname.find("_XBARI") != std::string::npos))
		{
			nextPort = nextPE->getInternalPort(portname);
		}

		currPort->isTempReserved = false;
		nextPort->isTempReserved = false;
		//std::cout << "Clearing vector port nextport = " << nextPort->getFullName() << " currport = "<< currPort->getFullName() <<"\n";
	}
}

void CGRAXMLCompile::PathFinderMapper::reserveVectorPorts(Port* currPort,DFGNode* src,int lat,int idx) {

	//printf("Start Reserving\n");
	int T = currPort->findParentPE()->T;
	int X = currPort->findParentPE()->X;
	int Y = currPort->findParentPE()->Y;

	int iter = Y*this->cgra->get_x_max()+X;
	int latency = lat;
	//printf("PORT->%s ",currPort->getName().c_str());
	string portname = currPort->getName();
	//if((portname.find("_O") != std::string::npos) && (portname.find("OUT_O") == std::string::npos))
		//restrictVectorPorts(currPort);

	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==this->cgra->get_t_max()-1)
			T = 0;
		else
			T += 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
		Port* nextPort ;
		string str1 = "IterationCounter";
		//printf("Module->%s\n",nextPE->getFullName().c_str());

		if((nextPE->getFullName().find("IterationCounter") != std::string::npos) or (portname.find("OUT_O") != std::string::npos))
			continue;
		else if((portname.find("_O") != std::string::npos))
		{
			//printf("Inside outputs\n");
			nextPort = nextPE->getOutPort(portname);
		}
		else if((portname.find("_T") != std::string::npos))
		{
			//printf("Inside outputs T\n");
			nextPort = nextFU->getOutPort(portname);
		}
		else if((portname.compare("T") == 0))
		{
			//printf("Inside outputs T\n");
			nextPort = nextDP->getOutPort(portname);
		}
		else if((portname.find("_I1") != std::string::npos) or (portname.find("_I2") != std::string::npos) or (portname.find("_P") != std::string::npos) or (portname.find("_I3") != std::string::npos) or (portname.find("_I4") != std::string::npos))
		{
			nextPort = nextFU->getInPort(portname);
		}
		else if((portname.find("I1") != std::string::npos) or (portname.find("I2") != std::string::npos) or (portname.find("P") != std::string::npos) or (portname.find("I3") != std::string::npos) or (portname.find("I4") != std::string::npos))
		{
			nextPort = nextDP->getInPort(portname);
		}
		else if((portname.find("_I") != std::string::npos))
		{
			//printf("Inside inputs\n");
			nextPort = nextPE->getInPort(portname);
		}
		else if((portname.find("_RO") != std::string::npos) or (portname.find("_RI") != std::string::npos))
		{
			nextPort = nextPE->getSingleRegPort(portname);
		}
		else if((portname.find("_XBARI") != std::string::npos))
		{
			nextPort = nextPE->getInternalPort(portname);
		}

		latency +=1;
		//nextPort->setNode(src, latency, idx, this);
//		printf("reserving vector=%s\n",nextDP->getFullName().c_str());

		if(nextPort)
			nextPort->isReserved = true;

		//printf("Reserving NextPort = ->%s\n",nextPort->getFullName().c_str());

		//Mark restricted vector ports

	}
}

void CGRAXMLCompile::PathFinderMapper::restrictVectorPorts(Port* currPort) {

	//printf("Start Reserving\n");
	int T = currPort->findParentPE()->T;
	int X = currPort->findParentPE()->X;
	int Y = currPort->findParentPE()->Y;

	string portname = currPort->getName();

	for (int t=0;t<cgra->get_t_max();t++)
	{
		vector<PE *> PEList = this->cgra->getSpatialPEList(t);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}

		if( (T % this->cgra->vec_size) != ((nextPE->T)% this->cgra->vec_size))
		{
			if(portname == "EAST_O" )
				nextPE->getOutPort("EAST_O")->isRestricted = true;
			if(portname == "WEST_O")
				nextPE->getOutPort("WEST_O")->isRestricted = true;
			if(portname == "SOUTH_O")
				nextPE->getOutPort("SOUTH_O")->isRestricted = true;
			if(portname == "NORTH_O")
				nextPE->getOutPort("NORTH_O")->isRestricted = true;
		}
	}

}
void CGRAXMLCompile::PathFinderMapper::clearReservedPort()
{

	for (int t = 0; t < cgra->get_t_max(); ++t) {
		vector<PE *> peList = this->cgra->getSpatialPEList(t);

		for (PE *pe : peList)
		{
			//std::cout << pe->getFullName() <<"\n";
			string str1="IterationCounter";
			if(!(strstr(pe->getFullName().c_str(),str1.c_str())))
			{
				pe->getSingleRegPort("TREG_RO")->isTempReserved = false;
				pe->getSingleRegPort("TREG_RI")->isTempReserved = false;
				pe->getSingleRegPort("NR_RO")->isTempReserved = false;
				pe->getSingleRegPort("NR_RI")->isTempReserved = false;
				pe->getSingleRegPort("SR_RO")->isTempReserved = false;
				pe->getSingleRegPort("SR_RI")->isTempReserved = false;
				pe->getSingleRegPort("WR_RO")->isTempReserved = false;
				pe->getSingleRegPort("WR_RI")->isTempReserved = false;
				pe->getSingleRegPort("ER_RO")->isTempReserved = false;
				pe->getSingleRegPort("ER_RI")->isTempReserved = false;
			}
		}
	}
}
bool CGRAXMLCompile::PathFinderMapper::checkInCameFrom(std::unordered_map<LatPort, LatPort, hash_LatPort> cameFrom, Port * currPort)
{
	int T = currPort->findParentPE()->T;
	int X = currPort->findParentPE()->X;
	int Y = currPort->findParentPE()->Y;

	int iter = Y*this->cgra->get_x_max()+X;
	bool is_true=false;
//int latency = lat;
	string portname = currPort->getName();
	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==this->cgra->get_t_max()-1)
			T = 0;
		else
			T += 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
		Port* nextPort ;
		string str1 = "IterationCounter";
		//printf("Module->%s\n",nextPE->getFullName().c_str());

		if((nextPE->getFullName().find("IterationCounter") != std::string::npos) or (portname.find("OUT_O") != std::string::npos))
			continue;
		else if((portname.find("_O") != std::string::npos))
		{
			//printf("Inside outputs\n");
			nextPort = nextPE->getOutPort(portname);
		}
		else if((portname.find("_T") != std::string::npos))
		{
			//printf("Inside outputs T\n");
			nextPort = nextFU->getOutPort(portname);
		}
		else if((portname.compare("T") == 0))
		{
			//printf("Inside outputs T\n");
			nextPort = nextDP->getOutPort(portname);
		}
		else if((portname.find("_I1") != std::string::npos) or (portname.find("_I2") != std::string::npos) or (portname.find("_P") != std::string::npos) or (portname.find("_I3") != std::string::npos) or (portname.find("_I4") != std::string::npos))
		{
			nextPort = nextFU->getInPort(portname);
		}
		else if((portname.find("I1") != std::string::npos) or (portname.find("I2") != std::string::npos) or (portname.find("P") != std::string::npos) or (portname.find("I3") != std::string::npos) or (portname.find("I4") != std::string::npos))
		{
			nextPort = nextDP->getInPort(portname);
		}
		else if((portname.find("_I") != std::string::npos))
		{
			//printf("Inside inputs\n");
			nextPort = nextPE->getInPort(portname);
		}
		else if((portname.find("_RO") != std::string::npos) or (portname.find("_RI") != std::string::npos))
		{
			nextPort = nextPE->getSingleRegPort(portname);
		}
		else if((portname.find("_XBARI") != std::string::npos))
		{
			nextPort = nextPE->getInternalPort(portname);
		}

		for (auto i : cameFrom)
		{
		if(i.first.second == nextPort)
		{
			is_true=true;
		}
		}
	}

	for (int i=0;i<(this->cgra->vec_size-1);i++)
	{
		if(T==0)
			T = this->cgra->get_t_max()-1;
		else
			T -= 1;

		vector<PE *> PEList = this->cgra->getSpatialPEList(T);
		PE* nextPE;
		for(PE* pe:PEList)
		{
			if(pe->X == X and pe->Y == Y)
				nextPE = pe;
		}
		FU* nextFU = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(nextFU);
		DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod("DP0"));
		Port* nextPort ;
		string str1 = "IterationCounter";
		//printf("Module->%s\n",nextPE->getFullName().c_str());

		if((nextPE->getFullName().find("IterationCounter") != std::string::npos) or (portname.find("OUT_O") != std::string::npos))
			continue;
		else if((portname.find("_O") != std::string::npos))
		{
			//printf("Inside outputs\n");
			nextPort = nextPE->getOutPort(portname);
		}
		else if((portname.find("_T") != std::string::npos))
		{
			//printf("Inside outputs T\n");
			nextPort = nextFU->getOutPort(portname);
		}
		else if((portname.compare("T") == 0))
		{
			//printf("Inside outputs T\n");
			nextPort = nextDP->getOutPort(portname);
		}
		else if((portname.find("_I1") != std::string::npos) or (portname.find("_I2") != std::string::npos) or (portname.find("_P") != std::string::npos) or (portname.find("_I3") != std::string::npos) or (portname.find("_I4") != std::string::npos))
		{
			nextPort = nextFU->getInPort(portname);
		}
		else if((portname.find("I1") != std::string::npos) or (portname.find("I2") != std::string::npos) or (portname.find("P") != std::string::npos) or (portname.find("I3") != std::string::npos) or (portname.find("I4") != std::string::npos))
		{
			nextPort = nextDP->getInPort(portname);
		}
		else if((portname.find("_I") != std::string::npos))
		{
			//printf("Inside inputs\n");
			nextPort = nextPE->getInPort(portname);
		}
		else if((portname.find("_RO") != std::string::npos) or (portname.find("_RI") != std::string::npos))
		{
			nextPort = nextPE->getSingleRegPort(portname);
		}
		else if((portname.find("_XBARI") != std::string::npos))
		{
			nextPort = nextPE->getInternalPort(portname);
		}

		for (auto i : cameFrom)
		{
		if(i.first.second == nextPort)
		{
			is_true=true;
		}
		}
	}

}
#endif
