/*
 * PathFinderMapper.cpp
 *
 *  Created on: 31 Mar 2018
 *      Author: manupa
 *      Edited By: Thilini
 */

#include "PathFinderMapper.h"

#include "HeuristicMapper.h"
#include <queue>
#include <assert.h>
#include <math.h>
#include <algorithm> // std::reverse
#include "DataPath.h"
#include "FU.h"

#include <stack>
#include <functional>
#include <set>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <bitset>

namespace CGRAXMLCompile
{

} /* namespace CGRAXMLCompile */

struct hash_LatPort { 
    size_t operator()(const pair<int, CGRAXMLCompile::Port*>& p) const
    { 
        auto hash1 = hash<int>{}(p.first); 
        auto hash2 = hash<CGRAXMLCompile::Port*>{}(p.second); 
        return hash1 ^ hash2; 
    } 
}; 

bool CGRAXMLCompile::PathFinderMapper::LeastCostPathAstar(LatPort start,
														  LatPort end, DataPath *endDP, std::vector<LatPort> &path,std::vector<LatPort> pathMSB, int &cost, DFGNode *node,
														  std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode,bool isStreaming,bool isPtoC)
{


	std::unordered_map<LatPort, int, hash_LatPort> cost_to_port;
	std::unordered_map<LatPort, LatPort, hash_LatPort> cameFrom;
	std::unordered_map<LatPort, int, hash_LatPort> curr_hops_to_port;

	path.clear();
	mutexPaths.clear();

	bool detailedDebug = false;


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

			return dist_dest;
		}



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


	};

	std::priority_queue<port_heuristic> q;
	q.push(port_heuristic(start, 0, lessthanII));


	cost_to_port[start] = 0;
	curr_hops_to_port[start] = 0;

	LatPort currPort;
	std::vector<LatPort> deadEnds;

	std::map<LatPort, std::shared_ptr<std::unordered_set<Port *>>> paths;

	std::unordered_set<Port *> emptyset;

	std::vector<LatPort> finalPath;

	Port *newNodeDPOut = endDP->getPotOutputPort(currNode);

	std::set<Port *> newNodeDPOutCP = newNodeDPOut->getMod()->getConflictPorts(newNodeDPOut);

	std::set<Port *> endPortCP = end.second->getMod()->getConflictPorts(end.second);



	int curr_least_cost_to_end = INT32_MAX;
	std::vector<LatPort> regs;
	bool inRecSet=false;
	for (std::pair<BackEdge, std::set<DFGNode *>> pair : RecCycles)
	{
		std::set<DFGNode *> rec_nodes = pair.second;
		if (rec_nodes.find(node) != rec_nodes.end())
		{
			inRecSet=true;
		}
	}
	while (!q.empty())
	{
		port_heuristic curr = q.top();
		currPort = curr.p;
		q.pop();
		std::unordered_set<Port *> *currPath;
		std::vector<LatPort> *currPathVec;

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

		assert(curr_hops_to_port.find(currPort) != curr_hops_to_port.end());
		if(curr_hops_to_port[currPort] > cgra->max_hops){
			continue;
		}

		if (currPort == end)
		{
			if(cost_to_port[currPort] < curr_least_cost_to_end){
				curr_least_cost_to_end = cost_to_port[currPort];
			}
			continue;
		}

		if(cost_to_port[currPort] > curr_least_cost_to_end){
			continue;
		}


		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);

		int q_len = q.size();


		for (LatPort nextLatPort : nextPorts)
		{
			Port *nextPort = nextLatPort.second;

			if (nextLatPort.first > end.first)
				continue; //continue if the next port has higher latency
			assert(nextLatPort.first - currPort.first <= 1);

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

			if (newNodeDPOutCP.find(nextPort) != newNodeDPOutCP.end())
			{
				continue;
			}

			if (endPortCP.find(nextPort) != endPortCP.end())
			{
				continue;
			}
#ifdef DUAL_STREAMING

			if(std::find(pathMSB.begin(),pathMSB.end(),nextLatPort) != pathMSB.end())
				continue;

			if((start.second->getName().find("LSB")!=string::npos) && (nextLatPort.second->getName().find("MSB")!=string::npos))
				continue;

			if((start.second->getName().find("MSB")!=string::npos) && (nextLatPort.second->getName().find("LSB")!=string::npos))
				continue;

			if(nextLatPort.second->getMSB() && (start.second->getName().find("LSB")!=string::npos))
				continue;
			if(nextLatPort.second->getLSB() && (start.second->getName().find("MSB")!=string::npos))
				continue;
#endif

#ifdef ROUTER_CONFIG
			std::string currPString = currPort.second->getName();
			std::string nextPString = nextLatPort.second->getName();
			size_t found0 = currPString.find("XBARI");
			size_t found1 = currPString.find("DP0_T");
			size_t found2 = currPString.find("TREG_RI");
			size_t found4 = nextPString.find("DP0_T");

			size_t found5 = currPString.find("DP0.T");
			size_t found6 = nextPString.find("DP0.P");
			size_t found7 = nextPString.find("DP0.I2");

			int crossbarCost=0;
			if((found0 !=string::npos or found1 !=string::npos or found2 !=string::npos) && !(found4 !=string::npos))
			{
				//crossbarCost = getCrossbarSimilarityCostwithSim(currPort,nextLatPort);
				//crossbarCost = getCrossbarSimilarityCost(currPort,nextLatPort);
				//crossbarCost = getCrossbarSimilarityCostDual(currPort,nextLatPort,false);
				crossbarCost = getCrossbarSimilarityCostDualclass4n(currPort,nextLatPort,false);

				//crossbarCost = getCrossbarSimilarityCostHetero(currPort,nextLatPort,false);


			}

			if(found5 !=string::npos and (found6 !=string::npos or found7 !=string::npos))
			{
				//crossbarCost = getCrossbarSimilarityCostwithSim(currPort,nextLatPort);
				//crossbarCost = getCrossbarSimilarityCost(currPort,nextLatPort);
				//crossbarCost = getCrossbarSimilarityCostHetero(currPort,nextLatPort,false);
				crossbarCost = getCrossbarSimilarityCostDualclass4n(currPort,nextLatPort,false);

			}

			if((crossbarCost >= ROUTER_COST))
			{
				continue;
			}
#endif


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

				if (nextLatPort.first == currPort.first)
				{
					nextLatPort.first = nextLatPort.first + 1;
				}

			}

			if (true)
			{ // unmapped port
				if (detailedDebug)
					std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
				if (detailedDebug)
					std::cout << "latency = " << nextLatPort.first << ",";
				int nextPortCost = cost_to_port[currPort] + calculateCost(currPort, nextLatPort, end, isStreaming, isPtoC, regs);

				if(congestedPorts[nextLatPort.second].size() == 1){
					if(congestedPorts[nextLatPort.second].find(node) != congestedPorts[nextLatPort.second].end())
					{
						nextPortCost -= nextLatPort.second->getCongCost();
					}
				}


				if((nextLatPort.second->getPE()->is_hot) && !((node->children.size()+node->parents.size())>3) && (nextLatPort.second->getPE() != end.second->getPE()))
					nextPortCost = nextPortCost + UOPCostFactor;

				if(nextPortCost < 0)

				if (nextPort->getNode() == node)
				{
						nextPortCost = cost_to_port[currPort];
				}
				if(nextPortCost < 0)
									std::cout << "THILINI:: cost=" << nextPortCost << "\tcurrPort-> " << currPort.second->getFullName() <<"\tnextPort->" << nextLatPort.second->getFullName() << "\n";

//---- THILINI:: Adding routing similarity
#ifdef ROUTER_CONFIG
				nextPortCost += crossbarCost;

#endif


				if (checkRecParentViolation(currNode, nextLatPort))
				{
					std::cout << "Port is not inserted, since it violated recurrence parent..\n";
					continue;
				}
				if(nextPortCost < 0)
					std::cout << "THILINI:: cost=" << nextPortCost << "\tcurrPort-> " << currPort.second->getFullName() <<"\tnextPort->" << nextLatPort.second->getFullName() << "\n";//<< "\tStart port-> " << start.second->getFullName() << "\tEnd Port-> " << end.second->getFullName() <<"\n";


				if (nextPortCost < cost_to_port[currPort])
				{
					std::cout << "nextPortCost = " << nextPortCost << "\n";
					std::cout << "cost_to_port[currPort] = " << cost_to_port[currPort] << "\n";
				}
				assert(nextPortCost >= cost_to_port[currPort]);

				if (cost_to_port.find(nextLatPort) != cost_to_port.end())
				{

					if (cost_to_port[nextLatPort] > nextPortCost)
					{

						cost_to_port[nextLatPort] = nextPortCost;

						cameFrom[nextLatPort] = currPort;
#ifdef DUAL_STREAMING
						if (isRegConType1 || isRegConType2)
						{
							bool found = false;
							for(LatPort latPR:regs){
								if((currPort.second->getName() == latPR.second->getName()) && (currPort.second->getPE()->X == latPR.second->getPE()->X) && (currPort.second->getPE()->Y == latPR.second->getPE()->Y))
									found =true;
							}
							if(!found)
							{
								regs.push_back(currPort);
							}
						}
#endif
						if(nextLatPort.first == currPort.first && nextLatPort.second->getPE() != currPort.second->getPE()){

							curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort] + 1;
						}
						else if(nextLatPort.first != currPort.first){
							curr_hops_to_port[nextLatPort] = 0;
						}
						else{
							curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort];
						}	


						if (!lessthanII )
						{
							std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
							newPath->insert(currPort.second);
							port_heuristic ph(nextLatPort, end, nextPortCost, newPath);
							ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
							ph.pathVec->push_back(currPort);

							q.push(ph);
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
#ifdef DUAL_STREAMING
						if (isRegConType1 || isRegConType2)
						{
							bool found = false;
							for(LatPort latPR:regs){
								if((currPort.second->getName() == latPR.second->getName()) && (currPort.second->getPE()->X == latPR.second->getPE()->X) && (currPort.second->getPE()->Y == latPR.second->getPE()->Y))
									found =true;
							}
							if(!found)
							{
								regs.push_back(currPort);
							}
						}
#endif
					if(nextLatPort.first == currPort.first && nextLatPort.second->getPE() != currPort.second->getPE()){

						curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort] + 1;
					}
					else if(nextLatPort.first != currPort.first){
						curr_hops_to_port[nextLatPort] = 0;
					}	
					else{
						curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort];
					}	


					if (!lessthanII )
					{
						std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
						newPath->insert(currPort.second);

						port_heuristic ph(nextLatPort, end, nextPortCost, newPath);

						ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
						ph.pathVec->push_back(currPort);

						q.push(ph);
					}
					else
					{

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


		return false; //routing failure
	}

	path.clear();

	currPort = end;
	while (currPort != start)
	{
		path.push_back(currPort);
		assert(cameFrom.find(currPort) != cameFrom.end());
		assert(currPort != cameFrom[currPort]);
		int prev_cost = cost_to_port[currPort];
		currPort = cameFrom[currPort];

	}
	path.push_back(start);
	std::reverse(path.begin(), path.end());
	cost = cost_to_port[end];

	if(congestedPorts[end.second].size() == 1){
		if(congestedPorts[end.second].find(node) == congestedPorts[end.second].end())
		{
			cost += endDP->getPotOutputPort(currNode)->getCongCost();
		}
	}


	//check if paths is working
	if (!lessthanII)
	{
		paths[end]->insert(end.second);
		finalPath.push_back(end);
		if (paths[end]->size() != path.size())
		{
			std::cout << "paths[end] size = " << paths[end]->size() << ",path.size() = " << path.size() << "\n";

			std::cout << "path = \n";
			for (LatPort lp : path)
			{
				std::cout << lp.second->getFullName() << ",lat=" << lp.first << "\n";
				if (paths[end]->find(lp.second) == paths[end]->end())
				{
					std::cout << "Not found in paths!\n";
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

		}

		path.clear();
		path = finalPath;
	}


	return true;
}

bool CGRAXMLCompile::PathFinderMapper::estimateRouting(DFGNode *node,
													   std::priority_queue<dest_with_cost> &estimatedRoutes,
													   DFGNode **failedNode)
{

	std::map<DFGNode *, std::vector<Port *>> possibleStarts;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	bool detailedDebug = false;

	for (DFGNode *parent : node->parents)
	{

		if (parent->rootDP != NULL)
		{ //already mapped

#ifdef DUAL_STREAMING
			string opType = parent->getOPtype(node);

				assert(parent->rootDP->getOutputDP()->getOutPort("T_LSB"));
				possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T_LSB"));

#else
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));
#endif
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
		{ // already mapped
			std::cout << "child=" << child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
			assert(child->rootDP->getLat() != -1);
			if (node->childrenOPType[child] == "PS")
			{
				std::cout << "Skipping.....\n";
				continue;
			}

#ifdef DUAL_STREAMING
			string opType = node->childrenOPType[child];
			string opType1 = "_MSB";
			string opType2 = "_LSB";
			if(opType != "P"){
				opType1 = opType + opType1;
				opType2 = opType + opType2;
				assert(child->rootDP->getInPort(opType1));
				assert(child->rootDP->getInPort(opType2));

				alreadyMappedChildPorts[child] = child->rootDP->getInPort(opType2);
			}else{
				assert(child->rootDP->getInPort(opType));
				alreadyMappedChildPorts[child] = child->rootDP->getInPort(opType);
			}

#else
			string opType = node->childrenOPType[child];
			string opType2 = "";
			opType2 = opType + opType2;
			assert(child->rootDP->getInPort(opType2));
			alreadyMappedChildPorts[child] = child->rootDP->getInPort(opType2);
#endif
			int ii = child->rootDP->getCGRA()->get_t_max();
			assert(child->rootDP->getLat() != -1);
			alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii);
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
			if (FU *fu = dynamic_cast<FU *>(submod))
			{

				if (fu->supportedOPs.find(node->op) == fu->supportedOPs.end())
				{
					continue;
				}

				if (fu->currOP.compare(node->op) == 0)
				{
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{
							
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
				else if (fu->currOP.compare("NOP") == 0)
				{
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							bool is_mem_op = node->op.find("LOAD") != string::npos || node->op.find("STORE") != string::npos;

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

	std::cout << "Candidate Dests = " << candidateDests.size() << "\n";
	if (candidateDests.empty())
		return false;

	std::cout << "Check Latancy\n";
	int minLat = getlatMinStartsPHI(node, possibleStarts);
	std::map<DataPath *, int> minLatDests = getLatCandDests(candidateDests, minLat);
	bool changed = false;
	candidateDests = modifyMaxLatCandDest(minLatDests, node, changed);
	std::cout << "Candidate Dests = " << candidateDests.size() << "\n";
	int ii = this->cgra->get_t_max();

	std::vector<std::pair<DataPath *,std::pair<std::vector<DFGNode *>,std::vector<DFGNode *>>>> candidateDestsX;


//---- THILINI:: Adding crossbar and opcode
#ifdef OPCODE_CONST_CONFIG

	std::map<DataPath *, int> candidateDestWithConst = checkConstSimilarity(candidateDests,node);

	std::map<DataPath *, int> candidateDestWithOpcode = checkOpcodeSimilarity(candidateDests,node);

	if((candidateDestWithConst.size() != candidateDests.size()) or (candidateDestWithOpcode.size() != candidateDests.size()) )
	{
		candidateDests = removeConstFilledCandidates(candidateDests,candidateDestWithConst,candidateDestWithOpcode);
	}
#endif

	int minLatSucc = 1000000000;
	std::priority_queue<dest_with_cost> estimatedRoutesTemp;

	int allowed_time_steps_for_connection = 5;
	int iterations = allowed_time_steps_for_connection;

	//Route Estimation
	for (int i = 0; i < iterations; ++i)
	{
		bool pathFromParentExist = false;
		bool pathExistMappedChild = false;

		DataPath * dest;


		std::vector<CGRAXMLCompile::DataPath *> candidateDestsFinal = candidateDests;

		for (DataPath *dest : candidateDestsFinal)
		{

			int minLatDestVal_prime = minLatDests[dest] + ii * i;

			PE * prevPE=this->cgra->getPrevPE(dest->getPE());

			DataPath * prevDP;

			for (Module *submod : prevPE->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(submod))
				{
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{
							prevDP = dp;
						}
					}
				}

			}

			std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
			pathFromParentExist = true;
			std::vector<LatPort> path;
			std::vector<LatPort> pathMSB;

			for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
			{

				bool isMultiCycle=false;

				int minLatDestVal = minLatDestVal_prime;
				DFGNode *parent = pair.first;


				if (parent->getOPtype(node) == "PS")
					continue;
				//select the suitable input port destination according to the destination mentioned in the parent node in the xml
				Port *destPort;
				Port *destPortR;

				Port *destPortMSB;

				bool isMSB=false;

#ifdef DUAL_STREAMING
				string opType = parent->getOPtype(node);
				string opParent = parent->op;
				string opNode = node->op;

				string opType1 = "_MSB";
				string opType2 = "_LSB";
				bool MSBLogic = ((opType == "P") or (opParent == "LOADB") or ((opParent == "MOVC") && parent->hasConst && (parent->constant < 128)) or ((opNode == "STOREB") && (opType == "I1")));
				opType1 = opType + opType1;
				opType2 = opType + opType2;
				if(!MSBLogic){
					isMSB =true;
				}else{
					isMSB =false;
				}


#else
				string opType = parent->getOPtype(node);
				destPort = dest->getInPort(opType);
#endif

				minLatDestVal = minLatDestVal_prime + parent->childNextIter[node] * ii;

				std::priority_queue<cand_src_with_cost> res;


				for (Port *startCand : pair.second)
				{

					Port *startCandMSB;
#ifdef DUAL_STREAMING

					if((startCand->getName()).find("_LSB") != std::string::npos)
					{
						if(!MSBLogic)
						{
							if(((startCand->getPE()->T) != (dest->getPE()->T)) && (prevDP->getMappedNode() == NULL))
								destPortR = prevDP->getInPort(opType2);
							else
								destPortR = dest->getInPort(opType2);

							destPort = dest->getInPort(opType2);

							if(isMSB){
								destPortMSB = dest->getInPort(opType1);
								startCandMSB = parent->rootDP->getOutputDP()->getOutPort("T_MSB");
							}
						}
						else if(opType != "P"){
							destPort = dest->getInPort(opType2);
							destPortR = dest->getInPort(opType2);
						}
						else
						{
							destPort = dest->getInPort(opType);
							destPortR = dest->getInPort(opType);
						}
					}

					else
					{

						destPort = dest->getInPort(opType);
						destPortR = dest->getInPort(opType);
					}
#endif

					int cost;

					std::map<Port *, std::set<DFGNode *>> mutexPaths;
					if (detailedDebug)
						std::cout << "par Estimating Path" << startCand->getFullName() << "," << startCand->getLat() << ","
								  << "--->" << destPort->getFullName() << "," << minLatDestVal << "," << ",parent_node = " << parent->idx
								  << "\n";

					LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
					assert(startCand->getLat() != -1);

					LatPort destPortLatR ;
					LatPort destPortLat = std::make_pair(minLatDestVal, destPort);
#ifdef DUAL_STREAMING

					destPortLatR = std::make_pair(minLatDestVal-1, destPortR);
#endif
					LatPort destPortLatMSB;
					LatPort startCandLatMSB;
#ifdef DUAL_STREAMING
					if(isMSB){
						destPortLatMSB = std::make_pair(minLatDestVal, destPortMSB);
						assert((minLatDestVal) % destPortMSB->getMod()->getCGRA()->get_t_max() == destPortMSB->getMod()->getPE()->T);
						startCandLatMSB = std::make_pair(startCandMSB->getLat(), startCandMSB);
						assert(startCandMSB->getLat() != -1);
					}
#endif

					assert((minLatDestVal) % destPort->getMod()->getCGRA()->get_t_max() == destPort->getMod()->getPE()->T);

					bool pathExist = false;
					{
						FU *parentFU = dest->getFU();
						assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
						int latency = parentFU->supportedOPs[node->op];

//#ifdef DUAL_STREAMING


#ifdef DUAL_STREAMING
						destPort = dest->getOutputPort(latency,false);
						destPortR = prevDP->getOutputPort(latency,false);
						if(isMSB)
							destPortMSB = dest->getOutputPort(latency,true);
#else
						destPort = dest->getOutputPort(latency);
#endif

						LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);
						LatPort	destPortLatR;
#ifdef DUAL_STREAMING
						if(destPort !=destPortR )
							destPortLatR = std::make_pair(minLatDestVal + latency, destPort);

						if(isMSB){
							LatPort destPortLatMSB = std::make_pair(minLatDestVal + latency, destPortMSB);

							if (canExitCurrPE(destPortLatMSB))
							{
								pathExist = true;
							}
							else
							{
								std::cout << "Cannot exit from :" << destPortLatMSB.second->getFullName() << "\n";
							}
						}
#endif

						if (canExitCurrPE(destPortLat))
						{
							pathExist = true;
						}
						else
						{
							std::cout << "Cannot exit from :" << destPortLat.second->getFullName() << "\n";
						}
					}
					int LSB_cost =0;
					bool data_avai_lsb=false;
#ifdef DUAL_STREAMING
					CGRA * cgra = destPortLatR.second->getPE()->getCGRA();
					PE * prev_p;
					if((destPortLat.second != destPortLatR.second) && isMSB){
						prev_p = cgra->getPrevPE(destPortLatR.second->getPE());
					}else
					{
						prev_p = cgra->getPrevPE(destPortLat.second->getPE());
					}

					FU* fu = static_cast<FU*>(prev_p->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

					if(dp->getInPort(destPortLat.second->getName())->getNode() == parent)
					{
						data_avai_lsb=true;
						pathExist = pathExist & true;
					}

					if(!data_avai_lsb){

					if((destPortLat.second != destPortLatR.second) && isMSB){
						pathExist = pathExist & LeastCostPathAstar(startCandLat, destPortLatR, prevDP, path, pathMSB,cost, parent, mutexPaths, node, isMultiCycle,true);
						if(!pathExist)
							pathExist = pathExist & LeastCostPathAstar(startCandLat, destPortLat, dest, path, pathMSB,cost, parent, mutexPaths, node, isMultiCycle,true);
						else
							destPortLat = destPortLatR;
					}else{
						pathExist = pathExist & LeastCostPathAstar(startCandLat, destPortLat, dest, path, pathMSB,cost, parent, mutexPaths, node, isMultiCycle,true);

					}

					if(!path.empty())
						pathMSB.insert(pathMSB.end(),path.begin(),path.end());


					LSB_cost =cost;
					}
					bool data_avai_msb=false;
					int MSB_cost =0;
					if(isMSB){
					PE * prev_p_msb;

					prev_p_msb = cgra->getPrevPE(destPortLatMSB.second->getPE());
					fu = static_cast<FU*>(prev_p_msb->getSubMod("FU0")); assert(fu);
					dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					if(dp->getInPort(destPortLatMSB.second->getName())->getNode() == parent)
					{
						data_avai_msb=true;
						pathExist = pathExist & true;
					}
					}

					if(MSBLogic && ((startCandLat.second->getPE()->X) == (destPortLat.second->getPE()->X)) && ((startCandLat.second->getPE()->Y) == (destPortLat.second->getPE()->Y)))
						LSB_cost += SAME_PE_COST;
					if(isMSB && !data_avai_msb){


						pathExist = pathExist & LeastCostPathAstar(startCandLatMSB, destPortLatMSB, dest, path, pathMSB, cost, parent, mutexPaths, node, isMultiCycle,true);
						if(!path.empty())
							pathMSB.insert(pathMSB.end(),path.begin(),path.end());

						MSB_cost =cost;
						path.clear();
					}


					cost = MSB_cost + LSB_cost;
#else
					pathExist = pathExist & LeastCostPathAstar(startCandLat, destPortLat, dest, path, cost, parent, mutexPaths, node);
					path.clear();

#endif
					if (!pathExist)
					{
						if (detailedDebug)
							std::cout << "par Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
						continue;
					}
					cost += dpPenaltyMap[dest];

//---- THILINI:: Adding crossbar and opcode
#ifdef OPCODE_CONST_CONFIG
					cost += candidateDestWithConst[dest];
					cost += candidateDestWithOpcode[dest];
#endif
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
			{;
				continue;
			}


			pathExistMappedChild = true;
			std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
			for (std::pair<DFGNode *, Port *> pair : alreadyMappedChildPorts)
			{
				bool isMultiCycle=false;

				int minLatDestVal = minLatDestVal_prime;
				DFGNode *child = pair.first;
				Port *childDestPort = pair.second;
				DataPath* childDP = child->rootDP;

				bool isMSB=false;
				Port *destPort;
				Port *destPortMSB;
				Port *childDestPortMSB;
#ifndef DUAL_STREAMING
				string opType = node->childrenOPType[child];
				string opType2 = "";
				if(!(opType == "P"))
					opType = opType + opType2;
#else
				string opType = node->childrenOPType[child];
				string opParent = node->op;
				string opChild = child->op;
				string opType1 = "_MSB";
				string opType2 = "_LSB";
				bool MSBLogic = (opType == "P") or (opParent == "LOADB") or ((opParent == "MOVC") && node->hasConst && (node->constant < 128)) or ((opChild == "STOREB") && (opType == "I1"));
				if(!MSBLogic){
					opType1 = opType + opType1;
					opType2 = opType + opType2;
					isMSB =true;
				}else{
					isMSB =false;
				}
#endif
				//THILI::: Check this
				if (child->idx == node->idx)
				{
					childDestPort = dest->getInPort(opType);
					if (detailedDebug) cout << "setting latency = " << minLatDestVal + ii << "\n";
					childDestPort->setLat(minLatDestVal + ii);
					childDP = dest;
				}

				std::vector<LatPort> path;
				std::vector<LatPort> pathMSB;
				int cost;

				FU *parentFU = dest->getFU();
				assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
				int latency = parentFU->supportedOPs[node->op];

				Port *childDestPortR = childDestPort;
				PE * prevPE=this->cgra->getPrevPE(childDestPort->getPE());
				DataPath * prevDP;

				for (Module *submod : prevPE->subModules)
				{
					if (FU *fu = dynamic_cast<FU *>(submod))
					{
						for (Module *submodFU : fu->subModules)
						{
							if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
							{
								prevDP = dp;
							}
						}
					}

				}
				if(isMSB && (childDestPort->getPE()->T != dest->getPE()->T)&& (prevDP->getMappedNode() == NULL))
				{
					childDestPortR = prevDP->getInPort(childDestPort->getName());
				}

#ifdef DUAL_STREAMING

					if((childDestPort->getName()).find("_LSB") != std::string::npos)
					{
							destPort = dest->getOutputPort(latency,false);
							if(isMSB){
								destPortMSB = dest->getOutputPort(latency,true);
								childDestPortMSB = child->rootDP->getInPort(opType + "_MSB");
							}
					}

					else
						destPort = dest->getOutputPort(latency,false);

#else
					destPort = dest->getOutputPort(latency,false);
#endif

				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				if (detailedDebug)
					std::cout << "already child Estimating Path" << destPort->getFullName() << "," << minLatDestVal + latency << ","
							  << "--->" << childDestPort->getFullName() << "," << childDestPort->getLat() << "," << "exist_child = " << child->idx  
							  << "\n";
				if (detailedDebug)
					std::cout << "lat = " << childDestPort->getLat() << ",PE=" << childDestPort->getMod()->getPE()->getName() << ",t=" << childDestPort->getMod()->getPE()->T << "\n";

				LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
				LatPort childDestPortLatR = std::make_pair(childDestPort->getLat()-1, childDestPortR);
				assert(childDestPort->getLat() != -1);

				LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);
				bool pathExistsMappedChildTemp = false;
				if(childDestPortLat != childDestPortLatR)
				{
					pathExistsMappedChildTemp = LeastCostPathAstar(destPortLat, childDestPortLatR, prevDP, path, pathMSB, cost, node, mutexPaths, child,isMultiCycle,false);

					if(!pathExistsMappedChildTemp)
						pathExistsMappedChildTemp = LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, pathMSB, cost, node, mutexPaths, child,isMultiCycle,false);
					else
						childDestPortLat = childDestPortLatR;
				}else{
					pathExistsMappedChildTemp = LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, pathMSB, cost, node, mutexPaths, child,isMultiCycle,false);
				}
				int cost_LSB = cost;

#ifdef DUAL_STREAMING
				if(MSBLogic && ((destPortLat.second->getPE()->X) == (childDestPortLat.second->getPE()->X)) && ((destPortLat.second->getPE()->Y) == (childDestPortLat.second->getPE()->Y)))
					cost_LSB += SAME_PE_COST;

				if(isMSB){
					LatPort childDestPortLatMSB = std::make_pair(childDestPort->getLat(), childDestPortMSB);

					assert(childDestPort->getLat() != -1);
					LatPort destPortLatMSB = std::make_pair(minLatDestVal + latency, destPortMSB);


					pathExistsMappedChildTemp = pathExistsMappedChildTemp & LeastCostPathAstar(destPortLatMSB, childDestPortLatMSB, childDP, pathMSB,path, cost, node, mutexPaths, child,isMultiCycle,false);
				}
				cost = cost + cost_LSB;
				path.clear();
#endif

				pathExistMappedChild = pathExistMappedChild & pathExistsMappedChildTemp;//LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, cost, node, mutexPaths, child);

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
					std::cout << "already child Estimating Path Failed!\n";
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

		if (pathFromParentExist & pathExistMappedChild)
			break;
	}


		int minBestCost = 10000000;
		int eq_count=0;
	while (!estimatedRoutesTemp.empty())
	{
		dest_with_cost top = estimatedRoutesTemp.top();
		estimatedRoutesTemp.pop();
		if (minLatDests[top.dest] == minLatSucc || !changed)
		{
			if(top.bestCost < minBestCost)
				minBestCost = top.bestCost;

			if(minBestCost == top.bestCost)
				eq_count ++;
			estimatedRoutes.push(top);
		}
		else
			std::cout << "Removed-> " << top.dest->getPE()->getFullName() << "\n";
	}


	if((eq_count > 1) && !estimatedRoutes.empty()){
		for(int i=0; i<eq_count;i++)
		{
			dest_with_cost top = estimatedRoutes.top();
			estimatedRoutes.pop();
			int secondary_cost =0;
			if(!possibleStarts.empty()){
				for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
				{
					std::vector <DFGNode *> siblings = pair.first->children;

					for(DFGNode * sib:siblings)
					{
						if((sib->rootDP != NULL) && (sib !=node))
						{
							int dx = abs(sib->rootDP->getPE()->X - top.dest->getPE()->X);
							int dy = abs(sib->rootDP->getPE()->Y - top.dest->getPE()->Y);
							secondary_cost +=(dx+dy);
						}else if((sib !=node)){

						std::vector <DFGNode *> sib_parents = sib->parents;

						for(DFGNode * sibp:sib_parents)
						{
							if((sib !=pair.first) && (sibp->rootDP != NULL))
							{
								int dx = abs(sibp->rootDP->getPE()->X - top.dest->getPE()->X);
								int dy = abs(sibp->rootDP->getPE()->Y - top.dest->getPE()->Y);
								secondary_cost +=(dx+dy);
							}
						}
						}
					}
					}
				}

				top.bestCost = top.bestCost + secondary_cost;
				estimatedRoutes.push(top);
			}
		}

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

		parentRoutingPortCount += thisParentNodeCount;
	}


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

		std::cout << "alreadyMappedChilds = " << currDest.alreadyMappedChilds.size() << "\n";
		std::cout << "alreadyMappedParents = " << currDest.parentStartLocs.size() << "\n";

		bool alreadMappedChildRouteSucc = true; //this will change to false if failure in alreadyMappedChilds
		std::map<DFGNode *, std::vector<LatPort>> mappedChildPaths;
		std::map<DFGNode *, std::map<Port *, std::set<DFGNode *>>> mappedChildMutexPaths;
		while (!currDest.alreadyMappedChilds.empty())
		{
			dest_child_with_cost dest_child_with_cost_ins = currDest.alreadyMappedChilds.top();
			currDest.alreadyMappedChilds.pop();
			std::cout << "alreadyMappedChilds :: Routing src-> " << dest_child_with_cost_ins.startPort.second->getPE()->getName() << " Routing dest-> " << dest_child_with_cost_ins.childDest.second->getPE()->getName()<< " mapping src-> " << currDest.dest->getPE()->getName() << " link type-> " << node->getOPtype(dest_child_with_cost_ins.child) << "\n";
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
			std::vector<LatPort> pathTmpMSB;
			for (LatPort p : possibleStarts)
			{
				bool isMultiCycle=false;
				int cost;

				if (LeastCostPathAstar(p, dest_child_with_cost_ins.childDest, dest_child_with_cost_ins.childDP, pathTmp,pathTmpMSB, cost, node, mutexPathsTmp, dest_child_with_cost_ins.child,isMultiCycle,false))
				{
					pathTmp.clear();
					q.push(cand_src_with_cost(p, dest_child_with_cost_ins.childDest, cost));
				}
			}

			int cost;
			std::vector<LatPort> path;
			std::vector<LatPort> path_MSB;
			LatPort src = dest_child_with_cost_ins.startPort;
			LatPort dest = dest_child_with_cost_ins.childDest;

			while (!q.empty())
			{
				cand_src_with_cost head = q.top();
				q.pop();
				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				bool isMultiCycle=false;

				bool isMSB=false;
				LatPort destMSB;
				LatPort srcMSB;
#ifdef DUAL_STREAMING
			string opType = node->childrenOPType[dest_child_with_cost_ins.child];
			string opParent = node->op;
			string opChild = dest_child_with_cost_ins.child->op;
			bool MSBLogic = (opType == "P") or (opParent == "LOADB") or ((opParent == "MOVC") && node->hasConst && (node->constant < 128)) or ((opChild == "STOREB") && (opType == "I1"));
			if(!MSBLogic && ((dest.second->getName()).find("_LSB") != std::string::npos)){
				isMSB = true;
				if(dest.second->getPE() == dest_child_with_cost_ins.childDP->getPE())
				{
					destMSB= std::make_pair(dest.first,dest_child_with_cost_ins.child->rootDP->getInPort(opType + "_MSB"));

				}else{
					destMSB= std::make_pair(dest.first+1,dest_child_with_cost_ins.child->rootDP->getInPort(opType + "_MSB"));

				}
				FU* fu = static_cast<FU*>(head.src.second->getPE()->getSubMod("FU0")); assert(fu);
				DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
				srcMSB = std::make_pair(head.src.first,dp->getOutPort("T_MSB"));
			}
#endif
			std::cout << "Serching path from -> "<< alreadMappedChildRouteSucc << head.src.second->getFullName() << " to " << dest.second->getFullName() << "\n";
				alreadMappedChildRouteSucc = LeastCostPathAstar(head.src, dest, dest_child_with_cost_ins.childDP, path, path_MSB, cost, node, mutexPaths, dest_child_with_cost_ins.child,isMultiCycle,false);
				int cost_LSB = cost;

#ifdef DUAL_STREAMING

				if(isMSB){
					std::cout << "Serching path from -> "<< alreadMappedChildRouteSucc << srcMSB.second->getFullName() << " to " << destMSB.second->getFullName() << "\n";
					alreadMappedChildRouteSucc = alreadMappedChildRouteSucc & LeastCostPathAstar(srcMSB, destMSB, dest_child_with_cost_ins.childDP, path_MSB,path, cost, node, mutexPaths, dest_child_with_cost_ins.child,isMultiCycle,false);
					cost = cost + cost_LSB;
				}

#endif
				if (alreadMappedChildRouteSucc)
				{
					assignPath(node, dest_child_with_cost_ins.child, path);
					std::cout << "src latency = " << head.src.first << " dest latency = " << dest.first << "\n";
#ifdef DUAL_STREAMING
					if(!path_MSB.empty()){
						assignPath(node, dest_child_with_cost_ins.child, path_MSB);

						std::cout << "src latency = " << srcMSB.first << " dest latency = " << destMSB.first << "\n";
						path.insert(path.end(), path_MSB.begin(), path_MSB.end());
					}
#endif

					mappedChildPaths[dest_child_with_cost_ins.child] = path;
					mappedChildMutexPaths[dest_child_with_cost_ins.child] = mutexPaths;
					std::cout << "Route success :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << " cost -> " << cost << "\n";
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
				bool isMultiCycle=false;

				std::cout << "alreadyMappedParents :: Routing src-> " << src.second->getPE()->getName() << " Routing dest-> " << dest.second->getPE()->getName()<< " mapping src-> " << currDest.dest->getPE()->getName() << " link type-> " << parent->getOPtype(node) << "\n";
				std::vector<LatPort> path;
				std::vector<LatPort> path_MSB;
				std::map<Port *, std::set<DFGNode *>> mutexPath;
				int cost;
				bool data_avai_lsb=false;

				PE * prev_p;
				int cost_LSB=0;
				prev_p = this->cgra->getPrevPE(dest.second->getPE());
				FU* fu = static_cast<FU*>(prev_p->getSubMod("FU0")); assert(fu);

				DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

				if(dp->getInPort(dest.second->getName())->getNode() == parent)
				{
						data_avai_lsb=true;
						succ = true;
				}
				std::vector<LatPort> additional_path;
				if(!data_avai_lsb){
				succ = LeastCostPathAstar(src, dest, currDest.dest, path, path_MSB, cost, parent, mutexPath, node, isMultiCycle,true);
				std::cout << "parent path size LSB= " << path.size() << "\n";
				std::cout << "cost LSB= " << cost << " Destination->" << currDest.dest->getPE()->getName()<< "\n";

				std::cout << "PAMU :mutex Paths->" << mutexPath.size() << "\n";


				if(dest.second->getPE() != currDest.dest->getPE())
				{
					FU * newFU = dest.second->getMod()->getNextTimeIns()->getFU();
					DataPath* dpn = static_cast<DataPath*>(newFU->getSubMod("DP0")); assert(dpn);
					Port* newP = dpn->getInPort(dest.second->getName());
					additional_path.push_back(std::make_pair(dest.first+1,newP));
					//additional_path.push_back(dest);
				}
					cost_LSB = cost;
				}else{

					additional_path.push_back(dest);
				}

				bool isMSB=false;
				LatPort destMSB;
				LatPort srcMSB;



#ifdef DUAL_STREAMING
			string opType = parent->getOPtype(node);;
			string opParent = parent->op;
			string opChild = node->op;
			bool MSBLogic = (opType == "P") or (opParent == "LOADB") or ((opParent == "MOVC") && parent->hasConst && (parent->constant < 128)) or ((opChild == "STOREB") && (opType == "I1"));


			bool data_avai_msb=false;
			if(!MSBLogic && ((dest.second->getName()).find("_LSB") != std::string::npos)){
			prev_p = this->cgra->getPrevPE(dest.second->getPE());
			fu = static_cast<FU*>(prev_p->getSubMod("FU0")); assert(fu);
			dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
			if(dp->getInPort(dest.second->getName())->getNode() == parent)
			{
					data_avai_msb=true;
					succ = succ & true;
			}
			}


			if(!MSBLogic && ((dest.second->getName()).find("_LSB") != std::string::npos) && !data_avai_msb){
				isMSB = true;

				if(dest.second->getPE() == currDest.dest->getPE())
				{
					destMSB= std::make_pair(dest.first,currDest.dest->getInPort(opType + "_MSB"));

				}else{
					destMSB= std::make_pair(dest.first+1,currDest.dest->getInPort(opType + "_MSB"));

				}
				srcMSB = std::make_pair(src.first,parent->rootDP->getOutputDP()->getOutPort("T_MSB"));
				succ =succ & LeastCostPathAstar(srcMSB, destMSB, currDest.dest, path_MSB, path, cost, parent, mutexPath, node, isMultiCycle,true);
				std::cout << "cost MSB= " << cost << " Destination->" << currDest.dest->getPE()->getName()<< "\n";
				cost = cost + cost_LSB;
				std::cout << "cost Total= " << cost << " Destination->" << currDest.dest->getPE()->getName()<< "\n";
				if((node->idx == 14) && (parent->idx == 0))
				{
					for(LatPort pp:path_MSB){
						std::cout << pp.second->getFullName() << "\n";
					}
				}
			}
			if(!MSBLogic && ((dest.second->getName()).find("_LSB") != std::string::npos)){
			if(data_avai_msb)
			{

				if(dest.second->getPE() == currDest.dest->getPE())
				{
					destMSB= std::make_pair(dest.first,currDest.dest->getInPort(opType + "_MSB"));

				}else{
					destMSB= std::make_pair(dest.first+1,currDest.dest->getInPort(opType + "_MSB"));

				}

				additional_path.push_back(destMSB);
			}
			}

#endif
				if (succ)
				{

					if(!path.empty()){
						assignPath(parent, node, path);
					}
					if(!additional_path.empty())
					{
						assignPath(parent, node,additional_path);
					}
					std::cout << "src latency = " << src.first << " dest latency = " << dest.first << "\n";
#ifdef DUAL_STREAMING
					if(isMSB & !path_MSB.empty())
					{
						if(!path.empty()){
						assignPath(parent, node, path_MSB);
						std::cout << "parent path size LSB= " << path_MSB.size() << "MSB =" << path.size() << "\n";

						std::cout << "src latency = " << srcMSB.first << " dest latency = " << destMSB.first << "\n";
						path.insert(path.end(), path_MSB.begin(),path_MSB.end());
						}
					}
#endif


					mappedParentMutexPaths[parent] = mutexPath;
					addedRoutingParentPorts += path.size();

					addedRoutingParentPorts -= 1;

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
			bool samePET=false;
			std::cout << "node=" << node->idx << ",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << ",lat=" << currDest.destLat << "\n";
			std::cout << "routing info ::\n";


			for (DFGNode *parent : node->parents)
			{
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

					}
					prev_lat = p->getLat();
				}
			}
			std::cout << "routing info done.\n";

			currDest.dest->assignNode(node, currDest.destLat, this->dfg);

			node->rootDP = currDest.dest;

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
		node->rootDP = currDest.dest;
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
													LatPort next_to_src, LatPort dest, bool isStreaming, bool isPtoC, std::vector<LatPort> regs)
{


	std::string next_to_srcName = next_to_src.second->getName();


	assert(src.second);
	assert(next_to_src.second);
	assert(dest.second);

	PE *srcPE = src.second->findParentPE();
	assert(srcPE);
	PE *nextPE = next_to_src.second->findParentPE();
	assert(nextPE);
	bool isRegConType1 = src.second->getName().find("REG_O") != std::string::npos &&
									 next_to_src.second->getName().find("REG_I") != std::string::npos;
	bool isRegConType2 = src.second->getName().find("_RO") != std::string::npos &&
									 next_to_src.second->getName().find("_RI") != std::string::npos;


	int distance;

	if(src.second->getPE()->T != dest.second->getPE()->T)
		distance = 0;
	else
		distance = regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());
/*#ifdef DUAL_STREAMING

		if (isRegConType1 || isRegConType2)
		{
			bool found = false;
			for(LatPort latPR:regs){
				if((src.second->getName() == latPR.second->getName()) && (src.second->getPE()->X == latPR.second->getPE()->X) && (src.second->getPE()->Y == latPR.second->getPE()->Y))
					found =true;
			}
			if(!found)
			{
				distance = distance*regs.size();
			}
		}

#endif*/

	distance = distance * PETransitionCostFactor + next_to_src.second->getCongCost() + PortTransitionCost;

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

#ifndef TORUS

		/*	CGRA *cgra = nextPE->getCGRA();
			if((nextPE->X==0 && nextPE->Y==0) or (nextPE->X==0 && nextPE->Y==(cgra->get_y_max()-1)) or (nextPE->X==(cgra->get_x_max()-1) && nextPE->Y==0) or (nextPE->X==(cgra->get_x_max()-1) && nextPE->Y==(cgra->get_y_max()-1)))
			{
				distance = distance + (4 - (freePorts)) * UOPCostFactor;
			}else if(nextPE->X==0 or nextPE->Y==0 or nextPE->Y==(cgra->get_y_max()-1) or nextPE->X==(cgra->get_x_max()-1))
			{
				distance = distance + (6 - (freePorts)) * UOPCostFactor;
			}else{*/
				distance = distance + (nextPE->outputPorts.size() * 2 - (freePorts)) * UOPCostFactor;
	//		}
#else
			distance = distance + (nextPE->outputPorts.size() * 2 - (freePorts)) * UOPCostFactor;
#endif


		if (nextPE->outputPorts.size() * 2 < freePorts)
		{
			std::cout << "outportsize = " << nextPE->outputPorts.size() << "\n";
			std::cout << "freePorts = " << freePorts << "\n";
		}
	}


	assert(distance > 0);

	if ((next_to_src.second->getName().compare("P") == 0) || (next_to_src.second->getName().compare("I1") == 0) || (next_to_src.second->getName().compare("I2") == 0))
	{

		FU *fu = next_to_src.second->getMod()->getFU();
		if ((fu->supportedOPs.find("LOAD") != fu->supportedOPs.end()) && (dest == next_to_src))
		{
			double memrescost_dbl = (double)this->dfg->unmappedMemOps / (double)cgra->freeMemNodes;
			memrescost_dbl = memrescost_dbl * (double)MEMResourceCost;
			distance = distance + (int)memrescost_dbl;
			if (this->dfg->unmappedMemOps == cgra->freeMemNodes)
			{
				distance = distance + MRC * 10;
			}
		}

	}

	assert(distance > 0);
	return distance;
}

bool CGRAXMLCompile::PathFinderMapper::Map(CGRA *cgra, DFG *dfg)
{
	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	int backTrackCredits = this->backTrackLimit;

	//Disable mutex paths to test pathfinder
	this->enableMutexPaths = true;

	this->cgra = cgra;
	this->dfg = dfg;

	Check_DFG_CGRA_Compatibility();

	//if(cgra->is_spm_modelled){
	//	UpdateVariableBaseAddr();
	//}
	//Testing 1 2 3
	//getLongestDFGPath(dfg->findNode(1093),dfg->findNode(82));

	//	SortSCCDFG();
	//	SortTopoGraphicalDFG();
	sortBackEdgePriorityASAP();
	//sortBackEdgePriorityASAPALP();
		//sortBackEdgePriorityALAP();

		//std::cout << "THILINI :: outside\n";
		//sortBackEdgePriorityFaninFanoutMax();
		//std::cout << "THILINI :: outside2\n";
	//sortBackEdgePriorityASAP();

	std::string mappingLogFileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths);  // + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths); // + ".routeInfo.log";

	bool mapSuccess = false;

	//std::string congestionInfoFileName = mappingLogFileName + ".congestion.info";
	//cout << "Opening congestion file : " << congestionInfoFileName << "!\n";
	//congestionInfoFile.open(congestionInfoFileName.c_str());
	//assert(congestionInfoFile.is_open());

	for (int i = 0; i < this->maxIter; ++i)
	{

		//std::string mappingLogFileName_withIter = mappingLogFileName + "_Iter=" + std::to_string(i) + ".mapping.csv";
		//std::string mappingLog2FileName_withIter = mappingLog2FileName + "_Iter=" + std::to_string(i) + ".routeInfo.log";
		//std::string usedPFileName = "usedPorts.log";

		//mappingLog.open(mappingLogFileName_withIter.c_str());
		//mappingLog2.open(mappingLog2FileName_withIter.c_str());
		//usedPLog.open(usedPFileName.c_str());

		//cout << "Opening mapping csv file : " << mappingLogFileName_withIter << "\n";
		//cout << "Opening routeInfo log file : " << mappingLog2FileName_withIter << "\n";

		//assert(mappingLog.is_open());
		//assert(mappingLog2.is_open());

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

		std::cout << "MAP begin...\n";

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
			//mappingLog << MapHeader.str();

			bool isEstRouteSucc = false;

			//fill the routing information
			if (estimatedRouteInfo.find(node) == estimatedRouteInfo.end())
			{
				//the routes are not estimated.
				std::priority_queue<dest_with_cost> estimatedRoutes;
				DFGNode *failedNode;
				isEstRouteSucc = estimateRouting(node, estimatedRoutes, &failedNode);
				std::cout << "IS route success : " << isEstRouteSucc << "\n";
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
							//mappingLog << "route estimation failed...\n";
							//mappingLog << "Map Failed!.\n";

							//mappingLog.close();
							//mappingLog2.close();
							return false;
						}
						backTrackCredits--;


						DFGNode *prevNode = mappedNodes.top();
						mappedNodes.pop();
						unmappedNodes.push(node);
						unmappedNodes.push(prevNode);

						prevNode->clear(this->dfg);
						estimatedRouteInfo.erase(node);


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
						//mappingLog << "Map Failed!.\n";
						//mappingLog.close();
						//mappingLog2.close();
						return false;
					}
				}
				estimatedRouteInfo[node] = estimatedRoutes;
			}

			bool isRouteSucc = false;
			DFGNode *failedNode = NULL;

			std::cout << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			//mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
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
					//mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					//mappingLog.close();
					//mappingLog2.close();
					return false;
				}
			}

			if (!isRouteSucc)
			{
				this->printMappingLog();
				this->printMappingLog2();
				if (mappedNodes.empty())
				{
					//mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					//mappingLog.close();
					//mappingLog2.close();
					return false;
				}

				if (enableBackTracking)
				{
					if (backTrackCredits == 0)
					{
						//mappingLog << "Map Failed!.\n";
						std::cout << "Map Failed!.\n";
						//mappingLog.close();
						//mappingLog2.close();
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
					//mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					//mappingLog.close();
					//mappingLog2.close();
					return false;
				}
			}

			backTrackCredits = std::min(this->backTrackLimit, backTrackCredits + 1);
			mappedNodes.push(node);
		}
		mapSuccess = updateCongestionCosts(i);

		if(!this->cgra->usedPorts.empty()){

		}
		if (mapSuccess)
		{
			break;
		}

		clearCurrMapping();

		estimatedRouteInfo.clear();
		//mappingLog.close();
		//mappingLog2.close();
	}



	if (mapSuccess)
	{
		//mappingLog << "Map Success!.\n";
		//mappingLog2 << "Map Success!.\n";
		this->printMappingLog();
		this->printMappingLog2();

		//cgra->PrintMappingForPillars(fNameLog1 + cgra->getCGRAName() + "mapping_i.txt", fNameLog1 + cgra->getCGRAName() + "mapping_r.txt");

		std::cout << "Map Success!.\n";
		//mappingLog.close();
		//mappingLog2.close();

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
		//mappingLog << "Map Failed!.\n";
		std::cout << "Map Failed!.\n";
		//mappingLog.close();
		//mappingLog2.close();
		return false;
	}
}

void CGRAXMLCompile::PathFinderMapper::assignPath(DFGNode *src, DFGNode *dest,
												  std::vector<LatPort> path)
{

	std::cout << "assigning path from:" << src->idx << " to:" << dest->idx << "\n";

	int srcPortCount = 0;

	int prevLat = -1;
	LatPort prevPort;
	bool isMSB=false;
	if(path.at(0).second->getName().find("_MSB") != std::string::npos)
		isMSB=true;

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

		if(prevPort.second != NULL)
		{
			if((prevPort.second->getName() == "EAST_O") or (prevPort.second->getName() == "WEST_O") or (prevPort.second->getName() == "SOUTH_O") or (prevPort.second->getName() == "NORTH_O") or (prevPort.second->getName() == "COL_O1") or (prevPort.second->getName() == "COL_O2") or (prevPort.second->getName() == "COL_O3") or (prevPort.second->getName() == "COL_O4") or (prevPort.second->getName() == "ROW_O1") or (prevPort.second->getName() == "ROW_O2") or (prevPort.second->getName() == "ROW_O3") or (prevPort.second->getName() == "ROW_O4")){
				std::pair <Port * ,Port *> portPair = std::make_pair(prevPort.second,p.second);
				if(std::find(this->cgra->UsedLinks.begin(),this->cgra->UsedLinks.end(),portPair) == this->cgra->UsedLinks.end())
					this->cgra->UsedLinks.push_back(portPair);
			}
		}


		prevLat = p.first;
		prevPort = p;

		if (p.second->getNode() == src)
		{
			srcPortCount++;
			continue;
		}

		p.second->setNode(src, p.first, this);
		if(isMSB)
		{
			p.second->setMSB(true);
			p.second->setLSB(false);
		}
		else
		{
			p.second->setMSB(false);
			p.second->setLSB(true);
		}

		std::set<DFGNode *> commonNodes = congestedPorts[p.second];
		bool is_shared=false;
		if(commonNodes.find(src) != commonNodes.end())
		{
			is_shared=true;
		}
		if(!is_shared){
		congestedPorts[p.second].insert(src);
		p.second->increaseConflictedUse(src, this);
		}

		if (std::find(src->routingPorts.begin(), src->routingPorts.end(), std::make_pair(p.second, dest->idx)) == src->routingPorts.end())
		{
			if (std::find(src->routingPorts.begin(), src->routingPorts.end(), std::make_pair(p.second, src->idx)) == src->routingPorts.end())
			{
				src->routingPorts.push_back(std::make_pair(p.second, dest->idx));
				std::cout << p.second->getFullName() << "\n";
			}
			else
			{
				std::cout << p.second->getFullName() << "\n";
				assert(p.second->getName().compare("T") == 0);
			}
		}

	}
	std::cout << "srcPortCount = " << srcPortCount << "\n";
}

bool CGRAXMLCompile::PathFinderMapper::updateCongestionCosts(int iter)
{
	bool noCongestion = true;

	std::set<int> conflictedTimeSteps;

	//congestionInfoFile << "**********************************\n";
	//congestionInfoFile << "II = " << this->cgra->get_t_max() << ",iter = " << iter << "\n";
	//congestionInfoFile << "**********************************\n";

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
					//congestionInfoFile << "CONGESTION:" << p->getFullName();
					for (DFGNode *node : pair.second)
					{
						std::cout << "," << node->idx << "|BB=" << node->BB;
						//congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
					}
					std::cout << "\n";
					//congestionInfoFile << "\n";
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
			//congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
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

				noConflicts = false;
			}

			if (noConflicts)
				continue;

			std::cout << "CONFLICT :" << p->getFullName();
			//congestionInfoFile << "CONFLICT :" << p->getFullName();
			for (DFGNode *node : pair.second)
			{
				std::cout << "," << node->idx << "|BB=" << node->BB;
				//congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
			}
			std::cout << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			std::cout << "\n";

			//congestionInfoFile << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			//congestionInfoFile << "\n";

			for (int i = 0; i < pair.second.size(); ++i)
			{
				p->increastCongCost();
			}
			conflictedTimeSteps.insert(p->getMod()->getPE()->T);
		}

		if (p->getHistoryCost() > 0)
		{
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
			//congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
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

		//congestionInfoFile << "****************************************\n";
		//congestionInfoFile << "Upperbound II = " << this->upperboundII << "\n";
		//congestionInfoFile << "On iter = " << iter << "\n";
		//congestionInfoFile << "****************************************\n";
	}

	//congestionInfoFile << std::endl;

	congestedPorts.clear();
	conflictedPorts.clear();
	conflictedTimeStepMap.clear();
	if (noCongestion)
		std::cout << "noCongestion!\n";
	if (noConflicts)
		std::cout << "noConflicts!\n";

	if (noCongestion)
		//congestionInfoFile << "noCongestion!\n";
	if (noConflicts)
		//congestionInfoFile << "noConflicts!\n";

	return noCongestion & noConflicts;
}

bool CGRAXMLCompile::PathFinderMapper::clearCurrMapping()
{
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << node->idx << "\n";
		std::cout << "CLEARING :: node=" << node->idx << ",destDP=" << node->rootDP->getName() << ",destPE=" << node->rootDP->getPE()->getName() << "\n";
		node->clear(this->dfg);
	}
	//std::cout << "HERE\n";
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

		for (PE *currPE : PEList)
		{


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

	};

	std::set<BEDist> backedges;

	for (DFGNode &node : dfg->nodeList)
	{

		if (node.idx == 97)
		{
			//std::cout << "node_idx:97,node_ASAP:" << node.ASAP << "\n";
		}
		for (DFGNode *child : node.children)
		{

			if (node.idx == 97)
			{
				//std::cout << "child_idx:" << child->idx << "child_ASAP:" << child->ASAP << "\n";
			}

			if (child->ASAP <= node.ASAP)
			{
				//std::cout << "inserting for : node=" << node.idx << ",child:" << child->idx << "\n";
				backedges.insert(BEDist(&node, child, node.ASAP - child->ASAP));
			}
		}
	}

	//populate reccycles
	std::cout << "Populate Rec Cycles!\n";
	RecCycles.clear();
	//exit(0);
	for (BEDist be : backedges)
	{
		//		std::set<DFGNode*> backedgePath;
		std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be.parent);

		for (DFGNode *n : backedgePathVec)
		{
			if (RecCycles[BackEdge(be.parent, be.child)].find(n) == RecCycles[BackEdge(be.parent, be.child)].end())
			{
				//std::cout << n->idx << ",";
			}
			RecCycles[BackEdge(be.parent, be.child)].insert(n);
		}

	}

	RecCyclesLS.clear();
	for (DFGNode &node : dfg->nodeList)
	{
		for (DFGNode *recParent : node.recParents)
		{
			BEDist be_temp(&node, recParent, node.ASAP - recParent->ASAP);

			std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be_temp.parent);

			//std::cout << "REC_CYCLELS :: BE_Parent = " << be_temp.parent->idx << "\n";
			//std::cout << "REC_CYCLELS :: BE_Child = " << be_temp.child->idx << "\n";
			//std::cout << "REC_CYCLELS :: BE_Parent's ancesotry : \n";
			for (DFGNode *n : backedgePathVec)
			{
				if (n == be_temp.parent)
					continue;
				//if (RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].find(n) == RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].end())
				//{
				//	std::cout << n->idx << ",";
				//}
				RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].insert(n);
			}
			//std::cout << "REC_CYCLELS :: Done!\n";

			backedges.insert(be_temp);
		}
	}

	std::map<DFGNode *, std::vector<DFGNode *>> beparentAncestors;
	std::map<DFGNode *, std::vector<DFGNode *>> bechildAncestors;
	//	std::map<DFGNode*,std::vector<DFGNode*>> bechildAncestors;
	std::map<std::pair<DFGNode *, DFGNode *>, bool> trueBackedges;

	for (BEDist be : backedges)
	{
		//std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		//std::cout << "BE CHILD = " << be.child->idx << "\n";

		//std::cout << "Ancestory : "
		//		  << "\n";
		beparentAncestors[be.parent] = dfg->getAncestoryASAP(be.parent);
		bechildAncestors[be.child] = dfg->getAncestoryASAP(be.child);
		//std::cout << "\n";

		if (std::find(beparentAncestors[be.parent].begin(),
					  beparentAncestors[be.parent].end(),
					  be.child) == beparentAncestors[be.parent].end())
		{
			//std::cout << "BE CHILD does not belong BE Parent's Ancestory\n";

			//Hack to force all backedges to be true backedges
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
		}
		else
		{
			//change this be.parent if PHI nodes are not removed
			//std::cout << "RecPHI inserted : " << be.child->idx << "\n";
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
			//			RecPHIs.insert(be.child);
		}

		//		bechildAncestors[be.child]=dfg->getAncestory(be.child);
	}



	std::map<DFGNode *, std::set<DFGNode *>> superiorChildren;

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
			//			if(trueBackedges[std::make_pair(be.parent,be.child)] == false) continue;
			if (std::find(mergedAncestories[key].begin(), mergedAncestories[key].end(), be.child) != mergedAncestories[key].end())
			{

				if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
				{
					superiorChildren[key].insert(be.child);
				}

				//std::cout << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedAncestories[key] = dfg->mergeAncestoryASAP(mergedAncestories[key], beparentAncestors[be.parent], RecCycles);
				merged = true;
				//std::cout << "Merging Done :: " << key->idx << ", " << be.parent->idx << "\n";
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
		//std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
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
		//std::cout << "BE CHILD = " << be.child->idx << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.child) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.child);
		}
	}

	std::cout << sortedNodeList.size() << "\n";

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
			}
		}
	}

	/*std::cout << "***********SORTED LIST*******************\n";
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "Node=" << node->idx << ",ASAP=" << node->ASAP << "\n";
	}*/
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

void CGRAXMLCompile::PathFinderMapper::sortBackEdgePriorityFaninFanoutMax()
{

	sortedNodeList.clear();


	std::map<int,std::vector<DFGNode *>> faninFanoutMaxNodeList;
	std::map< DFGNode *,int> fanNodes;
	std::priority_queue<std::pair<int,DFGNode *>> q;

	for (DFGNode &node : dfg->nodeList)
	{
		int totFaninFanout = node.children.size() + node.parents.size();
		faninFanoutMaxNodeList[totFaninFanout].push_back(&node);
		fanNodes[&node] = totFaninFanout;
	}
	int maxFanlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : faninFanoutMaxNodeList)
	{
		if (pair.first > maxFanlevel)
		{
			maxFanlevel = pair.first;
		}
	}
	std::vector<DFGNode *> maxNodes = faninFanoutMaxNodeList[maxFanlevel];
	assert(!maxNodes.empty());

	for(DFGNode* node:maxNodes){
		q.push(std::make_pair(maxFanlevel,node));
	}

	while(!q.empty())
	{
		std::pair<int,DFGNode *> currPair =  q.top();
		q.pop();

		if(std::find(sortedNodeList.begin(), sortedNodeList.end(),currPair.second) == sortedNodeList.end())
		{
			std::cout << currPair.second->idx << "\n";
			sortedNodeList.push_back(currPair.second);
		}

		std::vector <DFGNode *> childNodes= currPair.second->children;
		std::vector <DFGNode *> parentNodes= currPair.second->parents;

		for(DFGNode* node:childNodes){

			if(std::find(sortedNodeList.begin(), sortedNodeList.end(), node) == sortedNodeList.end())
				q.push(std::make_pair(fanNodes[node],node));
		}

		for(DFGNode* node:parentNodes){

			if(std::find(sortedNodeList.begin(), sortedNodeList.end(), node) == sortedNodeList.end())
				q.push(std::make_pair(fanNodes[node],node));
		}
	}

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

	/*std::cout << "***********SORTED LIST*******************\n";
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "Node=" << node->idx << ",ALAP=" << node->ALAP << "\n";
	}*/
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
	for (std::pair<DFGNode *, int> pair : minLat)
	{
		if (max < pair.second)
		{
			max = pair.second;
		}
		std::cout << "getlatMinStartsPHI :: minLat = " << max << "\n";
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



int CGRAXMLCompile::PathFinderMapper::getMaxLatencyBE(DFGNode *node, std::map<DataPath *, beParentInfo> &beParentDests, int &downSteamOps)
{

	std::set<BackEdge> setBackEdges;


	for (std::pair<BackEdge, std::set<DFGNode *>> pair : RecCycles)
	{
		BackEdge be = pair.first;
		std::set<DFGNode *> rec_nodes = pair.second;
		if (rec_nodes.find(node) != rec_nodes.end())
		{
			if (be.second->rootDP != NULL)
			{
				std::cout << "RecSet(" << be.first->idx << "," << be.second->idx << ")"
						  << " : ";
				for (DFGNode *n : rec_nodes)
				{
					std::cout << n->idx << ",";
				}
				std::cout << "\n";
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
				std::cout << "RecSet(" << be.first->idx << "," << be.second->idx << ")"
						  << " : ";
				for (DFGNode *n : rec_nodes)
				{
					std::cout << n->idx << ",";
				}
				std::cout << "\n";
				setBackEdges.insert(pair.first);
			}
		}
	}


	std::unordered_map<std::string, int> OpLatency = cgra->getGlobalOPMinLatencyMap();


	int maxLat = LARGE_VALUE;

	for (BackEdge be : setBackEdges)
	{
		int maxLatency = be.second->rootDP->getLat() + cgra->get_t_max();
		int noDownStreamOps = 0;

		std::map<int, std::set<DFGNode *>> asapOrder;
		std::map<int, int> asapMaxOpLat;
		std::map<int, int> asapMaxLat;

		if (RecCycles.find(be) != RecCycles.end())
		{

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
			std::cout << "ops : ";
			for (DFGNode *n : pair.second)
			{
				std::cout << "idx=" << n->idx << "[" << n->op << "]"
						  << "(" << OpLatency[n->op] << ")"
						  << ",";
				int new_lat = OpLatency[n->op];
				if (new_lat > maxOplatency)
					maxOplatency = new_lat;
			}
			std::cout << "\n";
			std::cout << "ASAP=" << pair.first << ",OPLAT=" << maxOplatency << "\n";

			if ((bpi.dsMEMfound == false) && (node->ASAP < pair.first))
			{
				if (maxOplatency == 2)
				{
					std::cout << "MEM FOUND SET TRUE!\n";
					bpi.dsMEMfound = true;
					bpi.uptoMEMops = upstreamOPs;
				}
			}

			if (node->ASAP < pair.first)
			{
				upstreamOPs++;
			}

			asapMaxOpLat[pair.first] = maxOplatency;
		}

		std::map<int, std::set<DFGNode *>>::reverse_iterator rit = asapOrder.rbegin();

		int prevLat = maxLatency;
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

			std::cout << "Adding Pseudo Connection :: parent=" << latestCousin->idx << ",to" << node->idx << "\n";
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


	std::unordered_map<std::string, int> oplatencyMap = cgra->getGlobalOPMinLatencyMap();


	q_init.insert(std::make_pair(src, oplatencyMap[src->op]));
	std::map<DFGNode *, std::map<int, DFGNode *>> cameFrom;
	q.push(q_init);

	while (!q.empty())
	{
		std::set<std::pair<DFGNode *, int>> curr = q.front();
		q.pop();
		std::set<std::pair<DFGNode *, int>> next;
		for (std::pair<DFGNode *, int> p1 : curr)
		{
			DFGNode *node = p1.first;
			std::cout << node->idx << ",";
			for (DFGNode *child : node->children)
			{
				if (node->childNextIter[child] == 1)
					continue;
				int nextLat = p1.second + oplatencyMap[child->op];
				next.insert(std::make_pair(child, nextLat));
				cameFrom[child][nextLat] = node;
			}
		}
		std::cout << "\n";
		if (!next.empty())
			q.push(next);
	}

	assert(cameFrom.find(dest) != cameFrom.end());

	DFGNode *temp = dest;
	while (temp != src)
	{
		std::cout << temp->idx << " <-- ";
		result.push_back(temp);
		temp = (*cameFrom[temp].rbegin()).second;
	}
	result.push_back(src);
	std::cout << "\n";
	//	assert(false);

	std::reverse(result.begin(), result.end());
	return result;
}

int CGRAXMLCompile::PathFinderMapper::getFreeMEMPeDist(PE *currPE)
{
	return currPE->T;
}

std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::modifyMaxLatCandDest(
	std::map<DataPath *, int> candDestIn, DFGNode *node, bool &changed)
{

	std::vector<DataPath *> res;

	std::map<DataPath *, beParentInfo> beParentDests;
	int downStreamOps = 0;
	int maxLat = getMaxLatencyBE(node, beParentDests, downStreamOps);

	if (maxLat != LARGE_VALUE)
		assert(!beParentDests.empty());

	changed = false;

	bool isMeMOp = checkMEMOp(node->op);


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
			for (std::pair<DataPath *, beParentInfo> pair : beParentDests)
			{
				//				if(pair.second.isLDST == false){
				PE *bePE = pair.first->getPE();

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
					std::cout << "**MEM FOUND DOWN**\n";
				}

				if (maxLat != LARGE_VALUE)
				{
				///	std::cout << "pe=" << pe->getName() << ",";
				//	std::cout << "dist=" << dist << ",";
				//	std::cout << "slack=" << pair.second.lat - maxLat << ",";
				//	std::cout << "downstreamOps=" << dsOps << "\n";
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
			//				std::cout << "pe=" << pe->getName() << ",";
			//				std::cout << "isMeMPE=" << pe->isMemPE << ",";
			//				std::cout << "Lat= " << pair.second << "\n";
			//				std::cout << "OK\n";
			res.push_back(pair.first);
		}
		else
		{
			changed = true;
		}
	}

	return res;
}

bool CGRAXMLCompile::PathFinderMapper::canExitCurrPE(LatPort p)
{

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
	// cout << "GetAllSupportedOPs :: currmod = " << currmod->getFullName() << "\n";

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

	GetAllSupportedOPs(cgra,all_supp_ops,all_supp_pointers);
	unordered_set<string> base_pointers;

	//cout << "all supported pointers : \n";
	for(string ptr : all_supp_pointers){
		cout << "\t" << ptr << "\n";
	}

	//cout << "all required pointers : \n";
	for(auto it = dfg->pointer_sizes.begin(); it != dfg->pointer_sizes.end(); it++){
		cout << "\t" << it->first << ",size = " << it->second << "\n";
	}

	for(DFGNode& node : dfg->nodeList){
		string op = node.op;
		if(all_supp_ops.find(op) == all_supp_ops.end()){
			cout << "op=" << op << " is not supported in this CGRA, exiting....\n";
			exit(EXIT_FAILURE);
			return false;
		}
	}

	if(cgra->is_spm_modelled){
		for(auto it = dfg->ldst_pointer_sizes.begin(); it != dfg->ldst_pointer_sizes.end(); it++){
			string pointer = it->first;
			if(all_supp_pointers.find(pointer) == all_supp_pointers.end()){
				cout << "pointer=" << pointer << " is not present in the CGRA, exiting....\n";
				exit(EXIT_FAILURE);
				return false;
			}
		}
	}
	else{
		//cout << "SPMs are not modelled, therefore ignoring supported pointers check.\n";
	}

	return true;
}

void CGRAXMLCompile::PathFinderMapper::UpdateVariableBaseAddr(){

	assert(cgra);
	assert(dfg);


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

	std::vector<std::vector<std::vector<InsFormat>>> InsFArr (cgra->get_t_max()+1,
			std::vector<std::vector<InsFormat>>(cgra->get_y_max(),
					std::vector<InsFormat>(cgra->get_x_max())
			)
	);
	for (int t = 0; t < cgra->get_t_max(); ++t) {
		vector<PE *> peList = this->cgra->getSpatialPEList(t);

				int iter=0;
				for (PE *pe : peList)
				{

					Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
					Port* easto = pe->getOutPort("EAST_O"); assert(easto);
					Port* westo = pe->getOutPort("WEST_O"); assert(westo);
					Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

					DFGNode* north_o_node = northo->getNode();
					DFGNode* east_o_node = easto->getNode();
					DFGNode* west_o_node = westo->getNode();
					DFGNode* south_o_node = southo->getNode();

					//RegFile* RFT = static_cast<RegFile*>(pe->getSubMod("RF0")); assert(RFT);
					FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

					DFGNode* currentMappedOP = dp->getMappedNode();
					//}

					int prev_t;
					int X;
					int Y;
					X = pe->X;
					Y = pe->Y;
					prev_t = (t + 2*cgra->get_t_max() - 1)%cgra->get_t_max();
					vector<PE *> prevPEList = this->cgra->getSpatialPEList(prev_t);

					PE* prevPE = prevPEList.at(iter);
					FU* prevFU = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevFU);
					DataPath* prevDP = static_cast<DataPath*>(prevFU->getSubMod("DP0"));
					DFGNode* mappedOP = prevDP->getMappedNode();


					iter++;

					Module* mod =  westo->getMod();
					DataPath *mod_dp = static_cast<DataPath *>(mod);


					Port* i1_ip = dp->getInPort("I1"); assert(i1_ip);
					Port* i2_ip = dp->getInPort("I2"); assert(i2_ip);
					Port* p_ip = dp->getInPort("P"); assert(p_ip);
					InsFormat insF;
					//PE* prevPE = pe;

					//XBar
					if(north_o_node){

						if(north_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.northo = "011";
						}
						else if(north_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.northo = "000";
						}
						else if(north_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.northo = "010";
						}
						else if(north_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.northo = "001";
						}
						else if(north_o_node == pe->getSingleRegPort("TREG_RI")->getNode() &&
								northo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							insF.northo = "101";
						}

						else if(north_o_node == fu->getOutPort("DP0_T")->getNode() && northo->getLat() == fu->getOutPort("DP0_T")->getLat()){
							insF.northo = "100";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
							PE* prevGotPE = this->cgra->getPrevPE(pe);
							FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

							Port* northoP = prevGotPE->getOutPort("NORTH_O"); assert(northoP);
							DFGNode* north_o_nodeP = northoP->getNode();
							//if(north_o_nodeP != NULL)
							//	std::cout << "Node-idx " << north_o_nodeP->idx << "\n";


							bool predi_condition= false;

							if(north_o_nodeP){
							std::vector<DFGNode *> parentNodes = north_o_nodeP->parents;


							for(DFGNode* node1:parentNodes)
							{
								if((node1->getOPtype(north_o_nodeP) == "P"))
								{
									predi_condition = true;
								}
							}
							}

							if(north_o_nodeP && (north_o_nodeP == north_o_node) && !predi_condition){
								if(north_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && northoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
									insF.northo = "101";
								}
								else if(north_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && northoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
									insF.northo = "100";
								}else{
									std::cout << "Port : " << northo->getFullName() << ",node = " << northo->getNode()->idx << ", source not found!\n";
									insF.northo = "111";
								}
							}

							//	assert(false);
#else
							std::cout << "Port : " << northo->getFullName() << ",node = " << northo->getNode()->idx << ", source not found!\n";
							insF.northo = "111";
							assert(false);
#endif
						}
					}


					else{
						insF.northo = "111";
					}

					if(east_o_node){
						//if(east_o_node != NULL)
						//		std::cout << "Node-idx_oute " << east_o_node->idx << "\n";
						if(east_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.easto = "011";
						}
						else if(east_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.easto = "000";
						}
						else if(east_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.easto = "010";
						}
						else if(east_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.easto = "001";
						}
						else if(east_o_node == pe->getSingleRegPort("TREG_RI")->getNode() &&
								easto->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							insF.easto = "101";
						}
						else if(east_o_node == fu->getOutPort("DP0_T")->getNode() && easto->getLat() == fu->getOutPort("DP0_T")->getLat()){
							insF.easto = "100";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
							PE* prevGotPE = this->cgra->getPrevPE(pe);
							FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

							Port* eastoP = prevGotPE->getOutPort("EAST_O"); assert(eastoP);
							DFGNode* east_o_nodeP = eastoP->getNode();
							//if(east_o_nodeP != NULL)
							//	std::cout << "Node-idx " << east_o_nodeP->idx << "\n";
							bool predi_condition= false;

							if(east_o_nodeP){
							std::vector<DFGNode *> parentNodes = east_o_nodeP->parents;


							for(DFGNode* node1:parentNodes)
							{
								if((node1->getOPtype(east_o_nodeP) == "P"))
								{
									predi_condition = true;
								}
							}
							}
							if(east_o_nodeP && (east_o_nodeP == east_o_node)  && !predi_condition){
								if(east_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && eastoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
									insF.easto = "101";
								}
								else if(east_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && eastoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
									insF.easto = "100";
								}else{
									std::cout << "Port : " << easto->getFullName() << ",node = " << easto->getNode()->idx << ", source not found!\n";
									insF.easto = "111";
								}
							}
#else
							std::cout << "Port : " << easto->getFullName() << ",node = " << easto->getNode()->idx << ", source not found!\n";
							insF.easto = "111";
							assert(false);
#endif
							//	assert(false);

						}
					}
					else{
						insF.easto = "111";
					}

					if(west_o_node){
						//if(west_o_node != NULL)
						//		std::cout << "Node-idx_outw " << west_o_node->idx << "\n";
						if(west_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.westo = "011";
						}
						else if(west_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.westo = "000";
						}
						else if(west_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.westo = "010";
						}
						else if(west_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.westo = "001";
						}
						else if(west_o_node ==  pe->getSingleRegPort("TREG_RI")->getNode() &&
								westo->getLat() ==  pe->getSingleRegPort("TREG_RI")->getLat()){
							insF.westo = "101";
						}

						else if(west_o_node == fu->getOutPort("DP0_T")->getNode() && westo->getLat() == fu->getOutPort("DP0_T")->getLat()){
							insF.westo = "100";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
								PE* prevGotPE = this->cgra->getPrevPE(pe);
								FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

								Port* westoP = prevGotPE->getOutPort("WEST_O"); assert(westoP);
								DFGNode* west_o_nodeP = westoP->getNode();
								//if(west_o_nodeP != NULL)
								//	std::cout << "Node-idx " << west_o_nodeP->idx << "\n";
								bool predi_condition= false;

								if(west_o_nodeP){
								std::vector<DFGNode *> parentNodes = west_o_nodeP->parents;


								for(DFGNode* node1:parentNodes)
								{
									if((node1->getOPtype(west_o_nodeP) == "P"))
									{
										predi_condition = true;
									}
								}
								}
								if(west_o_nodeP && (west_o_nodeP == west_o_node)  && !predi_condition){
									if(west_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && westoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
										insF.westo = "101";
									}
									else if(west_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && westoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
										insF.westo = "100";
									}else{
										std::cout << "Port : " << westo->getFullName() << ",node = " << westo->getNode()->idx << ", source not found!\n";
										insF.westo = "111";
									}
								}
#else
								std::cout << "Port : " << westo->getFullName() << ",node = " << westo->getNode()->idx << ", source not found!\n";
								insF.westo = "111";
								assert(false);
#endif
						}
					}
					else{
						insF.westo = "111";
					}

					if(south_o_node){
						//if(south_o_node != NULL)
						//		std::cout << "Node-idx_outs " << south_o_node->idx << "\n";
						if(south_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.southo = "011";
						}
						else if(south_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.southo = "000";
						}
						else if(south_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.southo = "010";
						}
						else if(south_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.southo = "001";
						}
						else if(south_o_node == pe->getSingleRegPort("TREG_RI")->getNode() &&
								southo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							insF.southo = "101";
						}
						else if(south_o_node == fu->getOutPort("DP0_T")->getNode() && southo->getLat() == fu->getOutPort("DP0_T")->getLat()){
							insF.southo = "100";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
							PE* prevGotPE = this->cgra->getPrevPE(pe);
							FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

							Port* southoP = prevGotPE->getOutPort("SOUTH_O"); assert(southoP);
							DFGNode* south_o_nodeP = southoP->getNode();
							//if(south_o_nodeP != NULL)
							//	std::cout << "Node-idx " << south_o_nodeP->idx << "\n";
							bool predi_condition= false;

							if(south_o_nodeP){
							std::vector<DFGNode *> parentNodes = south_o_nodeP->parents;


							for(DFGNode* node1:parentNodes)
							{
								if((node1->getOPtype(south_o_nodeP) == "P"))
								{
									predi_condition = true;
								}
							}
							}
							if(south_o_nodeP && (south_o_nodeP == south_o_node) && !predi_condition){
								if(south_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && southoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
									insF.southo = "101";
									//std::cout << "S_TREG\n";
								}
								else if(south_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && southoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
									insF.southo = "100";
									//std::cout << "S_ALU\n";
								}else{
									std::cout << "Port : " << southo->getFullName() << ",node = " << southo->getNode()->idx << ", source not found!\n";
									insF.southo = "111";
								}
							}
#else
							std::cout << "Port : " << southo->getFullName() << ",node = " << southo->getNode()->idx << ", source not found!\n";
							insF.southo = "111";
							assert(false);
#endif
						}
					}
					else{
						insF.southo = "111";
					}


					if(p_ip->getNode()){
						if(p_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.alu_p = "011";
						}
						else if(p_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.alu_p = "000";
						}
						else if(p_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.alu_p = "010";
						}
						else if(p_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.alu_p = "001";
						}
						else if(p_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() &&
								p_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							insF.alu_p = "101";
						}
						else if(p_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && p_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
							insF.alu_p = "100";
						}
						else if(p_ip->getNode()  == dp->getOutPort("T")->getNode() && p_ip->getLat() == dp->getOutPort("T")->getLat()){
							insF.alu_p = "100";
						}
						else{
							std::cout << "Port : " << p_ip->getFullName() << ",node = " << p_ip->getNode()->idx << ", source not found!\n";
						//	assert(false);
							insF.alu_p = "111";
						}
					}
					else{
						insF.alu_p = "111";
					}

					if(i1_ip->getNode()){
						if(i1_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.alu_i1 = "011";
							if(currentMappedOP &&currentMappedOP->type_i1i2){
								insF.alu_i2 = "011";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.alu_i1 = "000";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "000";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.alu_i1 = "010";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "010";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i1_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.alu_i1 = "001";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "001";
							}
						}
						else if(i1_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() &&
								i1_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							insF.alu_i1 = "101";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "101";
							}
						}
						else if(i1_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && i1_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
							insF.alu_i1 = "100";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "100";
							}
						}
						//THILINI:: check with RTl for correct config
						//					else if(i1_ip->getNode() == fu->getInPort("DP0_I1")->getNode() && i1_ip->getLat() == fu->getInPort("DP0_I1")->getLat()){
						//						insF.alu_i1 = "110";
						//					}

						else if(i1_ip->getNode() == dp->getOutPort("T")->getNode() && i1_ip->getLat() == dp->getOutPort("T")->getLat()){
							insF.alu_i1 = "100";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "100";
							}
						}
						else{
							std::cout << "Port : " << i1_ip->getFullName() << ",node = " << i1_ip->getNode()->idx << ", source not found!\n";
						//	assert(false);
							insF.alu_i1 = "111";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "111";
							}
						}
					}
					else{
						insF.alu_i1 = "111";
					}


					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
					if(i2_ip->getNode()){
						if(i2_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.alu_i2 = "011";
						}
						else if(i2_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.alu_i2 = "000";
						}
						else if(i2_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.alu_i2 = "010";
						}
						else if(i2_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i2_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.alu_i2 = "001";
						}
						else if(i2_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() &&
								i2_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							insF.alu_i2 = "101";
						}
						else if(i2_ip->getNode() == fu->getOutPort("DP0_T")->getNode() &&
								i2_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
							insF.alu_i2 = "100";
						}
						//THILINI:: check with RTl for correct config
						//					else if(i2_ip->getNode() == fu->getInPort("DP0_I2")->getNode() && i2_ip->getLat() == fu->getInPort("DP0_I2")->getLat()){
						//						insF.alu_i2 = "110";
						//					}
						else if(i2_ip->getNode() == dp->getOutPort("T")->getNode() && i2_ip->getLat() == dp->getOutPort("T")->getLat()){
							insF.alu_i2 = "100";
						}
						else{
							std::cout << "Port : " << i2_ip->getFullName() << ",node = " << i2_ip->getNode()->idx << ", source not found!\n";
						//	assert(false);
							insF.alu_i2 = "111";
						}
					}
					else{
						insF.alu_i2 = "111";
					}
					}

					//TREG WE
					if( pe->getSingleRegPort("TREG_RO")->getNode() &&
							fu->getOutPort("DP0_T")->getNode() &&
							pe->getSingleRegPort("TREG_RO")->getNode() == fu->getOutPort("DP0_T")->getNode() ){
						insF.treg_we = "1";
					}
					else{
						insF.treg_we = "0";
					}

					DFGNode *currNode = fu->getOutPort("DP0_T")->getNode();
					if(currNode){
						std::vector<DFGNode *> childrenNodes = currNode->children;
						insF.is_predicate = "0";
						for(DFGNode* node1:childrenNodes)
						{
							if((currNode->getOPtype(node1) == "P"))
							{
								insF.is_predicate = "1";
								break;
							}else
							{
								insF.is_predicate = "0";
							}
						}
					}else
					{
						insF.is_predicate = "0";
					}


					// Register write enables

					Port* northi = pe->getInPort("NORTH_I"); assert(northi);
					Port* easti = pe->getInPort("EAST_I"); assert(easti);
					Port* westi = pe->getInPort("WEST_I"); assert(westi);
					Port* southi = pe->getInPort("SOUTH_I"); assert(southi);

					//				RegFile* RF0 = static_cast<RegFile*>(pe->getSubMod("RF0"));
					//				RegFile* RF1 = static_cast<RegFile*>(pe->getSubMod("RF1"));
					//				RegFile* RF2 = static_cast<RegFile*>(pe->getSubMod("RF2"));
					//				RegFile* RF3 = static_cast<RegFile*>(pe->getSubMod("RF3"));


					if(pe->getSingleRegPort("NR_RO")->getNode() &&
							northi->getNode() &&
							pe->getSingleRegPort("NR_RO")->getNode() == northi->getNode()){
						insF.north_reg_we = "1";
					}
					else{
						insF.north_reg_we = "0";
					}


					if(pe->getSingleRegPort("ER_RO")->getNode() &&
							easti->getNode() &&
							pe->getSingleRegPort("ER_RO")->getNode() == easti->getNode()){
						insF.east_reg_we = "1";
					}
					else{
						insF.east_reg_we = "0";
					}

					if(pe->getSingleRegPort("WR_RO")->getNode() &&
							westi->getNode() &&
							pe->getSingleRegPort("WR_RO")->getNode() == westi->getNode()){
						insF.west_reg_we = "1";
					}
					else{
						insF.west_reg_we = "0";
					}

					if(pe->getSingleRegPort("SR_RO")->getNode() &&
							southi->getNode() &&
							pe->getSingleRegPort("SR_RO")->getNode() == southi->getNode()){
						insF.south_reg_we = "1";
					}
					else{
						insF.south_reg_we = "0";
					}


					//setting bypass bits
					DFGNode* northi_node = northi->getNode();
					DFGNode* easti_node = easti->getNode();
					DFGNode* westi_node = westi->getNode();
					DFGNode* southi_node = southi->getNode();


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
							std::cout << "pe=" << pe->getName() << ",node=" << pe->getSingleRegPort("WR_RI")->getNode()->idx << ",portnode=" << westi_node->idx << "\n";
							//assert(pe->getSingleRegPort("WR_RI")->getNode() != pe->getInternalPort("WEST_XBARI")->getNode());
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
						if(mappedOP->npb){
							insF.negated_predicate = "1";
						}
					}
					else{
						insF.opcode = "00000";
					}

					if(mappedOP && mappedOP->hasConst){
						insF.constant_valid = "1";
						insF.constant = mappedOP->get27bitConstantBinaryString();
					}
					else{
						insF.constant_valid = "0";
						//					insF.constant = "123456789012345678901234567";
						insF.constant = "000000000000000000000000000";
					}

					if( mappedOP && mappedOP->npb){
						insF.negated_predicate = "1";
						//					assert(false);
					}
					else{
						insF.negated_predicate = "0";
					}

					InsFArr[t+1][Y][X] = insF;

					//		}
					//	}
				}
	}
	InsFormat jumpl;
	jumpl.negated_predicate = "0";
	jumpl.constant_valid = "1";
	jumpl.constant = "000000000000" + std::bitset<5>(1).to_string() + std::bitset<5>(cgra->get_t_max()).to_string() + std::bitset<5>(1).to_string();
	assert(jumpl.constant.size() == 27);
	jumpl.opcode = "11110";
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
	jumpl.is_predicate = "0";
	//cout << "THILINI2  " <<  cgra->get_t_max() << "\n" ;

	int x;
	int y;
	vector<PE *> peList = this->cgra->getSpatialPEList(0);
	for (PE *pe : peList)
	{
		//for (int y = 0; y < Y; ++y) {
		//	for (int x = 0; x < X; ++x) {
		x = pe->X;
		y = pe->Y;

		jumpl.alu_p = InsFArr[cgra->get_t_max()][y][x].alu_p;
		jumpl.alu_i1 = InsFArr[cgra->get_t_max()][y][x].alu_i1;
		jumpl.alu_i2 = InsFArr[cgra->get_t_max()][y][x].alu_i2;
		jumpl.northo = InsFArr[cgra->get_t_max()][y][x].northo;
		jumpl.easto = InsFArr[cgra->get_t_max()][y][x].easto;
		jumpl.westo = InsFArr[cgra->get_t_max()][y][x].westo;
		jumpl.southo = InsFArr[cgra->get_t_max()][y][x].southo;
		///cout << "THILINI::" << jumpl.alu_p << jumpl.alu_i1 << jumpl.alu_i2 << "\n";
		InsFArr[0][y][x] = jumpl;
	}

	std::string binFName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs)  + "_XDim=" + std::to_string(this->cgra->get_x_max()) + "_YDim=" + std::to_string(this->cgra->get_y_max()) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths) + "_binary.bin";
	printBinFile(InsFArr,binFName,cgra);



}


void CGRAXMLCompile::PathFinderMapper::printHyCUBEBinary4(CGRA* cgra) {

	std::vector<std::vector<std::vector<InsFormat>>> InsFArr (cgra->get_t_max()+1,
			std::vector<std::vector<InsFormat>>(cgra->get_y_max(),
					std::vector<InsFormat>(cgra->get_x_max())
			)
	);
	for (int t = 0; t < cgra->get_t_max(); ++t) {
		vector<PE *> peList = this->cgra->getSpatialPEList(t);
				int iter=0;
				for (PE *pe : peList)
				{

					Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
					Port* easto = pe->getOutPort("EAST_O"); assert(easto);
					Port* westo = pe->getOutPort("WEST_O"); assert(westo);
					Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

					DFGNode* north_o_node = northo->getNode();
					DFGNode* east_o_node = easto->getNode();
					DFGNode* west_o_node = westo->getNode();
					DFGNode* south_o_node = southo->getNode();

					FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

					DFGNode* currentMappedOP = dp->getMappedNode();

					int prev_t;
					int X;
					int Y;
					X = pe->X;
					Y = pe->Y;
					prev_t = (t + 2*cgra->get_t_max() - 1)%cgra->get_t_max();
					vector<PE *> prevPEList = this->cgra->getSpatialPEList(prev_t);

					PE* prevPE = prevPEList.at(iter);
					FU* prevFU = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevFU);
					DataPath* prevDP = static_cast<DataPath*>(prevFU->getSubMod("DP0"));
					DFGNode* mappedOP = prevDP->getMappedNode();


					iter++;

					Module* mod =  westo->getMod();
					DataPath *mod_dp = static_cast<DataPath *>(mod);

					Port* i1_lsb_ip = dp->getInPort("I1_LSB"); assert(i1_lsb_ip);
					Port* i1_msb_ip = dp->getInPort("I1_MSB"); assert(i1_msb_ip);
					Port* i2_lsb_ip = dp->getInPort("I2_LSB"); assert(i2_lsb_ip);
					Port* i2_msb_ip = dp->getInPort("I2_MSB"); assert(i2_msb_ip);
					Port* p_ip = dp->getInPort("P"); assert(p_ip);

					InsFormat insF;


					//XBar
					if(north_o_node){

						if(north_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.northo = "0000";
						}
						else if(north_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.northo = "0010";
						}
						else if(north_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.northo = "0001";
						}
						else if(north_o_node == pe->getSingleRegPort("TREG_LSB_RI")->getNode() &&
								northo->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat() && !northo->getMSB()){
							insF.northo = "0101";
						}
						else if(north_o_node == pe->getSingleRegPort("TREG_MSB_RI")->getNode() &&
								northo->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat() && northo->getMSB()){
							insF.northo = "0110";
						}

						else if(north_o_node == fu->getOutPort("DP0_T_LSB")->getNode()
								&& northo->getLat() == fu->getOutPort("DP0_T_LSB")->getLat() && !northo->getMSB()){
							insF.northo = "0100";
						}
						else if(north_o_node == fu->getOutPort("DP0_T_MSB")->getNode()
								&& northo->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()&& northo->getMSB()){
							insF.northo = "0111";
						}
						else if(north_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								northo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.northo = "0011";
						}
						else{

							std::cout << "Port : " << northo->getFullName() << ",node = " << northo->getNode()->idx << ", source not found!\n";
							insF.northo = "1111";
							assert(false);
						}
					}


					else{
						insF.northo = "1111";
					}

					if(east_o_node){
						if(east_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.easto = "0011";
						}
						else if(east_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
													easto->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
												insF.easto = "0001";
						}
						else if(east_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								easto->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.easto = "0010";
						}
						else if(east_o_node == pe->getSingleRegPort("TREG_LSB_RI")->getNode() &&
								easto->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat() && !easto->getMSB()){
							insF.easto = "0101";
						}
						else if(east_o_node == fu->getOutPort("DP0_T_LSB")->getNode()
								&& easto->getLat() == fu->getOutPort("DP0_T_LSB")->getLat() && !easto->getMSB()){
							insF.easto = "0100";
						}
						else if(east_o_node == pe->getSingleRegPort("TREG_MSB_RI")->getNode() &&
								easto->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat() && easto->getMSB()){
													insF.easto = "0110";
						}
						else if(east_o_node == fu->getOutPort("DP0_T_MSB")->getNode()
								&& easto->getLat() == fu->getOutPort("DP0_T_MSB")->getLat() && easto->getMSB()){
													insF.easto = "0111";
												}
						else if(east_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
														easto->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
													insF.easto = "0000";
												}
						else{

							std::cout << "Port : " << easto->getFullName() << ",node = " << easto->getNode()->idx << ", source not found!\n";
							insF.easto = "1111";
							assert(false);
						}
					}
					else{
						insF.easto = "1111";
					}

					if(west_o_node){
						if(west_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.westo = "0011";
						}
						else if(west_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.westo = "0000";
						}
						else if(west_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								westo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.westo = "0001";
						}
						else if(west_o_node ==  pe->getSingleRegPort("TREG_LSB_RI")->getNode() &&
								westo->getLat() ==  pe->getSingleRegPort("TREG_LSB_RI")->getLat() && !westo->getMSB()){
							insF.westo = "0101";
						}
						else if(west_o_node == fu->getOutPort("DP0_T_LSB")->getNode() &&
								westo->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()  && !westo->getMSB()){
							insF.westo = "0100";
						}
						else if(west_o_node ==  pe->getSingleRegPort("TREG_MSB_RI")->getNode() &&
								westo->getLat() ==  pe->getSingleRegPort("TREG_MSB_RI")->getLat()&& westo->getMSB() ){
							insF.westo = "0110";
						}
						else if(west_o_node == fu->getOutPort("DP0_T_MSB")->getNode() &&
								westo->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()  && westo->getMSB()){
							insF.westo = "0111";
						}
						else if(west_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
														westo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
													insF.westo = "0010";
												}
						else{
								std::cout << "Port : " << westo->getFullName() << ",node = " << westo->getNode()->idx << ", source not found!\n";
								insF.westo = "1111";
								assert(false);
						}
					}
					else{
						insF.westo = "1111";
					}

					if(south_o_node){
						if(south_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.southo = "0011";
						}
						else if(south_o_node == pe->getInternalPort("EAST_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.southo = "0000";
						}
						else if(south_o_node == pe->getInternalPort("WEST_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.southo = "0010";
						}
						else if(south_o_node == pe->getSingleRegPort("TREG_LSB_RI")->getNode() &&
								southo->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat() && !southo->getMSB()){
							insF.southo = "0101";
						}
						else if(south_o_node == fu->getOutPort("DP0_T_LSB")->getNode()
								&& southo->getLat() == fu->getOutPort("DP0_T_LSB")->getLat() && !southo->getMSB()){
							insF.southo = "0100";
						}
						else if(south_o_node == pe->getSingleRegPort("TREG_MSB_RI")->getNode() &&
								southo->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat() && southo->getMSB()){
							insF.southo = "0110";
						}
						else if(south_o_node == fu->getOutPort("DP0_T_MSB")->getNode()
								&& southo->getLat() == fu->getOutPort("DP0_T_MSB")->getLat() && southo->getMSB()){
							insF.southo = "0111";
						}
						else if(south_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								southo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.southo = "0001";
						}
						else{
							std::cout << "Port : " << southo->getFullName() << ",node = " << southo->getNode()->idx << ", source not found!\n";
							insF.southo = "1111";
							assert(false);
						}
					}
					else{
						insF.southo = "1111";
					}


					if(p_ip->getNode()){
						if(p_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							insF.alu_p = "0011";
						}
						else if(p_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							insF.alu_p = "0000";
						}
						else if(p_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							insF.alu_p = "0010";
						}
						else if(p_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								p_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							insF.alu_p = "0001";
						}
						else if(p_ip->getNode() == pe->getSingleRegPort("TREG_LSB_RI")->getNode() &&
								p_ip->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
							insF.alu_p = "0101";
						}
						else if(p_ip->getNode() == fu->getOutPort("DP0_T_LSB")->getNode()
								&& p_ip->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
							insF.alu_p = "0100";
						}
						else if(p_ip->getNode()  == dp->getOutPort("T_LSB")->getNode()
								&& p_ip->getLat() == dp->getOutPort("T_LSB")->getLat()){
							insF.alu_p = "0100";
						}

						else{
							std::cout << "Port : " << p_ip->getFullName() << ",node = " << p_ip->getNode()->idx << ", source not found!\n";
							insF.alu_p = "1111";
						}
					}
					else{
						insF.alu_p = "1111";
					}

					if(i1_lsb_ip->getNode()){

						if(i1_lsb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i1_lsb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat() && !pe->getInternalPort("NORTH_XBARI")->getMSB()){
							insF.alu_i1 = "0011";
							if(currentMappedOP &&currentMappedOP->type_i1i2){
								insF.alu_i2 = "0011";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i1_lsb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat() && !pe->getInternalPort("EAST_XBARI")->getMSB()){
							insF.alu_i1 = "0000";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "0000";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i1_lsb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat() && !pe->getInternalPort("WEST_XBARI")->getMSB()){
							insF.alu_i1 = "0010";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "0010";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i1_lsb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat() && !pe->getInternalPort("SOUTH_XBARI")->getMSB()){
							insF.alu_i1 = "0001";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "0001";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getSingleRegPort("TREG_LSB_RI")->getNode() &&
								i1_lsb_ip->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat() && !pe->getSingleRegPort("TREG_LSB_RI")->getMSB()){
							insF.alu_i1 = "0101";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "0101";
							}
						}
						else if(i1_lsb_ip->getNode() == fu->getOutPort("DP0_T_LSB")->getNode() &&
								i1_lsb_ip->getLat() == fu->getOutPort("DP0_T_LSB")->getLat() && !fu->getOutPort("DP0_T_LSB")->getMSB()){
							insF.alu_i1 = "0100";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "0100";
							}
						}
						else if(i1_lsb_ip->getNode() == dp->getOutPort("T_LSB")->getNode() &&
								i1_lsb_ip->getLat() == dp->getOutPort("T_LSB")->getLat() && !dp->getOutPort("T_LSB")->getMSB()){
							insF.alu_i1 = "0100";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "0100";
							}
						}

						else{
							std::cout << "Port : " << i1_lsb_ip->getFullName() << ",node = " << i1_lsb_ip->getNode()->idx << ", source not found!\n";
							insF.alu_i1 = "1111";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2 = "1111";
							}
						}
					}
					else{
						insF.alu_i1 = "1111";
					}
					//std::cout << "XXXXXX\n";
					if(i1_msb_ip->getNode()){
						if(i1_msb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i1_msb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat() && pe->getInternalPort("NORTH_XBARI")->getMSB()){
							insF.alu_i1_msb = "0011";
							if(currentMappedOP &&currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "0011";
							}
						}
						else if(i1_msb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i1_msb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat() && pe->getInternalPort("EAST_XBARI")->getMSB()){
							insF.alu_i1_msb = "0000";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "0000";
							}
						}
						else if(i1_msb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i1_msb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat() && pe->getInternalPort("WEST_XBARI")->getMSB()){
							insF.alu_i1_msb = "0010";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "0010";
							}
						}
						else if(i1_msb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i1_msb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat() && pe->getInternalPort("SOUTH_XBARI")->getMSB()){
							insF.alu_i1_msb = "0001";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "0001";
							}
						}

						else if(i1_msb_ip->getNode() == pe->getSingleRegPort("TREG_MSB_RI")->getNode() &&
								i1_msb_ip->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat() && pe->getSingleRegPort("TREG_MSB_RI")->getMSB()){
							insF.alu_i1_msb = "0110";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "0110";
							}
						}
						else if(i1_msb_ip->getNode() == fu->getOutPort("DP0_T_MSB")->getNode() &&
								i1_msb_ip->getLat() == fu->getOutPort("DP0_T_MSB")->getLat() && fu->getOutPort("DP0_T_MSB")->getMSB()){
							insF.alu_i1_msb = "0111";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "0111";
							}
						}
						else if(i1_msb_ip->getNode() == dp->getOutPort("T_MSB")->getNode() &&
								i1_msb_ip->getLat() == dp->getOutPort("T_MSB")->getLat() && dp->getOutPort("T_MSB")->getMSB()){
							insF.alu_i1_msb = "0111";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "0111";
							}
						}
						else{
							std::cout << "Port : " << i1_msb_ip->getFullName() << ",node = " << i1_msb_ip->getNode()->idx << ", source not found!\n";
							insF.alu_i1_msb = "1111";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								insF.alu_i2_msb = "1111";
							}
						}
					}
					else{
						insF.alu_i1_msb = "1111";
					}


					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
					if(i2_lsb_ip->getNode()){
						if(i2_lsb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i2_lsb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat() && !pe->getInternalPort("NORTH_XBARI")->getMSB()){
							insF.alu_i2 = "0011";
						}
						else if(i2_lsb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i2_lsb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat() && !pe->getInternalPort("EAST_XBARI")->getMSB()){
							insF.alu_i2 = "0000";
						}
						else if(i2_lsb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i2_lsb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat() && !pe->getInternalPort("WEST_XBARI")->getMSB()){
							insF.alu_i2 = "0010";
						}
						else if(i2_lsb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i2_lsb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat() && !pe->getInternalPort("SOUTH_XBARI")->getMSB()){
							insF.alu_i2 = "0001";
						}
						else if(i2_lsb_ip->getNode() == pe->getSingleRegPort("TREG_LSB_RI")->getNode() &&
								i2_lsb_ip->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat() && !pe->getSingleRegPort("TREG_LSB_RI")->getMSB()){
							insF.alu_i2 = "0101";
						}
						else if(i2_lsb_ip->getNode() == fu->getOutPort("DP0_T_LSB")->getNode() &&
								i2_lsb_ip->getLat() == fu->getOutPort("DP0_T_LSB")->getLat() && !fu->getOutPort("DP0_T_LSB")->getMSB()){
							insF.alu_i2 = "0100";
						}
						else if(i2_lsb_ip->getNode() == dp->getOutPort("T_LSB")->getNode()
								&& i2_lsb_ip->getLat() == dp->getOutPort("T_LSB")->getLat() && !dp->getOutPort("T_LSB")->getMSB()){
							insF.alu_i2 = "0100";
						}

						else{
							std::cout << "Port : " << i2_lsb_ip->getFullName() << ",node = " << i2_lsb_ip->getNode()->idx << ", source not found!\n";
							insF.alu_i2 = "1111";
						}
					}
					else{
						insF.alu_i2 = "1111";
					}
					}


					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
					if(i2_msb_ip->getNode()){
						if(i2_msb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() &&
								i2_msb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat() && pe->getInternalPort("NORTH_XBARI")->getMSB()){
							insF.alu_i2_msb = "0011";
						}
						else if(i2_msb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() &&
								i2_msb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat() && pe->getInternalPort("EAST_XBARI")->getMSB()){
							insF.alu_i2_msb = "0000";
						}
						else if(i2_msb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() &&
								i2_msb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat() && pe->getInternalPort("WEST_XBARI")->getMSB()){
							insF.alu_i2_msb = "0010";
						}
						else if(i2_msb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() &&
								i2_msb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat() && pe->getInternalPort("SOUTH_XBARI")->getMSB()){
							insF.alu_i2_msb = "0001";
						}

						else if(i2_msb_ip->getNode() == pe->getSingleRegPort("TREG_MSB_RI")->getNode() &&
								i2_msb_ip->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat() && pe->getSingleRegPort("TREG_MSB_RI")->getMSB()){
							insF.alu_i2_msb = "0110";
						}
						else if(i2_msb_ip->getNode() == fu->getOutPort("DP0_T_MSB")->getNode() &&
								i2_msb_ip->getLat() == fu->getOutPort("DP0_T_MSB")->getLat() && fu->getOutPort("DP0_T_MSB")->getMSB()){
							insF.alu_i2_msb = "0111";
						}

						else if(i2_msb_ip->getNode() == dp->getOutPort("T_MSB")->getNode()
								&& i2_msb_ip->getLat() == dp->getOutPort("T_MSB")->getLat() && dp->getOutPort("T_MSB")->getMSB()){
							insF.alu_i2_msb = "0111";
						}
						else{
							std::cout << "Port : " << i2_msb_ip->getFullName() << ",node = " << i2_msb_ip->getNode()->idx << ", source not found!\n";
							insF.alu_i2_msb = "1111";
						}
					}
					else{
						insF.alu_i2_msb= "1111";
					}
					}
					//TREG WE
					if( pe->getSingleRegPort("TREG_LSB_RO")->getNode() &&
							fu->getOutPort("DP0_T_LSB")->getNode() &&
							pe->getSingleRegPort("TREG_LSB_RO")->getNode() == fu->getOutPort("DP0_T_LSB")->getNode() ){
						insF.treg_we = "1";
					}
					else{
						insF.treg_we = "0";
					}

					//TREG WE
					if( pe->getSingleRegPort("TREG_MSB_RO")->getNode() &&
							fu->getOutPort("DP0_T_MSB")->getNode() &&
							pe->getSingleRegPort("TREG_MSB_RO")->getNode() == fu->getOutPort("DP0_T_MSB")->getNode() ){
						insF.treg_we_msb = "1";
					}
					else{
						insF.treg_we_msb = "0";
					}

					DFGNode *currNode = fu->getOutPort("DP0_T_LSB")->getNode();
					if(currNode){
						std::vector<DFGNode *> childrenNodes = currNode->children;
						insF.is_predicate = "0";
						for(DFGNode* node1:childrenNodes)
						{
							if((currNode->getOPtype(node1) == "P"))
							{
								insF.is_predicate = "1";
								break;
							}else
							{
								insF.is_predicate = "0";
							}
						}
					}else
					{
						insF.is_predicate = "0";
					}


					// Register write enables

					Port* northi = pe->getInPort("NORTH_I"); assert(northi);
					Port* easti = pe->getInPort("EAST_I"); assert(easti);
					Port* westi = pe->getInPort("WEST_I"); assert(westi);
					Port* southi = pe->getInPort("SOUTH_I"); assert(southi);



					if(pe->getSingleRegPort("NR_RO")->getNode() &&
							northi->getNode() &&
							pe->getSingleRegPort("NR_RO")->getNode() == northi->getNode()){
						insF.north_reg_we = "1";
					}
					else{
						insF.north_reg_we = "0";
					}

					if(pe->getSingleRegPort("ER_RO")->getNode() &&
							easti->getNode() &&
							pe->getSingleRegPort("ER_RO")->getNode() == easti->getNode()){
						insF.east_reg_we = "1";
					}
					else{
						insF.east_reg_we = "0";
					}

					if(pe->getSingleRegPort("WR_RO")->getNode() &&
							westi->getNode() &&
							pe->getSingleRegPort("WR_RO")->getNode() == westi->getNode()){
						insF.west_reg_we = "1";
					}
					else{
						insF.west_reg_we = "0";
					}

					if(pe->getSingleRegPort("SR_RO")->getNode() &&
							southi->getNode() &&
							pe->getSingleRegPort("SR_RO")->getNode() == southi->getNode()){
						insF.south_reg_we = "1";
					}
					else{
						insF.south_reg_we = "0";
					}


					//setting bypass bits
					DFGNode* northi_node = northi->getNode();
					DFGNode* easti_node = easti->getNode();
					DFGNode* westi_node = westi->getNode();
					DFGNode* southi_node = southi->getNode();


					if(northi_node &&
							northi_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northi->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
						if(pe->getSingleRegPort("NR_RI")->getNode()){
							std::cout << "pe=" << pe->getName() << ",node=" << pe->getSingleRegPort("NR_RI")->getNode()->idx << ",portnode=" << northi_node->idx << "\n";
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
							std::cout << "pe=" << pe->getName() << ",node=" << pe->getSingleRegPort("WR_RI")->getNode()->idx << ",portnode=" << westi_node->idx << "\n";
							//assert(pe->getSingleRegPort("WR_RI")->getNode() != pe->getInternalPort("WEST_XBARI")->getNode());
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
						if(mappedOP->npb){
							insF.negated_predicate = "1";
						}
					}
					else{
						insF.opcode = "00000";
					}

					if(mappedOP && mappedOP->hasConst){
						insF.constant_valid = "1";
						insF.constant = mappedOP->get27bitConstantBinaryString();
					}
					else{
						insF.constant_valid = "0";
						insF.constant = "000000000000000000000000000";
					}

					if( mappedOP && mappedOP->npb){
						insF.negated_predicate = "1";
					}
					else{
						insF.negated_predicate = "0";
					}

					InsFArr[t+1][Y][X] = insF;

				}
	}
	InsFormat jumpl;
	jumpl.negated_predicate = "0";
	jumpl.constant_valid = "1";
	jumpl.constant = "000000000000" + std::bitset<5>(1).to_string() + std::bitset<5>(cgra->get_t_max()).to_string() + std::bitset<5>(1).to_string();
	assert(jumpl.constant.size() == 27);
	jumpl.opcode = "11110";
	jumpl.north_reg_we = "0";
	jumpl.east_reg_we = "0";
	jumpl.west_reg_we = "0";
	jumpl.south_reg_we = "0";
	jumpl.treg_we = "0";
	jumpl.north_reg_bypass = "0";
	jumpl.east_reg_bypass = "0";
	jumpl.west_reg_bypass = "0";
	jumpl.south_reg_bypass = "0";
	jumpl.alu_p = "1111";
	jumpl.alu_i1 = "1111";
	jumpl.alu_i2 = "1111";
	jumpl.northo = "1111";
	jumpl.easto = "1111";
	jumpl.westo = "1111";
	jumpl.southo = "1111";
	jumpl.is_predicate = "0";
	jumpl.alu_i1_msb = "1111";
	jumpl.alu_i2_msb = "1111";
	jumpl.treg_we_msb = "0";


	int x;
	int y;
	vector<PE *> peList = this->cgra->getSpatialPEList(0);
	for (PE *pe : peList)
	{
		x = pe->X;
		y = pe->Y;

		jumpl.alu_p = InsFArr[cgra->get_t_max()][y][x].alu_p;
		jumpl.alu_i1 = InsFArr[cgra->get_t_max()][y][x].alu_i1;
		jumpl.alu_i2 = InsFArr[cgra->get_t_max()][y][x].alu_i2;
		jumpl.northo = InsFArr[cgra->get_t_max()][y][x].northo;
		jumpl.easto = InsFArr[cgra->get_t_max()][y][x].easto;
		jumpl.westo = InsFArr[cgra->get_t_max()][y][x].westo;
		jumpl.southo = InsFArr[cgra->get_t_max()][y][x].southo;
		jumpl.alu_i1_msb = InsFArr[cgra->get_t_max()][y][x].alu_i1_msb;
		jumpl.alu_i2_msb = InsFArr[cgra->get_t_max()][y][x].alu_i2_msb;
		InsFArr[0][y][x] = jumpl;
	}

	std::string binFName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs)  + "_XDim=" + std::to_string(this->cgra->get_x_max()) + "_YDim=" + std::to_string(this->cgra->get_y_max()) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths) + "_binary.bin";
	//std::cout << "Print binary\n";
	printBinFile(InsFArr,binFName,cgra);
	std::cout << "Print binary11\n";
	printHyCUBEConfig(InsFArr,"config.bin",cgra);
}

void CGRAXMLCompile::PathFinderMapper::printBinFile(
		const std::vector<std::vector<std::vector<InsFormat> > >& insFArr,
		std::string fName, CGRA* cgra) {

	std::ofstream binFile(fName.c_str());
	int t_max = cgra->get_t_max() + 1;
	int y_max = cgra->get_y_max();
	int x_max = cgra->get_x_max();
	//int y_max = 4;
	//int x_max = 4;
	binFile << "NPB,CONSTVALID,CONST,OPCODE,REGWEN,TREGWEN,REGBYPASS,PRED,OP1,OP2,NORTH,WEST,SOUTH,EAST\n";


	for(int y = 0 ; y < y_max ; y++){
		for(int x = 0 ; x < x_max ; x++){

		}
	}



	for(int t = 0 ; t < t_max ; t++){
		binFile << t << "\n";
		for(int y = 0 ; y < y_max ; y++){
			for(int x = 0 ; x < x_max ; x++){

				binFile << "Y=" << y << " X=" << x << ",";
				binFile << insFArr[t][y][x].negated_predicate;
				binFile << insFArr[t][y][x].is_predicate;
				binFile << insFArr[t][y][x].constant_valid;
				binFile << insFArr[t][y][x].constant;
				binFile << insFArr[t][y][x].opcode;
				binFile << insFArr[t][y][x].north_reg_we;
				binFile << insFArr[t][y][x].west_reg_we;
				binFile << insFArr[t][y][x].south_reg_we;
				binFile << insFArr[t][y][x].east_reg_we;
				binFile << insFArr[t][y][x].treg_we;
				binFile << insFArr[t][y][x].south_reg_bypass;
				binFile << insFArr[t][y][x].north_reg_bypass;
				binFile << insFArr[t][y][x].west_reg_bypass;
				binFile << insFArr[t][y][x].east_reg_bypass;

				binFile << insFArr[t][y][x].alu_p;
				binFile << insFArr[t][y][x].alu_i2;
				binFile << insFArr[t][y][x].alu_i1;
				binFile << insFArr[t][y][x].northo;
				binFile << insFArr[t][y][x].westo;
				binFile << insFArr[t][y][x].southo;
				binFile << insFArr[t][y][x].easto;
				binFile << insFArr[t][y][x].alu_i1_msb;
				binFile << insFArr[t][y][x].alu_i2_msb;
				binFile << insFArr[t][y][x].treg_we_msb;


				binFile << "\n";
			}
		}
		binFile << "\n";
	}


	binFile.close();
}


std::map<CGRAXMLCompile::DataPath *, int> CGRAXMLCompile::PathFinderMapper::checkConstSimilarity(
		std::vector<CGRAXMLCompile::DataPath *> candDestIn,CGRAXMLCompile::DFGNode * node){
	std::map<CGRAXMLCompile::DataPath *, int> constCostAddedCandidates;

	//unordered_set<PE *> allPEs = this->cgra->getAllPEList();
	//this->cgra->getTemporalPEList(x, y);

	//TODO::Add a sperate structure to save constants to reduce compilation time
	int cost=LARGE_VALUE;

	for(DataPath *candDP: candDestIn)
	{
		if(node->hasConst)
		{
			int x= candDP->getPE()->X;
			int y= candDP->getPE()->Y;

			int currT = candDP->getPE()->T;
			vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);
			PE * spatialPE = this->cgra->getSpatialPE(candDP->getPE());

			int lastConst;
			int constCount=0;
			int LB_T=0;
			int UB_T=(this->cgra->get_t_max()-1);

			int LBConst;
			int UBConst;

			bool constEmpty=true;
			bool noLower=true;
			bool noUpper=true;

				for(PE * tempPE:temporalPEList)
				{
					FU* fu = static_cast<FU*>(tempPE->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					DFGNode* mappedOP = dp->getMappedNode();

					if(mappedOP && mappedOP->hasConst)
					{
						if(lastConst != mappedOP->constant)
							constCount++;
						lastConst = mappedOP->constant;

						if((tempPE->T < currT) && (tempPE->T >LB_T))
						{
							LB_T = tempPE->T;
							LBConst = mappedOP->constant;
							noLower=false;
						}

						if((tempPE->T > currT) && (tempPE->T < UB_T))
						{
							UB_T = tempPE->T;
							UBConst = mappedOP->constant;
							noUpper=false;
						}
						constEmpty=false;
					}
				}

				if(constCount < spatialPE->const_mem_size)
				{
					if(!constEmpty){
						if(!noLower && !noUpper)
						{
							int d1 = currT - LB_T;
							int d2 = UB_T - currT;

							if(LBConst == UBConst == node->constant){
								cost =0;
							}
							else if(LBConst == UBConst){
								cost = this->const_cost/(d1) + this->const_cost/(d2) + this->const_cost;
							}
							else if((LBConst == node->constant) && (UBConst != node->constant)){
								cost = this->const_cost/(d2);
							}
							else if((LBConst != node->constant) && (UBConst == node->constant)){
								cost = this->const_cost/(d1);
							}
							else{
								cost = this->const_cost/(d1) + this->const_cost/(d2);
							}
						}
						else if(!noLower)
						{
							int d1 = currT - LB_T;

							if(LBConst == node->constant){
								cost =0;
							}
							else{
								cost = this->const_cost/(d1);
							}
						}
						else if(!noUpper)
						{
							int d2 = UB_T - currT;
							if(UBConst == node->constant){
								cost =0;
							}
							else{
								cost = this->const_cost/(d2);
							}
						}
						else {
							cost =0;
						}
					}else
					{
						cost = this->const_cost/5;
					}
				}else{
					//cost =100000;
					//std::cout << "THILINITTT:: Const count " << constCount << "\t" << spatialPE->const_mem_size << "\n";
					continue;
				}

		}else{
			cost=0;
		}
		constCostAddedCandidates[candDP]=cost;
		//std::cout << "CONST ::" << candDP->getPE()->getName() << "  const cost  -> " << cost << " node->hasconst ===" << node->hasConst << "\n";
	}

	return constCostAddedCandidates;
}

std::map<CGRAXMLCompile::DataPath *, int> CGRAXMLCompile::PathFinderMapper::checkOpcodeSimilarity(
		std::vector<CGRAXMLCompile::DataPath *> candDestIn,CGRAXMLCompile::DFGNode * node){
	std::map<CGRAXMLCompile::DataPath *, int> opcodeCostAddedCandidates;

	//unordered_set<PE *> allPEs = this->cgra->getAllPEList();
	//this->cgra->getTemporalPEList(x, y);

	//TODO::Add a sperate structure to save constants to reduce compilation time
	int cost=LARGE_VALUE;

	for(DataPath *candDP: candDestIn)
	{
		if(!(node->op).empty())
		{
			int x= candDP->getPE()->X;
			int y= candDP->getPE()->Y;

			int currT = candDP->getPE()->T;
			vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);
			PE * spatialPE = this->cgra->getSpatialPE(candDP->getPE());

			std::string lastOpcode;
			int opcodeCount=0;
			int LB_T=0;
			int UB_T=(this->cgra->get_t_max()-1);

			std::string LBOpcode;
			std::string UBOpcode;

			bool opcodeEmpty=true;
			bool noLower=true;
			bool noUpper=true;

				for(PE * tempPE:temporalPEList)
				{
					FU* fu = static_cast<FU*>(tempPE->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					DFGNode* mappedOP = dp->getMappedNode();

					if(mappedOP && !((mappedOP->op).empty()))
					{
						if(lastOpcode != mappedOP->op)
							opcodeCount++;
						lastOpcode = mappedOP->op;

						if((tempPE->T < currT) && (tempPE->T >LB_T))
						{
							LB_T = tempPE->T;
							LBOpcode = mappedOP->op;
							noLower=false;
						}

						if((tempPE->T > currT) && (tempPE->T < UB_T))
						{
							UB_T = tempPE->T;
							UBOpcode = mappedOP->op;
							noUpper=false;
						}
						opcodeEmpty=false;
					}
				}

				if(opcodeCount < spatialPE->opcode_mem_size)
				{
					//if(opcodeCount >=4)
					//	std::cout << "OPCODECOUNT :: " << opcodeCount << "\n";
					if(!opcodeEmpty){
						if(!noLower && !noUpper)
						{
							int d1 = currT - LB_T;
							int d2 = UB_T - currT;

							if(!(LBOpcode.compare(UBOpcode)) && !(LBOpcode.compare(node->op))){
								cost =0;
							}
							else if(LBOpcode == UBOpcode){
								cost = this->opcode_cost/(d1) + this->opcode_cost/(d2) + this->opcode_cost;
							}
							else if((LBOpcode == node->op) && (UBOpcode != node->op)){
								cost = this->opcode_cost/(d2);
							}
							else if((LBOpcode.compare(node->op)) && !(UBOpcode.compare(node->op))){
								cost = this->opcode_cost/(d1);
							}
							else{
								cost = this->opcode_cost/(d1) + this->opcode_cost/(d2);
							}
						}
						else if(!noLower)
						{
							int d1 = currT - LB_T;

							if(!LBOpcode.compare(node->op)){
								cost =0;
							}
							else{
								cost = this->opcode_cost/(d1);
							}
						}
						else if(!noUpper)
						{
							int d2 = UB_T - currT;
							if(!UBOpcode.compare(node->op)){
								cost =0;
							}
							else{
								cost = this->opcode_cost/(d2);
							}
						}
						else {
							cost =0;
						}
					}else
					{
						cost = this->opcode_cost/5;
					}
				}else{
				//	cost =100000;
					continue;
				}

		}else{
			cost=0;
		}
		opcodeCostAddedCandidates[candDP]=cost;

	}

	return opcodeCostAddedCandidates;
}


std::map<std::string, std::string> CGRAXMLCompile::PathFinderMapper::getConnectedPorts(
		CGRAXMLCompile::PE * pe){
	//std::map<Port *, Port *> res;
	std::map<std::string, std::string> res;

					Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
					Port* easto = pe->getOutPort("EAST_O"); assert(easto);
					Port* westo = pe->getOutPort("WEST_O"); assert(westo);
					Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

					DFGNode* north_o_node = northo->getNode();
					DFGNode* east_o_node = easto->getNode();
					DFGNode* west_o_node = westo->getNode();
					DFGNode* south_o_node = southo->getNode();
					FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					Port* i1_ip = fu->getInPort("DP0_I1"); assert(i1_ip);
					Port* i2_ip = fu->getInPort("DP0_I2"); assert(i2_ip);
					Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);

					DFGNode * currentMappedOP = dp->getMappedNode();

					if(north_o_node){

						if(north_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

							res["NORTH_O"]="NORTH_XBARI";
						}
						else if(north_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){

							res["NORTH_O"]="EAST_XBARI";
						}
						else if(north_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){

							res["NORTH_O"]="WEST_XBARI";
						}
						else if(north_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){

							res["NORTH_O"]="SOUTH_XBARI";
						}
						else if(north_o_node == pe->getSingleRegPort("TREG_RI")->getNode() && northo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){

							res["NORTH_O"]="TREG_RI";
						}
						else if(north_o_node == fu->getOutPort("DP0_T")->getNode() && northo->getLat() == fu->getOutPort("DP0_T")->getLat()){

							res["NORTH_O"]="DP0_T";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
							PE* prevGotPE = this->cgra->getPrevPE(pe);
							FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

							Port* northoP = prevGotPE->getOutPort("NORTH_O"); assert(northoP);
							DFGNode* north_o_nodeP = northoP->getNode();
							bool predi_condition= false;

							if(north_o_nodeP){
							std::vector<DFGNode *> parentNodes = north_o_nodeP->parents;


							for(DFGNode* node1:parentNodes)
							{
								if((node1->getOPtype(north_o_nodeP) == "P"))
								{
									predi_condition = true;
								}
							}
							if(north_o_nodeP && (north_o_nodeP == north_o_node)  && !predi_condition ){
								if(north_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && northoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
									res["NORTH_O"]="TREG_RI";
								}
								else if(north_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && northoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
									res["NORTH_O"]="DP0_T";
								}else{
								//	std::cout << "Port : " << northo->getFullName() << ",node = " << northo->getNode()->idx << ", source not found!\n";
								}
							}
							}
#endif
						}
					}


					if(east_o_node){

						if(east_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["EAST_O"]="NORTH_XBARI";
						}
						else if(east_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["EAST_O"]="EAST_XBARI";
						}
						else if(east_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){

							res["EAST_O"]="WEST_XBARI";
						}
						else if(east_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){

							res["EAST_O"]="SOUTH_XBARI";
						}
						else if(east_o_node == pe->getSingleRegPort("TREG_RI")->getNode() && easto->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){

							res["EAST_O"]="TREG_RI";
						}
						else if(east_o_node == fu->getOutPort("DP0_T")->getNode() && easto->getLat() == fu->getOutPort("DP0_T")->getLat()){

							res["EAST_O"]="DP0_T";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
							PE* prevGotPE = this->cgra->getPrevPE(pe);
							FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

							Port* eastoP = prevGotPE->getOutPort("EAST_O"); assert(eastoP);
							DFGNode* east_o_nodeP = eastoP->getNode();
							bool predi_condition= false;

							if(east_o_nodeP){
							std::vector<DFGNode *> parentNodes = east_o_nodeP->parents;


							for(DFGNode* node1:parentNodes)
							{
								if((node1->getOPtype(east_o_nodeP) == "P"))
								{
									predi_condition = true;
								}
							}
							if(east_o_nodeP && (east_o_nodeP == east_o_node)  && !predi_condition){
								if(east_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && eastoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
									res["EAST_O"]="TREG_RI";
								}
								else if(east_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && eastoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
									res["EAST_O"]="DP0_T";
								}else{
									//std::cout << "Port : " << easto->getFullName() << ",node = " << easto->getNode()->idx << ", source not found!\n";
								}
							}
							}
#endif
						}

					}
					//else{
											//insF.easto = "111";
					//}

					if(west_o_node){

						if(west_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

							res["WEST_O"]="NORTH_XBARI";
						}
						else if(west_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){

							res["WEST_O"]="EAST_XBARI";
						}
						else if(west_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){

							res["WEST_O"]="WEST_XBARI";
						}
						else if(west_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){

							res["WEST_O"]="SOUTH_XBARI";
						}
						else if(west_o_node ==  pe->getSingleRegPort("TREG_RI")->getNode() && westo->getLat() ==  pe->getSingleRegPort("TREG_RI")->getLat()){

							res["WEST_O"]="TREG_RI";
						}
						else if(west_o_node == fu->getOutPort("DP0_T")->getNode() && westo->getLat() == fu->getOutPort("DP0_T")->getLat()){

							res["WEST_O"]="DP0_T";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
							PE* prevGotPE = this->cgra->getPrevPE(pe);
							FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

							Port* westoP = prevGotPE->getOutPort("WEST_O"); assert(westoP);
							DFGNode* west_o_nodeP = westoP->getNode();
							bool predi_condition= false;

							if(west_o_nodeP){
							std::vector<DFGNode *> parentNodes = west_o_nodeP->parents;

							for(DFGNode* node1:parentNodes)
							{
								if((node1->getOPtype(west_o_nodeP) == "P"))
								{
									predi_condition = true;
								}
							}
							}
							if(west_o_nodeP && (west_o_nodeP == west_o_node) && !predi_condition){
								if(west_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && westoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
									res["WEST_O"]="TREG_RI";
								}
								else if(west_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && westoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
									res["WEST_O"]="DP0_T";
								}else{
									//std::cout << "Port : " << westo->getFullName() << ",node = " << westo->getNode()->idx << ", source not found!\n";
								}
							}
#endif
						}
					}
					//else{
											//insF.westo = "111";
					//}

					if(south_o_node){

						if(south_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

							res["SOUTH_O"]="NORTH_XBARI";
						}
						else if(south_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){

							res["SOUTH_O"]="EAST_XBARI";
						}
						else if(south_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){

							res["SOUTH_O"]="WEST_XBARI";
						}
						else if(south_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){

							res["SOUTH_O"]="SOUTH_XBARI";
						}
						else if(south_o_node == pe->getSingleRegPort("TREG_RI")->getNode() && southo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){

							res["SOUTH_O"]="TREG_RI";
						}
						else if(south_o_node == fu->getOutPort("DP0_T")->getNode() && southo->getLat() == fu->getOutPort("DP0_T")->getLat()){

							res["SOUTH_O"]="DP0_T";
						}
						else{
//THILINI: Streaming interconnects
#ifdef STREAMING
							PE* prevGotPE = this->cgra->getPrevPE(pe);
							FU* prevGotFU = static_cast<FU*>(prevGotPE->getSubMod("FU0")); assert(prevGotFU);

							Port* southoP = prevGotPE->getOutPort("SOUTH_O"); assert(southoP);
							DFGNode* south_o_nodeP = southoP->getNode();
							bool predi_condition= false;
							if(south_o_nodeP){
							std::vector<DFGNode *> parentNodes = south_o_nodeP->parents;

							for(DFGNode* node1:parentNodes)
							{
								if((node1->getOPtype(south_o_nodeP) == "P"))
								{
									predi_condition = true;
								}
							}
							if(south_o_nodeP && (south_o_nodeP == south_o_node) && !predi_condition){
								if(south_o_nodeP == prevGotPE->getSingleRegPort("TREG_RI")->getNode() && southoP->getLat() == prevGotPE->getSingleRegPort("TREG_RI")->getLat()){
									res["SOUTH_O"]="TREG_RI";
								}
								else if(south_o_nodeP == prevGotFU->getOutPort("DP0_T")->getNode() && southoP->getLat() == prevGotFU->getOutPort("DP0_T")->getLat()){
									res["SOUTH_O"]="DP0_T";
								}else{
								//	std::cout << "Port : " << southo->getFullName() << ",node = " << southo->getNode()->idx << ", source not found!\n";
								}
							}
							}
#endif
						}
					}
					//else{
											//insF.southo = "111";
					//}


					if(p_ip->getNode()){
						if(p_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

							res["DP0_P"]="NORTH_XBARI";
						}
						else if(p_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){

							res["DP0_P"]="EAST_XBARI";
						}
						else if(p_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){

							res["DP0_P"]="WEST_XBARI";
						}
						else if(p_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){

							res["DP0_P"]="SOUTH_XBARI";
						}
						else if(p_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() && p_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){

							res["DP0_P"]="TREG_RI";
						}
						else if(p_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && p_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){

							res["DP0_P"]="DP0_T";
						}

						else{

						}
					}

					if(i1_ip->getNode()){
						if(i1_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

							res["DP0_I1"]="NORTH_XBARI";
							if(currentMappedOP &&currentMappedOP->type_i1i2){

								res["DP0_I2"]="NORTH_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){

							res["DP0_I1"]="EAST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){

								res["DP0_I2"]="EAST_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){

							res["DP0_I1"]="WEST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){

								res["DP0_I2"]="WEST_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){

							res["DP0_I1"]="SOUTH_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){

								res["DP0_I2"]="SOUTH_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() && i1_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){

							res["DP0_I1"]="TREG_RI";
							if(currentMappedOP && currentMappedOP->type_i1i2){

								res["DP0_I2"]="TREG_RI";
							}
						}
						else if(i1_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && i1_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){

							res["DP0_I1"]="DP0_T";
							if(currentMappedOP && currentMappedOP->type_i1i2){

								res["DP0_I2"]="DP0_T";
							}
						}
						else if(i1_ip->getNode() == dp->getOutPort("T")->getNode() && i1_ip->getLat() == dp->getOutPort("T")->getLat()){

							if(currentMappedOP && currentMappedOP->type_i1i2){

							}
						}
						else{

							if(currentMappedOP && currentMappedOP->type_i1i2){

							}
						}
					}

					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
						if(i2_ip->getNode()){
							if(i2_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){

								res["DP0_I2"]="NORTH_XBARI";
							}
							else if(i2_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){

								res["DP0_I2"]="EAST_XBARI";
							}
							else if(i2_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){

								res["DP0_I2"]="WEST_XBARI";
							}
							else if(i2_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){

								res["DP0_I2"]="SOUTH_XBARI";
							}
							else if(i2_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() && i2_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){

								res["DP0_I2"]="TREG_RI";
							}
							else if(i2_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && i2_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){

								res["DP0_I2"]="DP0_T";
							}

							else{

							}
						}

				}


	return res;
}
std::map<std::string, std::string> CGRAXMLCompile::PathFinderMapper::getConnectedPorts4(
		CGRAXMLCompile::PE * pe){

	std::map<std::string, std::string> res;

					Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
					Port* easto = pe->getOutPort("EAST_O"); assert(easto);
					Port* westo = pe->getOutPort("WEST_O"); assert(westo);
					Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

					DFGNode* north_o_node = northo->getNode();
					DFGNode* east_o_node = easto->getNode();
					DFGNode* west_o_node = westo->getNode();
					DFGNode* south_o_node = southo->getNode();
					FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
#ifdef DUAL_STREAMING
					Port* i1_lsb_ip = fu->getInPort("DP0_I1_LSB"); assert(i1_lsb_ip);
					Port* i1_msb_ip = fu->getInPort("DP0_I1_MSB"); assert(i1_msb_ip);
					Port* i2_lsb_ip = fu->getInPort("DP0_I2_LSB"); assert(i2_lsb_ip);
					Port* i2_msb_ip = fu->getInPort("DP0_I2_MSB"); assert(i2_msb_ip);
#else
					Port* i1_ip = fu->getInPort("DP0_I1"); assert(i1_ip);
					//Port* i1_msb_ip = fu->getInPort("DP0_I1_MSB"); assert(i1_msb_ip);
					Port* i2_ip = fu->getInPort("DP0_I2"); assert(i2_ip);
					//Port* i2_msb_ip = fu->getInPort("DP0_I2_MSB"); assert(i2_msb_ip);

#endif
					Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);

					DFGNode * currentMappedOP = dp->getMappedNode();

					if(north_o_node){

						if(north_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["NORTH_O"]="NORTH_XBARI";
						}
						else if(north_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["NORTH_O"]="EAST_XBARI";
						}
						else if(north_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["NORTH_O"]="WEST_XBARI";
						}
						else if(north_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && northo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["NORTH_O"]="SOUTH_XBARI";
						}
#ifdef DUAL_STREAMING

						else if(north_o_node == pe->getSingleRegPort("TREG_LSB_RI")->getNode() && northo->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
							res["NORTH_O"]="TREG_LSB_RI";
						}
						else if(north_o_node == pe->getSingleRegPort("TREG_MSB_RI")->getNode() && northo->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat()){
							res["NORTH_O"]="TREG_MSB_RI";
						}
						else if(north_o_node == fu->getOutPort("DP0_T_LSB")->getNode() && northo->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
							res["NORTH_O"]="DP0_T_LSB";
						}
						else if(north_o_node == fu->getOutPort("DP0_T_MSB")->getNode() && northo->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()){
							res["NORTH_O"]="DP0_T_MSB";
						}
#else
						else if(north_o_node == pe->getSingleRegPort("TREG_RI")->getNode() && northo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							res["NORTH_O"]="TREG_RI";
						}
						else if(north_o_node == fu->getOutPort("DP0_T")->getNode() && northo->getLat() == fu->getOutPort("DP0_T")->getLat()){
							res["NORTH_O"]="DP0_T";
						}
#endif
						else{
							res["NORTH_O"]="INV";
						}
					}

					if(east_o_node){
						if(east_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["EAST_O"]="NORTH_XBARI";
						}
						else if(east_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["EAST_O"]="EAST_XBARI";
						}
						else if(east_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["EAST_O"]="WEST_XBARI";
						}
						else if(east_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && easto->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["EAST_O"]="SOUTH_XBARI";
						}
#ifdef DUAL_STREAMING

						else if(east_o_node == pe->getSingleRegPort("TREG_LSB_RI")->getNode() && easto->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
							res["EAST_O"]="TREG_LSB_RI";
						}
						else if(east_o_node == pe->getSingleRegPort("TREG_MSB_RI")->getNode() && easto->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat()){
							res["EAST_O"]="TREG_MSB_RI";
						}
						else if(east_o_node == fu->getOutPort("DP0_T_LSB")->getNode() && easto->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
							res["EAST_O"]="DP0_T_LSB";
						}
						else if(east_o_node == fu->getOutPort("DP0_T_MSB")->getNode() && easto->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()){
							res["EAST_O"]="DP0_T_MSB";
						}
#else
						else if(east_o_node == pe->getSingleRegPort("TREG_RI")->getNode() && easto->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							res["EAST_O"]="TREG_RI";
						}
						else if(east_o_node == fu->getOutPort("DP0_T")->getNode() && easto->getLat() == fu->getOutPort("DP0_T")->getLat()){
							res["EAST_O"]="DP0_T";
						}

#endif
						else{
							res["EAST_O"]="INV";

						}

					}

					if(west_o_node){
						if(west_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["WEST_O"]="NORTH_XBARI";
						}
						else if(west_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["WEST_O"]="EAST_XBARI";
						}
						else if(west_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["WEST_O"]="WEST_XBARI";
						}
						else if(west_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && westo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["WEST_O"]="SOUTH_XBARI";
						}
#ifdef DUAL_STREAMING
						else if(west_o_node ==  pe->getSingleRegPort("TREG_LSB_RI")->getNode() && westo->getLat() ==  pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
							res["WEST_O"]="TREG_LSB_RI";
						}
						else if(west_o_node ==  pe->getSingleRegPort("TREG_MSB_RI")->getNode() && westo->getLat() ==  pe->getSingleRegPort("TREG_MSB_RI")->getLat()){
							res["WEST_O"]="TREG_MSB_RI";
						}
						else if(west_o_node == fu->getOutPort("DP0_T_LSB")->getNode() && westo->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
							res["WEST_O"]="DP0_T_LSB";
						}
						else if(west_o_node == fu->getOutPort("DP0_T_MSB")->getNode() && westo->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()){
							res["WEST_O"]="DP0_T_MSB";
						}
#else
						else if(west_o_node ==  pe->getSingleRegPort("TREG_RI")->getNode() && westo->getLat() ==  pe->getSingleRegPort("TREG_RI")->getLat()){
							res["WEST_O"]="TREG_RI";
						}
						else if(west_o_node == fu->getOutPort("DP0_T")->getNode() && westo->getLat() == fu->getOutPort("DP0_T")->getLat()){
							res["WEST_O"]="DP0_T";
						}
						
#endif
						else{
							res["WEST_O"]="INV";

						}
					}

					if(south_o_node){

						if(south_o_node == pe->getInternalPort("NORTH_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["SOUTH_O"]="NORTH_XBARI";
						}
						else if(south_o_node == pe->getInternalPort("EAST_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["SOUTH_O"]="EAST_XBARI";
						}
						else if(south_o_node == pe->getInternalPort("WEST_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["SOUTH_O"]="WEST_XBARI";
						}
						else if(south_o_node == pe->getInternalPort("SOUTH_XBARI")->getNode() && southo->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["SOUTH_O"]="SOUTH_XBARI";
						}
#ifdef DUAL_STREAMING

						else if(south_o_node == pe->getSingleRegPort("TREG_LSB_RI")->getNode() && southo->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
							res["SOUTH_O"]="TREG_LSB_RI";
						}
						else if(south_o_node == pe->getSingleRegPort("TREG_MSB_RI")->getNode() && southo->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat()){
							res["SOUTH_O"]="TREG_MSB_RI";
						}
						else if(south_o_node == fu->getOutPort("DP0_T_LSB")->getNode() && southo->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
							res["SOUTH_O"]="DP0_T_LSB";
						}
						else if(south_o_node == fu->getOutPort("DP0_T_MSB")->getNode() && southo->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()){
							res["SOUTH_O"]="DP0_T_MSB";
						}
#else
						else if(south_o_node == pe->getSingleRegPort("TREG_RI")->getNode() && southo->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							res["SOUTH_O"]="TREG_RI";
						}
						else if(south_o_node == fu->getOutPort("DP0_T")->getNode() && southo->getLat() == fu->getOutPort("DP0_T")->getLat()){
							res["SOUTH_O"]="DP0_T";
						}
#endif
						else{
							res["SOUTH_O"]="INV";
						}
					}


					if(p_ip->getNode()){
						if(p_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["DP0_P"]="NORTH_XBARI";
						}
						else if(p_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["DP0_P"]="EAST_XBARI";
						}
						else if(p_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["DP0_P"]="WEST_XBARI";
						}
						else if(p_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && p_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["DP0_P"]="SOUTH_XBARI";
						}
#ifdef DUAL_STREAMING

						else if(p_ip->getNode() == pe->getSingleRegPort("TREG_LSB_RI")->getNode() && p_ip->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
							res["DP0_P"]="TREG_LSB_RI";
						}
						else if(p_ip->getNode() == pe->getSingleRegPort("TREG_MSB_RI")->getNode() && p_ip->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat()){
							res["DP0_P"]="TREG_MSB_RI";
						}
						else if(p_ip->getNode() == fu->getOutPort("DP0_T_LSB")->getNode() && p_ip->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
							res["DP0_P"]="DP0_T_LSB";
						}
						else if(p_ip->getNode() == fu->getOutPort("DP0_T_MSB")->getNode() && p_ip->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()){
							res["DP0_P"]="DP0_T_MSB";
						}
#else
						else if(p_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() && p_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							res["DP0_P"]="TREG_RI";
						}
						else if(p_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && p_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
							res["DP0_P"]="DP0_T";
						}
#endif
						else{
							res["DP0_P"]="INV";
						}
					}

#ifdef DUAL_STREAMING
					if(i1_lsb_ip->getNode()){
						if(i1_lsb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i1_lsb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["DP0_I1_LSB"]="NORTH_XBARI";
							if(currentMappedOP &&currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="NORTH_XBARI";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i1_lsb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["DP0_I1_LSB"]="EAST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="EAST_XBARI";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i1_lsb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["DP0_I1_LSB"]="WEST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="WEST_XBARI";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i1_lsb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["DP0_I1_LSB"]="SOUTH_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="SOUTH_XBARI";
							}
						}
						else if(i1_lsb_ip->getNode() == pe->getSingleRegPort("TREG_LSB_RI")->getNode() && i1_lsb_ip->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
							res["DP0_I1_LSB"]="TREG_LSB_RI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="TREG_LSB_RI";
							}
						}
						else if(i1_lsb_ip->getNode() == fu->getOutPort("DP0_T_LSB")->getNode() && i1_lsb_ip->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
							res["DP0_I1_LSB"]="DP0_T_LSB";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="DP0_T_LSB";
							}
						}
						else if(i1_lsb_ip->getNode() == dp->getOutPort("T_LSB")->getNode() && i1_lsb_ip->getLat() == dp->getOutPort("T_LSB")->getLat()){
							res["DP0_I1_LSB"]="DP0_T_LSB";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="DP0_T_LSB";
							}
						}else{
							res["DP0_I2_LSB"]="INV";
						}
					}

					if(i1_msb_ip->getNode()){
						if(i1_msb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i1_msb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["DP0_I1_MSB"]="NORTH_XBARI";
							if(currentMappedOP &&currentMappedOP->type_i1i2){
								res["DP0_I2_MSB"]="NORTH_XBARI";
							}
						}
						else if(i1_msb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i1_msb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["DP0_I1_MSB"]="EAST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_MSB"]="EAST_XBARI";
							}
						}
						else if(i1_msb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i1_msb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["DP0_I1_MSB"]="WEST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_MSB"]="WEST_XBARI";
							}
						}
						else if(i1_msb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i1_msb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["DP0_I1_MSB"]="SOUTH_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_MSB"]="SOUTH_XBARI";
							}
						}
						else if(i1_msb_ip->getNode() == pe->getSingleRegPort("TREG_MSB_RI")->getNode() && i1_msb_ip->getLat() == pe->getSingleRegPort("TREG_MSB_RI")->getLat()){
							res["DP0_I1_MSB"]="TREG_MSB_RI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_MSB"]="TREG_MSB_RI";
							}
						}
						else if(i1_msb_ip->getNode() == fu->getOutPort("DP0_T_MSB")->getNode() && i1_msb_ip->getLat() == fu->getOutPort("DP0_T_MSB")->getLat()){
							res["DP0_I1_MSB"]="DP0_T_MSB";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_MSB"]="DP0_T_MSB";
							}
						}
						else if(i1_msb_ip->getNode() == dp->getOutPort("T_MSB")->getNode() && i1_msb_ip->getLat() == dp->getOutPort("T_MSB")->getLat()){
							res["DP0_I1_MSB"]="DP0_T_MSB";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_MSB"]="DP0_T_MSB";
							}
						}else{
							res["DP0_I1_MSB"]="INV";
						}
					}

					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
						if(i2_lsb_ip->getNode()){
							if(i2_lsb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i2_lsb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
								res["DP0_I2_LSB"]="NORTH_XBARI";
							}
							else if(i2_lsb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i2_lsb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
								res["DP0_I2_LSB"]="EAST_XBARI";
							}
							else if(i2_lsb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i2_lsb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
								res["DP0_I2_LSB"]="WEST_XBARI";
							}
							else if(i2_lsb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i2_lsb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
								res["DP0_I2_LSB"]="SOUTH_XBARI";
							}
							else if(i2_lsb_ip->getNode() == pe->getSingleRegPort("TREG_LSB_RI")->getNode() && i2_lsb_ip->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
								res["DP0_I2_LSB"]="TREG_LSB_RI";
							}
							else if(i2_lsb_ip->getNode() == fu->getOutPort("DP0_T_LSB")->getNode() && i2_lsb_ip->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
								res["DP0_I2_LSB"]="DP0_T_LSB";
							}
							else{
								res["DP0_I2_LSB"]="INV";

							}
						}
					}

					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
						if(i2_msb_ip->getNode()){
							if(i2_msb_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i2_msb_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
								res["DP0_I2_MSB"]="NORTH_XBARI";
							}
							else if(i2_msb_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i2_msb_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
								res["DP0_I2_MSB"]="EAST_XBARI";
							}
							else if(i2_msb_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i2_msb_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
								res["DP0_I2_MSB"]="WEST_XBARI";
							}
							else if(i2_msb_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i2_msb_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
								res["DP0_I2_MSB"]="SOUTH_XBARI";
							}
							else if(i2_msb_ip->getNode() == pe->getSingleRegPort("TREG_MSB_RI")->getNode() && i2_msb_ip->getLat() == pe->getSingleRegPort("TREG_LSB_RI")->getLat()){
								res["DP0_I2_MSB"]="TREG_LSB_RI";
							}
							else if(i2_msb_ip->getNode() == fu->getOutPort("DP0_T_MSB")->getNode() && i2_msb_ip->getLat() == fu->getOutPort("DP0_T_LSB")->getLat()){
								res["DP0_I2_MSB"]="DP0_T_LSB";
							}
							else{

								res["DP0_I2_MSB"]="INV";
							}
						}
					}
#else
					if(i1_ip->getNode()){
						if(i1_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
							res["DP0_I1"]="NORTH_XBARI";
							if(currentMappedOP &&currentMappedOP->type_i1i2){
								res["DP0_I2"]="NORTH_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
							res["DP0_I1"]="EAST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2"]="EAST_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
							res["DP0_I1"]="WEST_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2"]="WEST_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i1_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
							res["DP0_I1"]="SOUTH_XBARI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2"]="SOUTH_XBARI";
							}
						}
						else if(i1_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() && i1_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
							res["DP0_I1"]="TREG_RI";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2"]="TREG_RI";
							}
						}
						else if(i1_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && i1_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
							res["DP0_I1"]="DP0_T";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2_LSB"]="DP0_T";
							}
						}
						else if(i1_ip->getNode() == dp->getOutPort("T")->getNode() && i1_ip->getLat() == dp->getOutPort("T")->getLat()){
							res["DP0_I1"]="DP0_T";
							if(currentMappedOP && currentMappedOP->type_i1i2){
								res["DP0_I2"]="DP0_T";
							}
						}else{
							res["DP0_I2"]="INV";
						}
					}

				
					if(!(currentMappedOP && currentMappedOP->type_i1i2)){
						if(i2_ip->getNode()){
							if(i2_ip->getNode() == pe->getInternalPort("NORTH_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("NORTH_XBARI")->getLat()){
								res["DP0_I2"]="NORTH_XBARI";
							}
							else if(i2_ip->getNode() == pe->getInternalPort("EAST_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("EAST_XBARI")->getLat()){
								res["DP0_I2"]="EAST_XBARI";
							}
							else if(i2_ip->getNode() == pe->getInternalPort("WEST_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("WEST_XBARI")->getLat()){
								res["DP0_I2"]="WEST_XBARI";
							}
							else if(i2_ip->getNode() == pe->getInternalPort("SOUTH_XBARI")->getNode() && i2_ip->getLat() == pe->getInternalPort("SOUTH_XBARI")->getLat()){
								res["DP0_I2"]="SOUTH_XBARI";
							}
							else if(i2_ip->getNode() == pe->getSingleRegPort("TREG_RI")->getNode() && i2_ip->getLat() == pe->getSingleRegPort("TREG_RI")->getLat()){
								res["DP0_I2"]="TREG_RI";
							}
							else if(i2_ip->getNode() == fu->getOutPort("DP0_T")->getNode() && i2_ip->getLat() == fu->getOutPort("DP0_T")->getLat()){
								res["DP0_I2"]="DP0_T";
							}
							else{
								res["DP0_I2"]="INV";

							}
						}
					}

#endif



	return res;
}
std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::removeConstFilledCandidates(
		std::vector<CGRAXMLCompile::DataPath *> candDestIn, std::map<CGRAXMLCompile::DataPath *,int> constDest, std::map<CGRAXMLCompile::DataPath *,int> opcodeDests){
	std::vector<DataPath *> res;

	std::vector<DataPath *> resConst;
	std::vector<DataPath *> resOpcode;
	for (std::pair<DataPath *, int> pair : constDest)
	{
		resConst.push_back(pair.first);
	}

	for (std::pair<DataPath *, int> pair : opcodeDests)
	{
			resOpcode.push_back(pair.first);
	}

	for(DataPath *candDP: candDestIn)
	{
		if((std::find(resConst.begin(),resConst.end(),candDP)!=resConst.end()) && (std::find(resOpcode.begin(),resOpcode.end(),candDP)!=resOpcode.end()))
			res.push_back(candDP);

	}
 return res;
}

int CGRAXMLCompile::PathFinderMapper::getCrossbarSimilarityCost(LatPort src,
													LatPort next_to_src)
{
	int cost=LARGE_VALUE;

	int x= next_to_src.second->getPE()->X;
	int y= next_to_src.second->getPE()->Y;

	int currT = next_to_src.second->getPE()->T;
	vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);

	bool class_A_active=false;
	bool class_B_active=false;

	bool class_A_active_prev=false;
	bool class_B_active_prev=false;
	bool class_A_active_next=false;
	bool class_B_active_next=false;

	PE * pe = next_to_src.second->getPE();
	PE * prevPE = this->cgra->getPrevPE(pe);
	PE * nextPE = this->cgra->getNextPE(pe);
	std::string currPString = next_to_src.second->getName();
	FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
	DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

	FU* prevfu = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevfu);
	DataPath* prevdp = static_cast<DataPath*>(prevfu->getSubMod("DP0")); assert(prevdp);

	Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
	Port* easto = pe->getOutPort("EAST_O"); assert(easto);
	Port* westo = pe->getOutPort("WEST_O"); assert(westo);
	Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

	Port* i1_ip = fu->getInPort("DP0_I1"); assert(i1_ip);
	Port* i2_ip = fu->getInPort("DP0_I2"); assert(i2_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);


	std::map<std::string, std::string> currentPEmap = getConnectedPorts(pe);
	std::map<std::string, std::string> prevPEmap = getConnectedPorts(prevPE);
	std::map<std::string, std::string> nextPEmap = getConnectedPorts(nextPE);

	if(!pe->is_hot)
	{
		if((!(currentPEmap["NORTH_O"]==prevPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==prevPEmap["EAST_O"]) && !(next_to_src.second == easto)) or (!(currentPEmap["DP0_I2"]==prevPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
				{
					class_A_active = true;
					class_A_active_prev = true;
				}
		if((!(currentPEmap["SOUTH_O"]==prevPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==prevPEmap["WEST_O"]) && !(next_to_src.second == westo)) or (!(currentPEmap["DP0_P"]==prevPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
				{
					class_B_active = true;
					class_B_active_prev = true;
				}

		if((!(currentPEmap["NORTH_O"]==nextPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==nextPEmap["EAST_O"]) && !(next_to_src.second == easto)) or (!(currentPEmap["DP0_I2"]==nextPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
				{
					class_A_active = true;
					class_A_active_next = true;
				}
		if((!(currentPEmap["SOUTH_O"]==nextPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==nextPEmap["WEST_O"]) && !(next_to_src.second == westo)) or (!(currentPEmap["DP0_P"]==nextPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
				{
					class_B_active = true;
					class_B_active_next = true;
				}
	}
	else
	{
		if((!(currentPEmap["NORTH_O"]==prevPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==prevPEmap["EAST_O"]) && !(next_to_src.second == easto)))
				{
					class_A_active = true;
					class_A_active_prev = true;
				}
		if((!(currentPEmap["SOUTH_O"]==prevPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==prevPEmap["WEST_O"]) && !(next_to_src.second == westo)))
				{
					class_B_active = true;
					class_B_active_prev = true;
				}

		if((!(currentPEmap["NORTH_O"]==nextPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==nextPEmap["EAST_O"]) && !(next_to_src.second == easto)))
				{
					class_A_active = true;
					class_A_active_next = true;
				}
		if((!(currentPEmap["SOUTH_O"]==nextPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==nextPEmap["WEST_O"]) && !(next_to_src.second == westo)))
				{
					class_B_active = true;
					class_B_active_next = true;
				}
	}

	//decide the class of the current classA(1) classB(0)
	bool isClassA;
	if(!pe->is_hot)
	{

		if(next_to_src.second == northo or next_to_src.second == easto  or next_to_src.second == i2_ip)
		{
			isClassA = true;
		}

		if(next_to_src.second == southo or next_to_src.second == westo  or next_to_src.second == p_ip)
		{
			isClassA = false;
		}
	}else
	{

		if(next_to_src.second == northo or next_to_src.second == easto)
		{
			isClassA = true;
		}

		if(next_to_src.second == southo or next_to_src.second == westo)
		{
			isClassA = false;
		}
	}

	if((class_A_active_prev or class_A_active_next) && !class_B_active_prev && !class_B_active_next)
	{

		if(!pe->is_hot)
		{
			if(isClassA or next_to_src.second == i1_ip)
			{
				cost =0;
			}else
			{
				cost = 10000000;
			}
		}else
		{
			if(isClassA or next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
			{
				cost =0;
			}else
			{
				cost = 10000000;
			}
		}
	}


	else if((class_B_active_prev or class_B_active_next) && !class_A_active_prev && !class_A_active_next)
	{

		if(!pe->is_hot)
		{
			if(!isClassA or next_to_src.second == i1_ip)
			{
				cost =0;
			}else
			{
				cost = 10000000;
			}
		}else
		{
			if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
			{
				cost =0;
			}else
			{
				cost = 10000000;
			}
		}
	}
	else if((class_B_active_prev && class_A_active_next) or (class_B_active_next && class_A_active_prev))
	{

				cost = 10000000;
	}

	else if(!class_A_active_prev && !class_B_active_prev && !class_A_active_next && !class_B_active_next)
	{
		PE * LB_classA_PE;
		PE * UB_classA_PE;

		PE * LB_classB_PE;
		PE * UB_classB_PE;

		PE * LB_PE;
		PE * UB_PE;

		int LB_T=0;
		int UB_T=(this->cgra->get_t_max()-1);

		int LB_classA_T=0;
		int UB_classA_T=(this->cgra->get_t_max()-1);

		int LB_classB_T=0;
		int UB_classB_T=(this->cgra->get_t_max()-1);

		bool noLower=true;
		bool noUpper=true;

		for(PE * tempPE:temporalPEList)
		{
			FU* tempfu = static_cast<FU*>(tempPE->getSubMod("FU0")); assert(tempfu);

			Port* prev_northo = tempPE->getOutPort("NORTH_O"); assert(prev_northo);
			Port* prev_easto = tempPE->getOutPort("EAST_O"); assert(prev_easto);
			Port* prev_westo = tempPE->getOutPort("WEST_O"); assert(prev_westo);
			Port* prev_southo = tempPE->getOutPort("SOUTH_O"); assert(prev_southo);

			Port* prev_i1_ip = tempfu->getInPort("DP0_I1"); assert(prev_i1_ip);
			Port* prev_i2_ip = tempfu->getInPort("DP0_I2"); assert(prev_i2_ip);
			Port* prev_p_ip = tempfu->getInPort("DP0_P"); assert(prev_p_ip);

			if( (tempPE->T < currT) && (tempPE->T >= LB_T))
			{
				LB_T = tempPE->T;
				LB_PE = tempPE;
				noLower =false;
			}

			if( (tempPE->T > currT) && (tempPE->T <= UB_T))
			{
				UB_T = tempPE->T;
				UB_PE = tempPE;
				noUpper =false;
			}
		}
		DFGNode * prevXbarNode;
		DFGNode * prevOutNode;
		Port * prevo;

		DFGNode * nextXbarNode;
		DFGNode * nextOutNode;
		Port * nexto;

			if(!noLower)
			{
				if(src.second == pe->getSingleRegPort("TREG_RI"))
				{
					prevXbarNode = LB_PE->getSingleRegPort(src.second->getName())->getNode();
				}
				else if(src.second == fu->getOutPort("DP0_T"))
				{
					FU* lbfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(lbfu);
					DataPath* lbdp = static_cast<DataPath*>(lbfu->getSubMod("DP0")); assert(lbdp);
					prevXbarNode = lbfu->getOutPort(src.second->getName())->getNode();
				}
				else
				{
					prevXbarNode = LB_PE->getInternalPort(src.second->getName())->getNode();
				}

				if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
				{
					FU* tempfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(tempfu);
					DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
					prevo =tempfu->getInPort(next_to_src.second->getName()); assert(prevo);

				}
				else
				{
					prevo =LB_PE->getOutPort(next_to_src.second->getName()); assert(prevo);
				}
				prevOutNode = prevo->getNode();
			}

			if(!noUpper)
			{
				if(src.second == pe->getSingleRegPort("TREG_RI"))
				{
					nextXbarNode = UB_PE->getSingleRegPort(src.second->getName())->getNode();
				}
				else if(src.second == fu->getOutPort("DP0_T"))
				{
					FU* ubfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(ubfu);
					DataPath* ubdp = static_cast<DataPath*>(ubfu->getSubMod("DP0")); assert(ubdp);
					nextXbarNode = ubfu->getOutPort(src.second->getName())->getNode();
				}
				else
				{
					nextXbarNode = UB_PE->getInternalPort(src.second->getName())->getNode();
				}

				if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
				{
					FU* tempfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(tempfu);
					//std::cout << "NextPort  = " << next_to_src.second->getName() << "\n";
					DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
					nexto =tempfu->getInPort(next_to_src.second->getName()); assert(nexto);

				}
				else
				{
					nexto =UB_PE->getOutPort(next_to_src.second->getName()); assert(nexto);
				}
				nextOutNode = nexto->getNode();
			}

			if(!noLower && !noUpper)
			{
				if((prevXbarNode==prevOutNode) && (nextXbarNode == nextOutNode))
				{
					cost =0;
				}
				else if((prevXbarNode==prevOutNode))
				{
					cost = 100/(currT - LB_T);
				}
				else if((nextXbarNode == nextOutNode))
				{
					cost = 100/(UB_T-currT);
				}
				else
				{
					cost = 1000/(currT - LB_T) +100/(UB_T-currT);
				}

			}
			else if(!noLower)
			{
				if((prevXbarNode==prevOutNode))
				{
					cost =0;
				}
				else
				{
					cost = 1000/(currT - LB_T);
				}

			}
			else if(!noUpper)
			{
				if((nextXbarNode == nextOutNode))
				{
					cost = 0;
				}
				else
				{
					cost = 1000/(UB_T-currT);
				}
			}
			else
			{
				cost =0;
			}

	}
	else
	{
		if(!pe->is_hot)
		{
			if((!(currentPEmap["DP0_I1"]==prevPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip))or (!(currentPEmap["DP0_I1"]==nextPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip)))
			{
				cost =0;
			}
			else
			{
				cost = 20000000;
			}
		}else
		{

			if((!(currentPEmap["DP0_I1"]==prevPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip))or (!(currentPEmap["DP0_I1"]==nextPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip)))
			{
				cost =0;
			}
			else if((!(currentPEmap["DP0_I2"]==prevPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip))or (!(currentPEmap["DP0_I2"]==nextPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
			{
				cost =0;
			}
			else if((!(currentPEmap["DP0_P"]==prevPEmap["DP0_P"]) && !(next_to_src.second == p_ip))or (!(currentPEmap["DP0_P"]==nextPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
			{
				cost =0;
			}
			else
			{
				//std::cout << "INVALID:: BOTH CLASSES ACTIVE";
				cost = 20000000;
			}
		}
	}


	return cost;
}

int CGRAXMLCompile::PathFinderMapper::getCrossbarSimilarityCostwithSim(LatPort src,
													LatPort next_to_src)
{
	int cost=LARGE_VALUE;

	int x= next_to_src.second->getPE()->X;
	int y= next_to_src.second->getPE()->Y;

	int currT = next_to_src.second->getPE()->T;
	vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);

	bool class_A_active=false;
	bool class_B_active=false;

	bool class_A_active_prev=false;
	bool class_B_active_prev=false;
	bool class_A_active_prev_prev=false;
	bool class_B_active_prev_prev=false;
	bool class_A_active_next=false;
	bool class_B_active_next=false;

	PE * pe = next_to_src.second->getPE();
	PE * prevPE = this->cgra->getPrevPE(pe);
	PE * prevPrevPE = this->cgra->getPrevPE(prevPE);
	PE * nextPE = this->cgra->getNextPE(pe);
	std::string currPString = next_to_src.second->getName();
	FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
	DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

	FU* prevfu = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevfu);
	DataPath* prevdp = static_cast<DataPath*>(prevfu->getSubMod("DP0")); assert(prevdp);

	Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
	Port* easto = pe->getOutPort("EAST_O"); assert(easto);
	Port* westo = pe->getOutPort("WEST_O"); assert(westo);
	Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

	Port* i1_ip = fu->getInPort("DP0_I1"); assert(i1_ip);
	Port* i2_ip = fu->getInPort("DP0_I2"); assert(i2_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);


	std::map<std::string, std::string> currentPEmap = getConnectedPorts(pe);
	std::map<std::string, std::string> prevPEmap = getConnectedPorts(prevPE);
	std::map<std::string, std::string> prevPrevPEmap = getConnectedPorts(prevPrevPE);
	std::map<std::string, std::string> nextPEmap = getConnectedPorts(nextPE);


	if(!pe->is_hot)
	{
		if((!(prevPrevPEmap["NORTH_O"]==prevPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(prevPrevPEmap["EAST_O"]==prevPEmap["EAST_O"]) && !(next_to_src.second == easto)) or (!(prevPrevPEmap["DP0_I2"]==prevPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
				{
					//class_A_active = true;
					class_A_active_prev_prev = true;
				}
		if((!(prevPrevPEmap["SOUTH_O"]==prevPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(prevPrevPEmap["WEST_O"]==prevPEmap["WEST_O"]) && !(next_to_src.second == westo)) or (!(prevPrevPEmap["DP0_P"]==prevPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
				{
					//class_B_active = true;
					class_B_active_prev_prev = true;
				}
		if((!(currentPEmap["NORTH_O"]==prevPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==prevPEmap["EAST_O"]) && !(next_to_src.second == easto)) or (!(currentPEmap["DP0_I2"]==prevPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
				{
					class_A_active = true;
					class_A_active_prev = true;
				}
		if((!(currentPEmap["SOUTH_O"]==prevPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==prevPEmap["WEST_O"]) && !(next_to_src.second == westo)) or (!(currentPEmap["DP0_P"]==prevPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
				{
					class_B_active = true;
					class_B_active_prev = true;
				}

		if((!(currentPEmap["NORTH_O"]==nextPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==nextPEmap["EAST_O"]) && !(next_to_src.second == easto)) or (!(currentPEmap["DP0_I2"]==nextPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
				{
					class_A_active = true;
					class_A_active_next = true;
				}
		if((!(currentPEmap["SOUTH_O"]==nextPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==nextPEmap["WEST_O"]) && !(next_to_src.second == westo)) or (!(currentPEmap["DP0_P"]==nextPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
				{
					class_B_active = true;
					class_B_active_next = true;
				}
	}
	else
	{
		if((!(prevPrevPEmap["NORTH_O"]==prevPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(prevPrevPEmap["EAST_O"]==prevPEmap["EAST_O"]) && !(next_to_src.second == easto)))
				{
					//class_A_active = true;
					class_A_active_prev_prev = true;
				}
		if((!(prevPrevPEmap["SOUTH_O"]==prevPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(prevPrevPEmap["WEST_O"]==prevPEmap["WEST_O"]) && !(next_to_src.second == westo)))
				{
					//class_B_active = true;
					class_B_active_prev_prev = true;
				}
		if((!(currentPEmap["NORTH_O"]==prevPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==prevPEmap["EAST_O"]) && !(next_to_src.second == easto)))
				{
					class_A_active = true;
					class_A_active_prev = true;
				}
		if((!(currentPEmap["SOUTH_O"]==prevPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==prevPEmap["WEST_O"]) && !(next_to_src.second == westo)))
				{
					class_B_active = true;
					class_B_active_prev = true;
				}

		if((!(currentPEmap["NORTH_O"]==nextPEmap["NORTH_O"]) && !(next_to_src.second == northo)) or (!(currentPEmap["EAST_O"]==nextPEmap["EAST_O"]) && !(next_to_src.second == easto)))
				{
					class_A_active = true;
					class_A_active_next = true;
				}
		if((!(currentPEmap["SOUTH_O"]==nextPEmap["SOUTH_O"]) && !(next_to_src.second == southo)) or (!(currentPEmap["WEST_O"]==nextPEmap["WEST_O"]) && !(next_to_src.second == westo)))
				{
					class_B_active = true;
					class_B_active_next = true;
				}
	}

	//decide the class of the current classA(1) classB(0)
	bool isClassA;
	if(!pe->is_hot)
	{

		if(next_to_src.second == northo or next_to_src.second == easto  or next_to_src.second == i2_ip)
		{
			isClassA = true;
		}

		if(next_to_src.second == southo or next_to_src.second == westo  or next_to_src.second == p_ip)
		{
			isClassA = false;
		}
	}else
	{

		if(next_to_src.second == northo or next_to_src.second == easto)
		{
			isClassA = true;
		}

		if(next_to_src.second == southo or next_to_src.second == westo)
		{
			isClassA = false;
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//appending similarity costs
	bool eq_to_prev = false;
	bool eq_to_next = false;
	bool prev_null = false;
	bool next_null = false;

		if((class_A_active_prev_prev && isClassA) or (class_B_active_prev_prev && !isClassA))
		{
			if(!prevPEmap[currPString].empty())
			{
				prev_null =false;
				if(!prevPEmap[currPString].compare(src.second->getName()))
				{
					eq_to_prev =true;
				}
				else
				{
					eq_to_prev =false;
				}
			}else
			{
			prev_null = true;
			}
		} else if ((class_A_active_next && isClassA) or (class_B_active_next && !isClassA))
		{
			if(!nextPEmap[currPString].empty())
			{
				next_null =false;
				if(!nextPEmap[currPString].compare(src.second->getName()))
				{
					eq_to_next =true;
				}
				else
				{
					eq_to_next =false;
				}
			} else
			{
			next_null = true;
			}
		}else if ((class_A_active_prev && !isClassA))
		{
					prev_null =false;
					if(next_to_src.second == southo)
					{
						if(!prevPEmap["NORTH_O"].compare(src.second->getName()))
						{
							eq_to_prev =true;
						}
						else
						{
							eq_to_prev =false;
						}
					}
					else if(next_to_src.second == westo)
					{
						if(!prevPEmap["EAST_O"].compare(src.second->getName()))
						{
							eq_to_prev =true;
						}
						else
						{
							eq_to_prev =false;
						}
					}

					if(!pe->is_hot)
					{
						if(next_to_src.second == i2_ip)
						{
							if(!prevPEmap["DP0_I2"].compare(src.second->getName()))
							{
								eq_to_prev =true;
							}
							else
							{
								eq_to_prev =false;
							}
						}
					}
		}else if ((class_B_active_prev && isClassA))
		{
					prev_null =false;
					if(next_to_src.second == northo)
					{
						if(!prevPEmap["SOUTH_O"].compare(src.second->getName()))
						{
							eq_to_prev =true;
						}
						else
						{
							eq_to_prev =false;
						}
					}
					else if(next_to_src.second == easto)
					{
						if(!prevPEmap["WEST_O"].compare(src.second->getName()))
						{
							eq_to_prev =true;
						}
						else
						{
							eq_to_prev =false;
						}
					}

					if(!pe->is_hot)
					{
						if(next_to_src.second == p_ip)
						{
							if(!prevPEmap["DP0_P"].compare(src.second->getName()))
							{
								eq_to_prev =true;
							}
							else
							{
								eq_to_prev =false;
							}
						}
					}
		}else if ((class_A_active_next && !isClassA))
		{
					next_null =false;
					if(next_to_src.second == southo)
					{
						if(!nextPEmap["NORTH_O"].compare(src.second->getName()))
						{
							eq_to_next =true;
						}
						else
						{
							eq_to_next =false;
						}
					}
					else if(next_to_src.second == westo)
					{
						if(!nextPEmap["EAST_O"].compare(src.second->getName()))
						{
							eq_to_next =true;
						}
						else
						{
							eq_to_next =false;
						}
					}

					if(!pe->is_hot)
					{
						if(next_to_src.second == i2_ip)
						{
							if(!nextPEmap["DP0_I2"].compare(src.second->getName()))
							{
								eq_to_next =true;
							}
							else
							{
								eq_to_next =false;
							}
						}
					}
		}else if ((class_B_active_next && isClassA))
		{
					next_null =false;
					if(next_to_src.second == northo)
					{
						if(!nextPEmap["SOUTH_O"].compare(src.second->getName()))
						{
							eq_to_next =true;
						}
						else
						{
							eq_to_next =false;
						}
					}
					else if(next_to_src.second == easto)
					{
						if(!nextPEmap["WEST_O"].compare(src.second->getName()))
						{
							eq_to_next =true;
						}
						else
						{
							eq_to_next =false;
						}
					}

					if(!pe->is_hot)
					{
						if(next_to_src.second == p_ip)
						{
							if(!nextPEmap["DP0_P"].compare(src.second->getName()))
							{
								eq_to_next =true;
							}
							else
							{
								eq_to_next =false;
							}
						}
					}
		}else if(!class_B_active_next && !class_A_active_next)
		{
			if((next_to_src.second == northo) or (next_to_src.second == southo) )
			{
				if((!nextPEmap["SOUTH_O"].compare(src.second->getName())) or (!nextPEmap["NORTH_O"].compare(src.second->getName())))
				{
					eq_to_next =true;
				}
				else
				{
					eq_to_next =false;
				}
			}else if((next_to_src.second == westo) or (next_to_src.second == easto) )
			{
				if((!nextPEmap["WEST_O"].compare(src.second->getName())) or (!nextPEmap["EAST_O"].compare(src.second->getName())))
				{
					eq_to_next =true;
				}
				else
				{
					eq_to_next =false;
				}
			}else if((!pe->is_hot) && ((next_to_src.second == i2_ip) or (next_to_src.second == p_ip) ) )
			{
				if((!nextPEmap["DP0_I2"].compare(src.second->getName())) or (!nextPEmap["DP0_P"].compare(src.second->getName())))
				{
					eq_to_next =true;
				}
				else
				{
					eq_to_next =false;
				}
			}
		}else if(!class_A_active_prev && !class_B_active_prev)
		{
			if((next_to_src.second == northo) or (next_to_src.second == southo) )
			{
				if((!prevPEmap["SOUTH_O"].compare(src.second->getName())) or (!prevPEmap["NORTH_O"].compare(src.second->getName())))
				{
					eq_to_prev =true;
				}
				else
				{
					eq_to_prev =false;
				}
			}else if((next_to_src.second == westo) or (next_to_src.second == easto) )
			{
				if((!prevPEmap["WEST_O"].compare(src.second->getName())) or (!prevPEmap["EAST_O"].compare(src.second->getName())))
				{
					eq_to_prev =true;
				}
				else
				{
					eq_to_prev =false;
				}
			}else if((!pe->is_hot) && ((next_to_src.second == i2_ip) or (next_to_src.second == p_ip) ) )
			{
				if((!prevPEmap["DP0_I2"].compare(src.second->getName())) or (!prevPEmap["DP0_P"].compare(src.second->getName())))
				{
					eq_to_prev =true;
				}
				else
				{
					eq_to_prev =false;
				}
			}
		}

	if((class_A_active_prev or class_A_active_next) && !class_B_active_prev && !class_B_active_next)
	{

		if(!pe->is_hot)
		{
			if(isClassA or next_to_src.second == i1_ip)
			{
				if(eq_to_prev && eq_to_next)
				{
					cost =0;
				}else if(eq_to_prev or eq_to_next)
				{
					cost = 10;
				}else
				{
					cost = 100;
				}
			}else
			{
				cost = 10000000;
			}
		}else
		{
			if(isClassA or next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
			{
				if(eq_to_prev && eq_to_next)
				{
					cost =0;
				}else if(eq_to_prev or eq_to_next)
				{
					cost = 10;
				}else
				{
					cost = 100;
				}
			}else
			{
				cost = 10000000;
			}
		}
	}


	else if((class_B_active_prev or class_B_active_next) && !class_A_active_prev && !class_A_active_next)
	{

		if(!pe->is_hot)
		{
			if(!isClassA or next_to_src.second == i1_ip)
			{
				if(eq_to_prev && eq_to_next)
				{
					cost =0;
				}else if(eq_to_prev or eq_to_next)
				{
					cost = 10;
				}else
				{
					cost = 100;
				}
			}else
			{
				cost = 10000000;
			}
		}else
		{
			if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
			{
				if(eq_to_prev && eq_to_next)
				{
					cost =0;
				}else if(eq_to_prev or eq_to_next)
				{
					cost = 10;
				}else
				{
					cost = 100;
				}
			}else
			{
				cost = 10000000;
			}
		}
	}
	else if((class_B_active_prev && class_A_active_next) or (class_B_active_next && class_A_active_prev))
	{

				cost = 10000000;
	}

	else if(!class_A_active_prev && !class_B_active_prev && !class_A_active_next && !class_B_active_next)
	{
		PE * LB_classA_PE;
		PE * UB_classA_PE;

		PE * LB_classB_PE;
		PE * UB_classB_PE;

		PE * LB_PE;
		PE * UB_PE;

		int LB_T=0;
		int UB_T=(this->cgra->get_t_max()-1);

		int LB_classA_T=0;
		int UB_classA_T=(this->cgra->get_t_max()-1);

		int LB_classB_T=0;
		int UB_classB_T=(this->cgra->get_t_max()-1);

		bool noLower=true;
		bool noUpper=true;

		for(PE * tempPE:temporalPEList)
		{
			FU* tempfu = static_cast<FU*>(tempPE->getSubMod("FU0")); assert(tempfu);

			Port* prev_northo = tempPE->getOutPort("NORTH_O"); assert(prev_northo);
			Port* prev_easto = tempPE->getOutPort("EAST_O"); assert(prev_easto);
			Port* prev_westo = tempPE->getOutPort("WEST_O"); assert(prev_westo);
			Port* prev_southo = tempPE->getOutPort("SOUTH_O"); assert(prev_southo);

			Port* prev_i1_ip = tempfu->getInPort("DP0_I1"); assert(prev_i1_ip);
			Port* prev_i2_ip = tempfu->getInPort("DP0_I2"); assert(prev_i2_ip);
			Port* prev_p_ip = tempfu->getInPort("DP0_P"); assert(prev_p_ip);

			if( (tempPE->T < currT) && (tempPE->T >= LB_T))
			{
				LB_T = tempPE->T;
				LB_PE = tempPE;
				noLower =false;
			}

			if( (tempPE->T > currT) && (tempPE->T <= UB_T))
			{
				UB_T = tempPE->T;
				UB_PE = tempPE;
				noUpper =false;
			}
		}
		DFGNode * prevXbarNode;
		DFGNode * prevOutNode;
		Port * prevo;

		DFGNode * nextXbarNode;
		DFGNode * nextOutNode;
		Port * nexto;

			if(!noLower)
			{
				if(src.second == pe->getSingleRegPort("TREG_RI"))
				{
					prevXbarNode = LB_PE->getSingleRegPort(src.second->getName())->getNode();
				}
				else if(src.second == fu->getOutPort("DP0_T"))
				{
					FU* lbfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(lbfu);
					DataPath* lbdp = static_cast<DataPath*>(lbfu->getSubMod("DP0")); assert(lbdp);
					prevXbarNode = lbfu->getOutPort(src.second->getName())->getNode();
				}
				else
				{
					prevXbarNode = LB_PE->getInternalPort(src.second->getName())->getNode();
				}

				if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
				{
					FU* tempfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(tempfu);
					DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
					prevo =tempfu->getInPort(next_to_src.second->getName()); assert(prevo);

				}
				else
				{
					prevo =LB_PE->getOutPort(next_to_src.second->getName()); assert(prevo);
				}
				prevOutNode = prevo->getNode();
			}

			if(!noUpper)
			{
				if(src.second == pe->getSingleRegPort("TREG_RI"))
				{
					nextXbarNode = UB_PE->getSingleRegPort(src.second->getName())->getNode();
				}
				else if(src.second == fu->getOutPort("DP0_T"))
				{
					FU* ubfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(ubfu);
					DataPath* ubdp = static_cast<DataPath*>(ubfu->getSubMod("DP0")); assert(ubdp);
					nextXbarNode = ubfu->getOutPort(src.second->getName())->getNode();
				}
				else
				{
					nextXbarNode = UB_PE->getInternalPort(src.second->getName())->getNode();
				}

				if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
				{
					FU* tempfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(tempfu);
					//std::cout << "NextPort  = " << next_to_src.second->getName() << "\n";
					DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
					nexto =tempfu->getInPort(next_to_src.second->getName()); assert(nexto);

				}
				else
				{
					nexto =UB_PE->getOutPort(next_to_src.second->getName()); assert(nexto);
				}
				nextOutNode = nexto->getNode();
			}

			if(!noLower && !noUpper)
			{
				if((prevXbarNode==prevOutNode) && (nextXbarNode == nextOutNode))
				{
					cost =0;
				}
				else if((prevXbarNode==prevOutNode))
				{
					cost = 100/(currT - LB_T);
				}
				else if((nextXbarNode == nextOutNode))
				{
					cost = 100/(UB_T-currT);
				}
				else
				{
					cost = 1000/(currT - LB_T) +100/(UB_T-currT);
				}

			}
			else if(!noLower)
			{
				if((prevXbarNode==prevOutNode))
				{
					cost =0;
				}
				else
				{
					cost = 1000/(currT - LB_T);
				}

			}
			else if(!noUpper)
			{
				if((nextXbarNode == nextOutNode))
				{
					cost = 0;
				}
				else
				{
					cost = 1000/(UB_T-currT);
				}
			}
			else
			{
				cost =0;
			}

	}
	else
	{
		if(!pe->is_hot)
		{
			if((!(currentPEmap["DP0_I1"]==prevPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip))or (!(currentPEmap["DP0_I1"]==nextPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip)))
			{
				cost =0;
			}
			else
			{
				cost = 10000000;
			}
		}else
		{

			if((!(currentPEmap["DP0_I1"]==prevPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip))or (!(currentPEmap["DP0_I1"]==nextPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip)))
			{
				cost =0;
			}
			else if((!(currentPEmap["DP0_I2"]==prevPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip))or (!(currentPEmap["DP0_I2"]==nextPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
			{
				cost =0;
			}
			else if((!(currentPEmap["DP0_P"]==prevPEmap["DP0_P"]) && !(next_to_src.second == p_ip))or (!(currentPEmap["DP0_P"]==nextPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
			{
				cost =0;
			}
			else
			{

				cost = 10000000;
			}
		}
	}


	return cost;
}


int CGRAXMLCompile::PathFinderMapper::getCrossbarSimilarityCostHetero(LatPort src,
													LatPort next_to_src,bool multiCycle)
{
	int cost=ROUTER_COST;

	int x= next_to_src.second->getPE()->X;
	int y= next_to_src.second->getPE()->Y;

	int currT = next_to_src.second->getPE()->T;
	vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);


	bool class_A_active_prev=false;
	bool class_B_active_prev=false;
	bool class_A_active_prev_prev=false;
	bool class_B_active_prev_prev=false;
	bool class_A_active_next=false;
	bool class_B_active_next=false;


	bool class_A_active_next_next=false;
	bool class_B_active_next_next=false;


	PE * pe = next_to_src.second->getPE();
	Module * mod = next_to_src.second->getMod();
	PE * prevPE = this->cgra->getPrevPE(pe);
	PE * prevPrevPE = this->cgra->getPrevPE(prevPE);
	PE * nextPE = this->cgra->getNextPE(pe);
	std::string currPString = next_to_src.second->getName();
	FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
	DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

	FU* prevfu = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevfu);
	DataPath* prevdp = static_cast<DataPath*>(prevfu->getSubMod("DP0")); assert(prevdp);

	Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
	Port* easto = pe->getOutPort("EAST_O"); assert(easto);
	Port* westo = pe->getOutPort("WEST_O"); assert(westo);
	Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

	Port* i1_ip = fu->getInPort("DP0_I1"); assert(i1_ip);
	Port* i2_ip = fu->getInPort("DP0_I2"); assert(i2_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);


	std::map<std::string, std::string> currentPEmap = getConnectedPorts(pe);
	std::map<std::string, std::string> prevPEmap = getConnectedPorts(prevPE);
	std::map<std::string, std::string> prevPrevPEmap = getConnectedPorts(prevPrevPE);
	std::map<std::string, std::string> nextPEmap = getConnectedPorts(nextPE);


	PE * nextNextPE = this->cgra->getNextPE(nextPE);
	std::map<std::string, std::string> nextNextPEmap = getConnectedPorts(nextNextPE);


	//remove the hardcoded PE array size
	int x_max = 4;
	int y_max = 4;

	std::vector<Port *> classAP;
	std::vector<Port *> classBP;

	bool is_corner=false;
	Port * p_corner_port;
	Port * p_common_port1;
	Port * p_common_port2;

	if((pe->X == 0) && (pe->Y == 0))
	{
		classAP.push_back(easto);
		classBP.push_back(southo);

	} else if((pe->X == 0) && (pe->Y == y_max-1))	{
		classAP.push_back(easto);
		classBP.push_back(northo);

	}else if((pe->X == x_max-1) && (pe->Y == 0))
	{
		classAP.push_back(westo);
		classBP.push_back(southo);

	}else if((pe->X == x_max-1) && (pe->Y == y_max-1))
	{
		classAP.push_back(northo);
		classBP.push_back(westo);

	}else if(pe->X == 0)
	{
		is_corner = true;
		p_corner_port = easto;
		classAP.push_back(northo);

		classBP.push_back(southo);

	}else if(pe->X == x_max-1)
	{
		is_corner = true;
		p_corner_port = westo;
		classAP.push_back(northo);

		classBP.push_back(southo);

	}else if(pe->Y == 0)
	{
		is_corner = true;
		p_corner_port = southo;
		classAP.push_back(easto);

		classBP.push_back(westo);

	}else if(pe->Y == y_max-1)
	{
		is_corner = true;
		p_corner_port = northo;
		classAP.push_back(easto);

		classBP.push_back(westo);

	}else
	{


		classAP.push_back(easto);
		classAP.push_back(northo);

		classBP.push_back(westo);
		classBP.push_back(southo);

		if(((pe->X == 2) && (pe->Y == 1)))
			classAP.push_back(southo);
		else
			classBP.push_back(i2_ip);

		if(((pe->X == 1) && (pe->Y == 2)))
			classBP.push_back(northo);
		else
			classAP.push_back(i2_ip);

		if(((pe->X == 1) && (pe->Y == 1)))
			classBP.push_back(easto);
		else
			classAP.push_back(i2_ip);

		if(((pe->X == 2) && (pe->Y == 2)))
			classAP.push_back(westo);
		else
			classBP.push_back(i2_ip);
	}

	if(!pe->is_hot)
	{
		classAP.push_back(i2_ip);
		classBP.push_back(p_ip);
	} else
	{
		if((pe->X == 2) && (pe->Y == 1))
			p_common_port1 = southo;
		else
			p_common_port1 = i2_ip;

		if((pe->X == 1) && (pe->Y == 2))
			p_common_port1 = northo;
		else
			p_common_port1 = i2_ip;

		if((pe->X == 1) && (pe->Y == 1))
			p_common_port1 = easto;
		else
			p_common_port1 = i2_ip;

		if((pe->X == 2) && (pe->Y == 2))
			p_common_port1 = westo;
		else
			p_common_port1 = i2_ip;

		p_common_port2 = p_ip;
	}

	bool isClassA;

	for (auto & element : classAP) {
	    std::string portName = element->getName();
	    if(!(prevPrevPEmap[portName]==prevPEmap[portName]) && !(next_to_src.second == element))
		{
	    	class_A_active_prev_prev = true;
		}

	    if((!(currentPEmap[portName]==prevPEmap[portName]) && !(next_to_src.second == element)))
	    {
	    	class_A_active_prev = true;
	    }

	    if((!(currentPEmap[portName]==nextPEmap[portName]) && !(next_to_src.second == element)))
	    {
	    	class_A_active_next = true;
	    }

//THILINI: Streaming interconnects
if(multiCycle)
{
	    if((!(nextNextPEmap[portName]==nextPEmap[portName]) && !(next_to_src.second == element)))
	   	    {
	   	    	class_A_active_next_next = true;
	   	    }
}

	    if(next_to_src.second == element)
	    {
	    	isClassA = true;
	    }
	}

	for (auto & element : classBP) {
		std::string portName = element->getName();
		if(!(prevPrevPEmap[portName]==prevPEmap[portName]) && !(next_to_src.second == element))
		{
			class_B_active_prev_prev = true;
		}

		if((!(currentPEmap[portName]==prevPEmap[portName]) && !(next_to_src.second == element)))
		{
	    	class_B_active_prev = true;
	    //	std::cout << "THILINI_B: " << portName << "\n";
		}

		if((!(currentPEmap[portName]==nextPEmap[portName]) && !(next_to_src.second == element)))
		{
	    	class_B_active_next = true;
		}

//THILINI: Streaming interconnects
if(multiCycle)
{
		if((!(nextNextPEmap[portName]==nextPEmap[portName]) && !(next_to_src.second == element)))
		{
			   class_B_active_next_next = true;
		}
}

		if(next_to_src.second == element)
		{
			isClassA = false;
		}
	}


//THILINI: Streaming interconnects
if(!multiCycle or (next_to_src.second == i1_ip or next_to_src.second == p_ip or next_to_src.second == i2_ip))
{

	if((class_A_active_prev or class_A_active_next) && !class_B_active_prev && !class_B_active_next)
	{

		if(!pe->is_hot)
		{
			if(is_corner)
			{
				if(isClassA or next_to_src.second == i1_ip or next_to_src.second == p_corner_port)
				{

					cost =0;

				}else
				{
					cost = ROUTER_COST;
				}
			}else{
				if(isClassA or next_to_src.second == i1_ip)
				{

					cost =0;

				}else
				{
					cost = ROUTER_COST;
				}
			}
		}else
		{
			if(is_corner)
			{
				if(isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2 or next_to_src.second == p_corner_port)
				{

					cost =0;

				}else
				{
					cost = ROUTER_COST;
				}
			}else{
				if(isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2)
				{

					cost =0;

				}else
				{
					cost = ROUTER_COST;
				}
			}

			}

	}

	else if((class_B_active_prev or class_B_active_next) && !class_A_active_prev && !class_A_active_next)
	{
		if(!pe->is_hot)
		{
			if(is_corner)
			{
			if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == p_corner_port)
			{

					cost =0;
			}else
			{
				cost = ROUTER_COST;
			}
			}else{
				if(!isClassA or next_to_src.second == i1_ip)
				{

						cost =0;

				}else
				{
					cost = ROUTER_COST;
				}
			}
		}else
		{
			if(is_corner)
			{
			if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2 or next_to_src.second == p_corner_port)
			{

					cost =0;

			}else
			{
				cost = ROUTER_COST;
			}
			}else
			{
				if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2)
				{

						cost =0;

				}else
				{
					cost = ROUTER_COST;
				}
			}
		}
	}
	else if((class_B_active_prev && class_A_active_next) or (class_B_active_next && class_A_active_prev))
	{

				cost = ROUTER_COST;
	}
	else if(!class_A_active_prev && !class_B_active_prev && !class_A_active_next && !class_B_active_next)
	{

		PE * LB_classA_PE;
		PE * UB_classA_PE;

		PE * LB_classB_PE;
		PE * UB_classB_PE;

		PE * LB_PE;
		PE * UB_PE;

		int LB_T=0;
		int UB_T=(this->cgra->get_t_max()-1);

		int LB_classA_T=0;
		int UB_classA_T=(this->cgra->get_t_max()-1);

		int LB_classB_T=0;
		int UB_classB_T=(this->cgra->get_t_max()-1);

		bool noLower=true;
		bool noUpper=true;

		for(PE * tempPE:temporalPEList)
		{
			FU* tempfu = static_cast<FU*>(tempPE->getSubMod("FU0")); assert(tempfu);

			Port* prev_northo = tempPE->getOutPort("NORTH_O"); assert(prev_northo);
			Port* prev_easto = tempPE->getOutPort("EAST_O"); assert(prev_easto);
			Port* prev_westo = tempPE->getOutPort("WEST_O"); assert(prev_westo);
			Port* prev_southo = tempPE->getOutPort("SOUTH_O"); assert(prev_southo);

			Port* prev_i1_ip = tempfu->getInPort("DP0_I1"); assert(prev_i1_ip);
			Port* prev_i2_ip = tempfu->getInPort("DP0_I2"); assert(prev_i2_ip);
			Port* prev_p_ip = tempfu->getInPort("DP0_P"); assert(prev_p_ip);

			if( (tempPE->T < currT) && (tempPE->T >= LB_T))
			{
				LB_T = tempPE->T;
				LB_PE = tempPE;
				noLower =false;
			}

			if( (tempPE->T > currT) && (tempPE->T <= UB_T))
			{
				UB_T = tempPE->T;
				UB_PE = tempPE;
				noUpper =false;
			}
		}
		DFGNode * prevXbarNode;
		DFGNode * prevOutNode;
		Port * prevo;

		DFGNode * nextXbarNode;
		DFGNode * nextOutNode;
		Port * nexto;

			if(!noLower)
			{
				if(src.second == pe->getSingleRegPort("TREG_RI"))
				{
					prevXbarNode = LB_PE->getSingleRegPort(src.second->getName())->getNode();
				}
				else if(src.second == fu->getOutPort("DP0_T"))
				{
					FU* lbfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(lbfu);
					DataPath* lbdp = static_cast<DataPath*>(lbfu->getSubMod("DP0")); assert(lbdp);
					prevXbarNode = lbfu->getOutPort(src.second->getName())->getNode();
				}
				else
				{
					prevXbarNode = LB_PE->getInternalPort(src.second->getName())->getNode();
				}

				if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
				{
					FU* tempfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(tempfu);
					DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
					prevo =tempfu->getInPort(next_to_src.second->getName()); assert(prevo);

				}
				else
				{
					prevo =LB_PE->getOutPort(next_to_src.second->getName()); assert(prevo);
				}
				prevOutNode = prevo->getNode();
			}

			if(!noUpper)
			{
				if(src.second == pe->getSingleRegPort("TREG_RI"))
				{
					nextXbarNode = UB_PE->getSingleRegPort(src.second->getName())->getNode();
				}
				else if(src.second == fu->getOutPort("DP0_T"))
				{
					FU* ubfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(ubfu);
					DataPath* ubdp = static_cast<DataPath*>(ubfu->getSubMod("DP0")); assert(ubdp);
					nextXbarNode = ubfu->getOutPort(src.second->getName())->getNode();
				}
				else
				{
					nextXbarNode = UB_PE->getInternalPort(src.second->getName())->getNode();
				}

				if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
				{
					FU* tempfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(tempfu);
					DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
					nexto =tempfu->getInPort(next_to_src.second->getName()); assert(nexto);

				}
				else
				{
					nexto =UB_PE->getOutPort(next_to_src.second->getName()); assert(nexto);
				}
				nextOutNode = nexto->getNode();
			}

			if(!noLower && !noUpper)
			{
				if((prevXbarNode==prevOutNode) && (nextXbarNode == nextOutNode))
				{
					cost =0;
				}
				else if((prevXbarNode==prevOutNode))
				{
					cost = 100/(currT - LB_T);
				}
				else if((nextXbarNode == nextOutNode))
				{
					cost = 100/(UB_T-currT);
				}
				else
				{
					cost = 1000/(currT - LB_T) +100/(UB_T-currT);
				}

			}
			else if(!noLower)
			{
				if((prevXbarNode==prevOutNode))
				{
					cost =0;
				}
				else
				{
					cost = 1000/(currT - LB_T);
				}

			}
			else if(!noUpper)
			{
				if((nextXbarNode == nextOutNode))
				{
					cost = 0;
				}
				else
				{
					cost = 1000/(UB_T-currT);
				}
			}
			else
			{
				cost =0;
			}

	}
	else
	{

		if(!pe->is_hot)
		{
			if((!(currentPEmap["DP0_I1"]==prevPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip))or (!(currentPEmap["DP0_I1"]==nextPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip)))
			{
				cost =0;
			}
			else
			{
				cost = ROUTER_COST;
			}
		}else
		{

			if((!(currentPEmap["DP0_I1"]==prevPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip))or (!(currentPEmap["DP0_I1"]==nextPEmap["DP0_I1"]) && !(next_to_src.second == i1_ip)))
			{
				cost =0;
			}
			else if((!(currentPEmap["DP0_I2"]==prevPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip))or (!(currentPEmap["DP0_I2"]==nextPEmap["DP0_I2"]) && !(next_to_src.second == i2_ip)))
			{
				cost =0;
			}
			else if((!(currentPEmap["DP0_P"]==prevPEmap["DP0_P"]) && !(next_to_src.second == p_ip))or (!(currentPEmap["DP0_P"]==nextPEmap["DP0_P"]) && !(next_to_src.second == p_ip)))
			{
				cost =0;
			}
			else
			{
				cost = ROUTER_COST;
			}
		}
	}
}else{

		if((class_A_active_prev or class_A_active_next) && !class_B_active_prev && !class_B_active_next)
			{

			if(!class_B_active_next_next)
			{
				if(!pe->is_hot)
				{
					if(is_corner)
					{
						if(isClassA or next_to_src.second == i1_ip or next_to_src.second == p_corner_port)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}else{
						if(isClassA or next_to_src.second == i1_ip)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}
				}else
				{
					if(is_corner)
					{
						if(isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2 or next_to_src.second == p_corner_port)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}else{
						if(isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}

				}
			}else {
				cost = ROUTER_COST;
			}
		}

		else if((class_B_active_prev or class_B_active_next) && !class_A_active_prev && !class_A_active_next)
		{
			if(!class_A_active_next_next)
			{
				if(!pe->is_hot)
				{
					if(is_corner)
					{
						if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == p_corner_port)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}else{
						if(!isClassA or next_to_src.second == i1_ip)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}
				}else
				{
					if(is_corner)
					{
						if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2 or next_to_src.second == p_corner_port)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}else
					{
						if(!isClassA or next_to_src.second == i1_ip or next_to_src.second == p_common_port1 or next_to_src.second == p_common_port2)
						{
							cost =0;
						}else
						{
							cost = ROUTER_COST;
						}
					}
				}
			}else {
				cost = ROUTER_COST;
			}
		}
		else if((class_B_active_prev && class_A_active_next) or (class_B_active_next && class_A_active_prev))
		{
					cost = ROUTER_COST;
		}else if((!class_A_active_prev && !class_B_active_prev && !class_A_active_next && !class_B_active_next) && (class_B_active_next_next && class_A_active_next_next))
		{
					cost = ROUTER_COST;
		}else if((!class_A_active_prev && !class_B_active_prev && !class_A_active_next && !class_B_active_next) && (class_B_active_next_next && !class_A_active_next_next))
		{
				if(is_corner)
				{
					if(!isClassA or next_to_src.second == p_corner_port)
					{
						cost =0;
					}else
					{
						cost = ROUTER_COST;
					}
				}else{
					if(!isClassA)
					{
						cost =0;
					}else
					{
						cost = ROUTER_COST;
					}
				}

		}else if((!class_A_active_prev && !class_B_active_prev && !class_A_active_next && !class_B_active_next) && (!class_B_active_next_next && class_A_active_next_next))
		{
				if(is_corner)
				{
					if(isClassA or next_to_src.second == p_corner_port)
					{
						cost =0;
					}else
					{
						cost = ROUTER_COST;
					}
				}else{
					if(isClassA)
					{
						cost =0;
					}else
					{
						cost = ROUTER_COST;
					}
				}
		}
		else if(!class_A_active_prev && !class_B_active_prev && !class_A_active_next && !class_B_active_next && !class_B_active_next_next && !class_A_active_next_next)
		{
			PE * LB_classA_PE;
			PE * UB_classA_PE;

			PE * LB_classB_PE;
			PE * UB_classB_PE;

			PE * LB_PE;
			PE * UB_PE;

			int LB_T=0;
			int UB_T=(this->cgra->get_t_max()-1);

			int LB_classA_T=0;
			int UB_classA_T=(this->cgra->get_t_max()-1);

			int LB_classB_T=0;
			int UB_classB_T=(this->cgra->get_t_max()-1);

			bool noLower=true;
			bool noUpper=true;

			for(PE * tempPE:temporalPEList)
			{
				FU* tempfu = static_cast<FU*>(tempPE->getSubMod("FU0")); assert(tempfu);

				Port* prev_northo = tempPE->getOutPort("NORTH_O"); assert(prev_northo);
				Port* prev_easto = tempPE->getOutPort("EAST_O"); assert(prev_easto);
				Port* prev_westo = tempPE->getOutPort("WEST_O"); assert(prev_westo);
				Port* prev_southo = tempPE->getOutPort("SOUTH_O"); assert(prev_southo);

				Port* prev_i1_ip = tempfu->getInPort("DP0_I1"); assert(prev_i1_ip);
				Port* prev_i2_ip = tempfu->getInPort("DP0_I2"); assert(prev_i2_ip);
				Port* prev_p_ip = tempfu->getInPort("DP0_P"); assert(prev_p_ip);

				if( (tempPE->T < currT) && (tempPE->T >= LB_T))
				{
					LB_T = tempPE->T;
					LB_PE = tempPE;
					noLower =false;
				}

				if( (tempPE->T > currT) && (tempPE->T <= UB_T))
				{
					UB_T = tempPE->T;
					UB_PE = tempPE;
					noUpper =false;
				}
			}
			DFGNode * prevXbarNode;
			DFGNode * prevOutNode;
			Port * prevo;

			DFGNode * nextXbarNode;
			DFGNode * nextOutNode;
			Port * nexto;

				if(!noLower)
				{
					if(src.second == pe->getSingleRegPort("TREG_RI"))
					{
						prevXbarNode = LB_PE->getSingleRegPort(src.second->getName())->getNode();
					}
					else if(src.second == fu->getOutPort("DP0_T"))
					{
						FU* lbfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(lbfu);
						DataPath* lbdp = static_cast<DataPath*>(lbfu->getSubMod("DP0")); assert(lbdp);
						prevXbarNode = lbfu->getOutPort(src.second->getName())->getNode();
					}
					else
					{
						prevXbarNode = LB_PE->getInternalPort(src.second->getName())->getNode();
					}

					if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
					{
						FU* tempfu = static_cast<FU*>(LB_PE->getSubMod("FU0")); assert(tempfu);
						DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
						prevo =tempfu->getInPort(next_to_src.second->getName()); assert(prevo);

					}
					else
					{
						prevo =LB_PE->getOutPort(next_to_src.second->getName()); assert(prevo);
					}
					prevOutNode = prevo->getNode();
				}

				if(!noUpper)
				{
					if(src.second == pe->getSingleRegPort("TREG_RI"))
					{
						nextXbarNode = UB_PE->getSingleRegPort(src.second->getName())->getNode();
					}
					else if(src.second == fu->getOutPort("DP0_T"))
					{
						FU* ubfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(ubfu);
						DataPath* ubdp = static_cast<DataPath*>(ubfu->getSubMod("DP0")); assert(ubdp);
						nextXbarNode = ubfu->getOutPort(src.second->getName())->getNode();
					}
					else
					{
						nextXbarNode = UB_PE->getInternalPort(src.second->getName())->getNode();
					}

					if(next_to_src.second == i1_ip or next_to_src.second == i2_ip or next_to_src.second == p_ip)
					{
						FU* tempfu = static_cast<FU*>(UB_PE->getSubMod("FU0")); assert(tempfu);
						DataPath* tempdp = static_cast<DataPath*>(tempfu->getSubMod("DP0")); assert(tempdp);
						nexto =tempfu->getInPort(next_to_src.second->getName()); assert(nexto);

					}
					else
					{
						nexto =UB_PE->getOutPort(next_to_src.second->getName()); assert(nexto);
					}
					nextOutNode = nexto->getNode();
				}

				if(!noLower && !noUpper)
				{
					if((prevXbarNode==prevOutNode) && (nextXbarNode == nextOutNode))
					{
						cost =0;
					}
					else if((prevXbarNode==prevOutNode))
					{
						cost = 100/(currT - LB_T);
					}
					else if((nextXbarNode == nextOutNode))
					{
						cost = 100/(UB_T-currT);
					}
					else
					{
						cost = 1000/(currT - LB_T) +100/(UB_T-currT);
					}

				}
				else if(!noLower)
				{
					if((prevXbarNode==prevOutNode))
					{
						cost =0;
					}
					else
					{
						cost = 1000/(currT - LB_T);
					}

				}
				else if(!noUpper)
				{
					if((nextXbarNode == nextOutNode))
					{
						cost = 0;
					}
					else
					{
						cost = 1000/(UB_T-currT);
					}
				}
				else
				{
					cost =0;
				}

		}
		else
		{

					cost = ROUTER_COST;

		}
}

	return cost;
}


int CGRAXMLCompile::PathFinderMapper::getCrossbarSimilarityCostDual(LatPort src,
													LatPort next_to_src,bool multiCycle)
{


	int cost=ROUTER_COST;
	int x= next_to_src.second->getPE()->X;
	int y= next_to_src.second->getPE()->Y;


	vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);

	PE * pe = next_to_src.second->getPE();
	//Module * mod = next_to_src.second->getMod();
	PE * prevPE = this->cgra->getPrevPE(pe);
	PE * prevPrevPE = this->cgra->getPrevPE(prevPE);
	PE * nextPE = this->cgra->getNextPE(pe);
	std::string currPString = next_to_src.second->getName();
	FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
	DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

	FU* prevfu = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevfu);
	DataPath* prevdp = static_cast<DataPath*>(prevfu->getSubMod("DP0")); assert(prevdp);

	Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
	Port* easto = pe->getOutPort("EAST_O"); assert(easto);
	Port* westo = pe->getOutPort("WEST_O"); assert(westo);
	Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

	Port* i1_lsb_ip = fu->getInPort("DP0_I1_LSB"); assert(i1_lsb_ip);
	Port* i1_msb_ip = fu->getInPort("DP0_I1_MSB"); assert(i1_msb_ip);
	Port* i2_lsb_ip = fu->getInPort("DP0_I2_LSB"); assert(i2_lsb_ip);
	Port* i2_msb_ip = fu->getInPort("DP0_I2_MSB"); assert(i2_msb_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);

	std::map<std::string, std::string> currentPEmap = getConnectedPorts4(pe);
	std::map<std::string, std::string> prevPEmap = getConnectedPorts4(prevPE);
	std::map<std::string, std::string> prevPrevPEmap = getConnectedPorts4(prevPrevPE);
	std::map<std::string, std::string> nextPEmap = getConnectedPorts4(nextPE);


	PE * nextNextPE = this->cgra->getNextPE(nextPE);
	std::map<std::string, std::string> nextNextPEmap = getConnectedPorts4(nextNextPE);

	//remove the hardcoded PE array size
	int x_max = 4;
	int y_max = 4;

	std::vector<std::vector<Port *>> classes;
	std::vector<Port *> class1;
	std::vector<Port *> class2;


	std::vector<Port *> classAP;
	std::vector<Port *> classBP;



#ifndef TORUS
	if((pe->X == 0) && (pe->Y == 0))
	{
		//std::cout << "Inside crossbar simi0 \n";
		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(southo);

		class2.push_back(easto);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);


	} else if((pe->X == 0) && (pe->Y == y_max-1))
	{
		//std::cout << "Inside crossbar simi1 \n";
		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(northo);


		class2.push_back(easto);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);


	}else if((pe->X == x_max-1) && (pe->Y == 0))
	{
		//std::cout << "Inside crossbar simi2 \n";
		class1.push_back(westo);
		class1.push_back(southo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);


		class2.push_back(westo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);



	}else if((pe->X == x_max-1) && (pe->Y == y_max-1))
	{
		//std::cout << "Inside crossbar simi3 \n";
		class1.push_back(westo);
		class1.push_back(northo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);

		class2.push_back(westo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);


	}else if(pe->X == 0)
	{

		class1.push_back(northo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(easto);
		class1.push_back(southo);

		class2.push_back(easto);
		class2.push_back(southo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);


	}else if(pe->X == x_max-1)
	{

		class1.push_back(northo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(westo);
		class1.push_back(southo);


		class2.push_back(westo);
		class2.push_back(southo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);


	}else if(pe->Y == 0)
	{

		class1.push_back(westo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(southo);
		class1.push_back(easto);

		class2.push_back(southo);
		class2.push_back(easto);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);


	}else if(pe->Y == y_max-1)
	{

		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(northo);
		class1.push_back(westo);

		class2.push_back(northo);
		class2.push_back(westo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

	}else
	{
#endif


		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);


		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);

		if(((pe->X == 2) && (pe->Y == 1))){
			class1.push_back(easto);
			class1.push_back(northo);
			class1.push_back(southo);

			class2.push_back(westo);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);
		}else if(((pe->X == 1) && (pe->Y == 2))){
			class1.push_back(easto);
			class1.push_back(northo);
			class1.push_back(southo);

			class2.push_back(westo);
			class2.push_back(northo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);
		}else if(((pe->X == 1) && (pe->Y == 1))){
			class1.push_back(easto);
			class1.push_back(westo);
			class1.push_back(northo);

			class2.push_back(easto);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);
		}else if(((pe->X == 2) && (pe->Y == 2))){
			class1.push_back(easto);
			class1.push_back(westo);
			class1.push_back(northo);

			class2.push_back(westo);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);
		}else{
			class1.push_back(easto);
			class1.push_back(westo);
			class1.push_back(northo);

			class2.push_back(westo);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);
		}
#ifndef TORUS
	}
#endif

	classes.push_back(class1);
	classes.push_back(class2);

	//bool isClassA;
	int size = classes.size();
	std::vector<bool> active_class_prev(size,false);
	std::vector<bool> active_class_next(size,false);
	std::vector<bool> active_class(size,false);
	std::vector<bool> curr_class(size,false);

	std::vector<std::vector<Port *>> prev_active_list;
	std::vector<std::vector<Port *>> next_active_list;

	std::vector<Port *> commonPorts;
	std::vector <Port* > class_vec0 = classes[0];
	for (Port * element : class_vec0) {
		bool is_common = true;
		for(int i=1;i<size;i++){
			if(std::find(classes[i].begin(),classes[i].end(),element) == classes[i].end())
				is_common = false;
		}
		if(is_common)
			commonPorts.push_back(element);
	}
	std::map<int,bool> allPINV;
	std::map<int,bool> allCINV;
	std::map<int,bool> allNINV;

	for(int i=0;i<size;i++){
		bool prevAllINV = true;
		bool nextAllINV = true;
		bool currAllINV = true;
		std::vector <Port* > class_vec = classes[i];
		for (Port * element : class_vec) {
			std::string portName = element->getName();
			if(currentPEmap[portName] != "INV")
				currAllINV = false;
			if(prevPEmap[portName] != "INV")
				prevAllINV = false;
			if(nextPEmap[portName] != "INV")
				nextAllINV = false;
		}
		allPINV[i]=prevAllINV;
		allNINV[i]=nextAllINV;
		allCINV[i]=currAllINV;
	}


	std::vector<int> active_index;
	for(int i=0;i<size;i++){

		std::vector <Port* > class_vec = classes[i];


		for (Port * element : class_vec) {
			std::string portName = element->getName();
			//std::cout << portName << "\n";
			if((std::find(commonPorts.begin(),commonPorts.end(),element) != commonPorts.end()) or (std::find(commonPorts.begin(),commonPorts.end(),next_to_src.second) != commonPorts.end()))
				continue;
			if((!(currentPEmap[portName]==prevPEmap[portName]) && !(next_to_src.second == element)))
			{

				if(!(!allPINV[i] && allCINV[i]))
					active_class_prev[i] = true;

			}

			if((!(currentPEmap[portName]==nextPEmap[portName]) && !(next_to_src.second == element)))
			{

				if(!(allNINV[i] && !allCINV[i]))
					active_class_next[i] = true;

			}

			if(next_to_src.second == element)
			{

				curr_class[i] = true;

			}


		}
		active_class[i] = active_class_prev[i] or active_class_next[i];

		if(active_class[i])
			active_index.push_back(i);
	}

	if(active_index.size() == 1)
	{
		if(curr_class[active_index.at(0)])
			cost =0;
		else if (std::find(commonPorts.begin(),commonPorts.end(),next_to_src.second) != commonPorts.end()){
			cost =0;
		}else {
			cost = ROUTER_COST;

		}
	}else if(active_index.size() > 1)
	{


		bool belong_to_active=false;
		for(int j:active_index)
		{
			if(curr_class[j])
				belong_to_active=true;
		}

		if(belong_to_active)
			cost =0;
		else {
			cost = ROUTER_COST;

		}
	}else if(active_index.size() == 0){
		cost =0;
	}


	if(pe->is_hot)
		cost =0;

	return cost;
}
std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::modifyUsedConnCandDest(
	std::vector<DataPath *> candDestIn, std::vector<Port *> availablePorts, bool &changed1)
{
	std::vector<DataPath *> res1;


	for (DataPath *candDP: candDestIn)
	{
		PE *candPE = candDP->getPE();
		int inUnused=0;
		int outUnused=0;
		std::vector<Port *> candPEOutP = candPE->outputPorts;
		std::vector<Port *> candPEInP = candPE->inputPorts;

		for (Port *inP:candPEInP)
		{
			std::vector<Port *>::iterator it = std::find(availablePorts.begin(),availablePorts.end(),inP);
			if(it != availablePorts.end())
			{
				inUnused++;
			}
		}
		for (Port *outP:candPEOutP)
		{
			std::vector<Port *>::iterator it = std::find(availablePorts.begin(),availablePorts.end(),outP);
			if(it != availablePorts.end())
			{
				outUnused++;
			}
		}

		if(inUnused >0 && outUnused >0)
		{
			res1.push_back(candDP);
		}

	}


	return res1;
}


std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::modifyLatencyDest(
	std::vector<DataPath *> candDestIn, DFGNode *node,int diff)
{
	std::vector<DataPath *> res2;

	std::vector<DFGNode *> parentlist = node->parents;


    DFGNode * maxLatP=NULL;
    int maxL=-1;
    bool isP;
	for(DFGNode* node1:parentlist)
	{
		if (node1->getOPtype(node) != "P")
			isP=false;
		if(node1->rootDP != NULL)
		{
			int latP = node1->rootDP->getLat();
			if(maxL < latP)
			{
				maxL = latP;
				maxLatP = node1;
			}
		}
	}

	int T_P;
	int next_t;


			T_P = maxLatP->rootDP->getPE()->T;
			next_t = (T_P + maxLatP->rootDP->getFU()->supportedOPs[maxLatP->op]) % this->cgra->get_t_max();


	for(DataPath *candDP: candDestIn)
	{
		int dest_t = candDP->getPE()->T;

		if((node->parents).empty())
		{
				res2.push_back(candDP);
				continue;
		}
		else if(((node->parents).size()==1))
		{
			for(DFGNode * parent:node->parents)
            {
				if(parent->rootDP!=NULL)
				{
					if(((parent->rootDP->getPE()->X==candDP->getPE()->X) && (parent->rootDP->getPE()->Y==candDP->getPE()->Y)) or (parent->getOPtype(node) == "P"))
					{
						res2.push_back(candDP);
					}else
					{

						if((abs(dest_t - next_t)) != 0)
						{

							res2.push_back(candDP);
						}

					}
				}
			}
		}
		else
		{

			bool samePEMax=false;
			bool diffPEMax=false;
			for(DFGNode * parent:node->parents)
			{
				if(parent->rootDP!=NULL){
					if(parent->rootDP->getLat()==maxL)
					{
						if(((parent->rootDP->getPE()->X==candDP->getPE()->X) && (parent->rootDP->getPE()->Y==candDP->getPE()->Y)))
							samePEMax=true;
						else
							diffPEMax=true;
					}
				}
			}
				if((samePEMax && !diffPEMax) or isP)
				{
					res2.push_back(candDP);
					continue;
				}else
				{
					int dest_t = candDP->getPE()->T;
					if((abs(dest_t - next_t)) != 0)
					{
						res2.push_back(candDP);
					}

				}
			}
	}

	return res2;
}

std::vector<std::pair<CGRAXMLCompile::DataPath *,bool>> CGRAXMLCompile::PathFinderMapper::prevTimeDest(
	std::vector<DataPath *> candDestIn,DFGNode *node,int diff)
{
	std::vector<std::pair<DataPath *,bool>> res3;

	std::vector<DFGNode *> parentlist = node->parents;


	    DFGNode * maxLatP=NULL;
	    int maxL=-1;
	    bool isP;
		for(DFGNode* node1:parentlist)
		{
			if (node1->getOPtype(node) != "P")
				isP=false;
			if(node1->rootDP != NULL)
			{
				int latP = node1->rootDP->getLat();
				if(maxL < latP)
				{
					maxL = latP;
					maxLatP = node1;
				}
			}
		}

		for(DataPath *candDP: candDestIn)
		{
			if((node->parents).empty())
			{
					res3.push_back(std::make_pair(candDP,false));
					continue;
			}
			else if(((node->parents).size()==1))
			{
				for(DFGNode * parent:node->parents)
	            {
					if(parent->rootDP!=NULL)
					{
						if(((parent->rootDP->getPE()->X==candDP->getPE()->X) && (parent->rootDP->getPE()->Y==candDP->getPE()->Y))  or (parent->getOPtype(node) == "P"))
						{
							res3.push_back(std::make_pair(candDP,false));
						}else
						{
							//if((node->ASAP < node->ALAP) && !(maxLatP->ASAP < maxLatP->ALAP))
							//{
									PE * prevPE=this->cgra->getPrevPE(candDP->getPE());
									//int i=(((diff+3)/2)-1);

									for (Module *submod : prevPE->subModules)
									{
										if (FU *fu = dynamic_cast<FU *>(submod))
										{
											for (Module *submodFU : fu->subModules)
											{
												if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
												{
													//if(dp->getMappedNode()==NULL)
														res3.push_back(std::make_pair(dp,true));
												}
											}
										}

									}

						}
					}
	            }
			}
			else
			{
				bool samePEMax=false;
				bool diffPEMax=false;
				for(DFGNode * parent:node->parents)
				{
					if(parent->rootDP!=NULL){
						if(parent->rootDP->getLat()==maxL)
						{
							if((parent->rootDP->getPE()->X==candDP->getPE()->X) && (parent->rootDP->getPE()->Y==candDP->getPE()->Y))
								samePEMax=true;
							else
								diffPEMax=true;
						}
						}

				}
					if((samePEMax && !diffPEMax)  or isP)
					{
						res3.push_back(std::make_pair(candDP,false));
						continue;
					}else
					{

								PE * prevPE=this->cgra->getPrevPE(candDP->getPE());

								for (Module *submod : prevPE->subModules)
								{
									if (FU *fu = dynamic_cast<FU *>(submod))
									{
										for (Module *submodFU : fu->subModules)
										{
											if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
											{
											//	if(dp->getMappedNode()==NULL)
													res3.push_back(std::make_pair(dp,true));
											}
										}
									}

								}

					}
				}
		}

 return res3;
}


std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::modifyUsedDest(
	std::vector<DataPath *> candDestIn, DFGNode *node)
{
	std::vector<DataPath *> res4;
	std::vector<DFGNode *> parentNodes = node->parents;

	int maxL=-1;
	for(DFGNode* node1:parentNodes)
	{
		if(node1->rootDP != NULL)
		{
			int latP = node1->rootDP->getLat();
			if(maxL < latP)
			{
				maxL = latP;
			}
		}
	}
	for(DataPath *candDP: candDestIn)
	{
		if((node->parents).empty())
		{
			auto it = std::find(usedDPs.begin(),usedDPs.end(),candDP);
			if(it==usedDPs.end())
			{
				res4.push_back(candDP);
			    continue;
			}

			}
			else if(((node->parents).size()==1))
			{
				for(DFGNode * parent:node->parents)
                {
					if(parent->rootDP!=NULL){
						if((parent->rootDP->getPE()->X==candDP->getPE()->X) && (parent->rootDP->getPE()->Y==candDP->getPE()->Y))
						{
							auto it = std::find(usedDPs.begin(),usedDPs.end(),candDP);
							if(it==usedDPs.end())
							{
								res4.push_back(candDP);
								//break;
							}
							}else
							{
								DataPath * nextDP = findNextDP(candDP);
								auto it = std::find(usedDPs.begin(),usedDPs.end(),nextDP);
								if(it==usedDPs.end())
								{
									res4.push_back(candDP);
									//break;
								}
								}
								}
							}
			}
			else
			{
				bool samePEMax=false;
				bool diffPEMax=false;
				for(DFGNode * parent:node->parents)
				{
					if(parent->rootDP!=NULL){
					if(parent->rootDP->getLat()==maxL)
					{
						if((parent->rootDP->getPE()->X==candDP->getPE()->X) && (parent->rootDP->getPE()->Y==candDP->getPE()->Y))
							samePEMax=true;
						else
							diffPEMax=true;
					}
					}
				}
				if(samePEMax && !diffPEMax)
				{
					auto it = std::find(usedDPs.begin(),usedDPs.end(),candDP);
					if(it==usedDPs.end())
					{
						res4.push_back(candDP);
						continue;
					}
				}else
				{
					DataPath * nextDP = findNextDP(candDP);
					auto it = std::find(usedDPs.begin(),usedDPs.end(),nextDP);
					if(it==usedDPs.end())
					{
						res4.push_back(candDP);
						continue;
					}
				}
			}
	}
						return res4;

}

CGRAXMLCompile::DataPath * CGRAXMLCompile::PathFinderMapper::findNextDP(DataPath * currDP)
{
	DataPath * nextDP;
	PE * currPE=currDP->getPE();
	PE * nextPE=this->cgra->getNextPE(currPE);
	for (Module *submod : nextPE->subModules)
	        {
	        	if (FU *fu = dynamic_cast<FU *>(submod))
	        	{
	        		for (Module *submodFU : fu->subModules)
	        		{
	        			if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
	        			{

	        				//assuming only single DP
	        				nextDP=dp;
	        			}
	        	}
	        }

	        }
	return nextDP;
}


std::vector<CGRAXMLCompile::LatPort> CGRAXMLCompile::PathFinderMapper::getNextPath(std::vector<LatPort> path)
{
	std::vector<LatPort> res;

	for(LatPort port:path)
	{

			PE* currPE = port.second->getPE();
			Port* nextP;
			PE* nextPE = this->cgra->getNextPE(currPE);
			Module* currMod = port.second->getMod();
			Module* nextMod = currPE->getNextTimeIns();

			if(port.second->getType()== 0) {//IN

				if((port.second->getName().compare("DP0_P")==0) or (port.second->getName().compare("P")==0) or (port.second->getName().compare("DP0_I1")==0) or (port.second->getName().compare("I1")==0) or (port.second->getName().compare("DP0_I2")==0) or (port.second->getName().compare("I2")==0))
					continue;
				else
					nextP = nextPE->getInPort(port.second->getName());
			}
			else if(port.second->getType()== 1) {//OUT

				if((port.second->getName().compare("T")==0) or (port.second->getName().compare("DP0_T")==0))
					continue;
				else

					nextP = nextMod->getOutPort(port.second->getName());
			}
			else if(port.second->getType()== 2){ //INT
				nextP = nextPE->getInternalPort(port.second->getName());
			}
			else if(port.second->getType()== 3) {//REGI
				if((port.second->getName().compare("TREG_RI")==0))
					continue;
				else
					nextP = nextPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 4){ //REGO
				if((port.second->getName().compare("TREG_RO")==0))
					continue;
				else
					nextP = nextPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 5){//SOCKET
				nextP = nextPE->getSocketPort(port.second->getName());
			}
			LatPort nextLatP;
			nextLatP.first = port.first+1;
			nextLatP.second = nextP;
			res.push_back(nextLatP);
	}

	return res;
}

CGRAXMLCompile::LatPort CGRAXMLCompile::PathFinderMapper::getNextLatPort(LatPort port)
{
	LatPort res;

			PE* currPE = port.second->getPE();
			Port* nextP;
			PE* nextPE = this->cgra->getNextPE(currPE);
			Module* currMod = port.second->getMod();
			Module* nextMod = currPE->getNextTimeIns();
			if(port.second->getType()== 0) {//IN
				if((port.second->getName().compare("DP0_P")==0)  or (port.second->getName().compare("DP0_I1")==0) or (port.second->getName().compare("DP0_I2")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					nextP = fu->getInPort(port.second->getName());
				}

				else if((port.second->getName().compare("P")==0) or (port.second->getName().compare("I1")==0) or (port.second->getName().compare("I2")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					nextP = dp->getInPort(port.second->getName());
				}
				else
					nextP = nextPE->getInPort(port.second->getName());
			}
			else if(port.second->getType()== 1) {//OUT

				if((port.second->getName().compare("DP0_T")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					nextP = fu->getOutPort(port.second->getName());
				}
				else if((port.second->getName().compare("T")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					nextP = dp->getOutPort(port.second->getName());
				}
				else
					nextP = nextMod->getOutPort(port.second->getName());
			}
			else if(port.second->getType()== 2){ //INT
				nextP = nextPE->getInternalPort(port.second->getName());
			}
			else if(port.second->getType()== 3) {//REGI

					nextP = nextPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 4){ //REGO

					nextP = nextPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 5){//SOCKET
				nextP = nextPE->getSocketPort(port.second->getName());
			}
			LatPort nextLatP;
			nextLatP.first = port.first+1;
			nextLatP.second = nextP;

			res = nextLatP;

	return res;
}

std::vector<CGRAXMLCompile::LatPort> CGRAXMLCompile::PathFinderMapper::getPrevPath(std::vector<LatPort> path)
{
	std::vector<LatPort> res;

	for(LatPort port:path)
	{

			PE* currPE = port.second->getPE();
			Port* nextP;
			PE* prevPE = this->cgra->getPrevPE(currPE);
			Module* currMod = port.second->getMod();
			Module* prevMod = (Module *)prevPE;

			if(port.second->getType()== 0) {//IN
				if((port.second->getName().compare("DP0_P")==0) or (port.second->getName().compare("P")==0) or (port.second->getName().compare("DP0_I1")==0) or (port.second->getName().compare("I1")==0) or (port.second->getName().compare("DP0_I2")==0) or (port.second->getName().compare("I2")==0))
					continue;
				else
					nextP = prevPE->getInPort(port.second->getName());
			}
			else if(port.second->getType()== 1) {//OUT
				if((port.second->getName().compare("T")==0) or (port.second->getName().compare("DP0_T")==0))
					continue;
				else{

					std::cout << port.second->getFullName() << " " << prevMod->getFullName()<< "\n";
					nextP = prevMod->getOutPort(port.second->getName());
				}
			}
			else if(port.second->getType()== 2){ //INT
				nextP = prevPE->getInternalPort(port.second->getName());
			}
			else if(port.second->getType()== 3) {//REGI
				if((port.second->getName().compare("TREG_RI")==0))
					continue;
				else
					nextP = prevPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 4){ //REGO
				if((port.second->getName().compare("TREG_RO")==0))
					continue;
				else
					nextP = prevPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 5){//SOCKET
				nextP = prevPE->getSocketPort(port.second->getName());
			}
			LatPort nextLatP;
			nextLatP.first = port.first-1;
			nextLatP.second = nextP;
			res.push_back(nextLatP);
	}

	return res;
}

CGRAXMLCompile::LatPort CGRAXMLCompile::PathFinderMapper::getPrevLatPort(LatPort port)
{
	LatPort res;

			PE* currPE = port.second->getPE();
			Port* nextP;
			PE* nextPE = this->cgra->getPrevPE(currPE);
			Module* currMod = port.second->getMod();
			Module* nextMod = (Module *)nextPE;
			if(port.second->getType()== 0) {//IN

				if((port.second->getName().compare("DP0_P")==0)  or (port.second->getName().compare("DP0_I1")==0) or (port.second->getName().compare("DP0_I2")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					nextP = fu->getInPort(port.second->getName());
				}

				else if((port.second->getName().compare("P")==0) or (port.second->getName().compare("I1")==0) or (port.second->getName().compare("I2")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					nextP = dp->getInPort(port.second->getName());
				}
				else
					nextP = nextPE->getInPort(port.second->getName());
			}
			else if(port.second->getType()== 1) {//OUT

				if((port.second->getName().compare("DP0_T")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					nextP = fu->getOutPort(port.second->getName());
				}
				else if((port.second->getName().compare("T")==0))
				{
					FU* fu = static_cast<FU*>(nextPE->getSubMod("FU0")); assert(fu);
					DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);
					nextP = dp->getOutPort(port.second->getName());
				}else
					nextP = nextMod->getOutPort(port.second->getName());
			}
			else if(port.second->getType()== 2){ //INT
				nextP = nextPE->getInternalPort(port.second->getName());
			}
			else if(port.second->getType()== 3) {//REGI

					nextP = nextPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 4){ //REGO

					nextP = nextPE->getSingleRegPort(port.second->getName());
			}
			else if(port.second->getType()== 5){//SOCKET
				nextP = nextPE->getSocketPort(port.second->getName());
			}
			LatPort nextLatP;
			nextLatP.first = port.first+1;
			nextLatP.second = nextP;

			res = nextLatP;

	return res;
}




std::vector<std::pair<CGRAXMLCompile::DataPath *,std::pair<std::vector<CGRAXMLCompile::DFGNode *>,std::vector<CGRAXMLCompile::DFGNode *>>>> CGRAXMLCompile::PathFinderMapper::modifyLatencyDestUnordered(
	std::vector<DataPath *> candDestIn, DFGNode *node)
{
	std::vector<std::pair<DataPath *,std::pair<std::vector<DFGNode *>,std::vector<DFGNode *>>>> res2;

	std::vector<DataPath *> res2P;
	std::vector<DataPath *> res2C;

	std::vector<DFGNode *> parentlist = node->parents;
	std::vector<DFGNode *> childlist = node->children;

	bool isChildMapped=false;
	bool isParentMapped=false;

    std::vector<std::pair <DFGNode *,bool>> mappedP;
    std::vector<std::pair <DFGNode *,bool>> mappedC;


    int maxL=-1;
    int minL=10000;
    bool isP_P=false;
    bool isP_C=false;

	for(DFGNode* node1:parentlist)
	{
		isP_P=false;
		if(node1->rootDP != NULL)
		{
			if (node1->getOPtype(node) == "P")
				isP_P=true;
			isParentMapped =true;
			mappedP.push_back(std::make_pair(node1,isP_P));
		}
	}

	for(DFGNode* node1:childlist)
	{
		isP_C=false;
		if(node1->rootDP != NULL)
		{
			if (node->getOPtype(node1) == "P")
				isP_C=true;
			isChildMapped =true;
			mappedC.push_back(std::make_pair(node1,isP_C));
		}
	}

	for(DataPath *candDP: candDestIn)
	{
		std::vector<DFGNode *> multiCycleNodesP;
		std::vector<DFGNode *> multiCycleNodesC;
		if(!isChildMapped && !isParentMapped)
		{

			res2.push_back(std::make_pair(candDP,std::make_pair(multiCycleNodesP,multiCycleNodesC)));
		}
		else if(isChildMapped && !isParentMapped)
		{

			bool all_areP=true;
			for(std::pair<DFGNode *, bool> pair : mappedC)
			{
				if (!pair.second)
					all_areP=false;
			}

			if(all_areP)
				res2.push_back(std::make_pair(candDP,std::make_pair(multiCycleNodesP,multiCycleNodesC)));
			else
			{
				DataPath *newDP = checkDP(candDP,mappedC,node,false);
				if(newDP != NULL){
				for(std::pair<DFGNode *, bool> pair : mappedC)
				{
					if((!(pair.first->rootDP->getPE()->X==newDP->getPE()->X) or !(pair.first->rootDP->getPE()->Y==newDP->getPE()->Y)) && !pair.second)
					{
						multiCycleNodesC.push_back(pair.first);
					}
				}
					res2.push_back(std::make_pair(newDP,std::make_pair(multiCycleNodesP,multiCycleNodesC)));
				}
			}
		}
		else if(!isChildMapped && isParentMapped)
		{

			bool all_areP=true;
			for(std::pair<DFGNode *, bool> pair : mappedP)
			{
				if (!pair.second)
					all_areP=false;
			}

			if(all_areP)
			{
				res2.push_back(std::make_pair(candDP,std::make_pair(multiCycleNodesP,multiCycleNodesC)));

			}
			else
			{
				DataPath *newDP = checkDP(candDP,mappedP,node,true);

				if(newDP != NULL)
				{
				for(std::pair<DFGNode *, bool> pair : mappedP)
				{
					if((!(pair.first->rootDP->getPE()->X==newDP->getPE()->X) or !(pair.first->rootDP->getPE()->Y==newDP->getPE()->Y)) && !pair.second)
					{
						multiCycleNodesP.push_back(pair.first);
					}
				}

					res2.push_back(std::make_pair(newDP,std::make_pair(multiCycleNodesP,multiCycleNodesC)));
				}
			}
		}
		else
		{

			bool all_arePParent=true;
			bool all_arePChild=true;
			for(std::pair<DFGNode *, bool> pair : mappedC)
			{
				if (!pair.second)
					all_arePChild=false;
			}
			for(std::pair<DFGNode *, bool> pair : mappedP)
			{
				if (!pair.second)
					all_arePParent=false;
			}

			if(all_arePParent && all_arePChild)
				res2.push_back(std::make_pair(candDP,std::make_pair(multiCycleNodesP,multiCycleNodesC)));
			else if(all_arePParent)
				res2P.push_back(candDP);
			else if(all_arePChild)
				res2C.push_back(candDP);
			else
			{
				DataPath *newDP_P = checkDP(candDP,mappedP,node,true);
				if(newDP_P != NULL)
					res2P.push_back(newDP_P);
				DataPath *newDP_C = checkDP(candDP,mappedC,node,false);
				if(newDP_C != NULL)
					res2C.push_back(newDP_C);
			}
		}
	}


	for(DataPath *commDP: res2P)
	{
		std::vector<DFGNode *> multiCycleNodesP;
		std::vector<DFGNode *> multiCycleNodesC;
		auto it = std::find(res2C.begin(),res2C.end(),commDP);

		if(it != res2C.end())
		{
			for(std::pair<DFGNode *, bool> pair : mappedP)
			{
				if((!(pair.first->rootDP->getPE()->X==commDP->getPE()->X) or !(pair.first->rootDP->getPE()->Y==commDP->getPE()->Y)) && !pair.second)
				{
					multiCycleNodesP.push_back(pair.first);
				}
			}
			for(std::pair<DFGNode *, bool> pair : mappedC)
			{
				if((!(pair.first->rootDP->getPE()->X==commDP->getPE()->X) or !(pair.first->rootDP->getPE()->Y==commDP->getPE()->Y)) && !pair.second)
				{
					multiCycleNodesC.push_back(pair.first);
				}
			}
			res2.push_back(std::make_pair(commDP,std::make_pair(multiCycleNodesP,multiCycleNodesC)));
		}

	}


	return res2;
}

CGRAXMLCompile::DataPath * CGRAXMLCompile::PathFinderMapper::checkDP(
	DataPath * candDestIn, std::vector<std::pair <DFGNode *,bool>> mappedList, DFGNode *node,bool isParent)
{
	DataPath * res2=NULL;

	DFGNode * latNode=NULL;
	DFGNode * latNodeP=NULL;
	int latencyN;
	int latencyNP;

	if(isParent)
	{
		latencyN = -1;
		latencyNP = -1;
	}
	else
	{
		latencyN = 1000;
		latencyNP = 1000;
	}

	int T1;
	int T2;
	for(std::pair<DFGNode *, bool> pair : mappedList)
	{
		if(pair.first->rootDP != NULL)
		{
			if(!pair.second)
			{
				int latP = pair.first->rootDP->getLat();
				if(((latencyN < latP) && isParent) or ((latencyN > latP) && !isParent))
				{
					latencyN = latP;
					latNode = pair.first;

				}
			}else
			{
				int latP = pair.first->rootDP->getLat();
				if(((latencyN < latP) && isParent) or ((latencyN > latP) && !isParent))
				{
					latencyNP = latP;
					latNodeP = pair.first;

				}
			}
		}
	}
	int T_P = latNode->rootDP->getPE()->T;
	int T_P_P = latNode->rootDP->getPE()->T;
	int next_t = (T_P + latNode->rootDP->getFU()->supportedOPs[latNode->op]) % this->cgra->get_t_max();
	int next_tp = (T_P_P + latNode->rootDP->getFU()->supportedOPs[latNode->op]) % this->cgra->get_t_max();
	int prev_t = (T_P - candDestIn->getFU()->supportedOPs[node->op]) % this->cgra->get_t_max();
	int prev_tp = (T_P_P - candDestIn->getFU()->supportedOPs[node->op]) % this->cgra->get_t_max();
	int dest_t = candDestIn->getPE()->T;

	int dest_toT,dest_toTP,tt,ttp;
	if(isParent)
	{
		tt = next_t;
		ttp = next_tp;

		if(tt > dest_t)
			dest_toT = (dest_t + this->cgra->get_t_max() - tt)%this->cgra->get_t_max();
		else
			dest_toT = (dest_t - tt)%this->cgra->get_t_max();

		if(ttp > dest_t)
			dest_toTP = (dest_t + this->cgra->get_t_max() - ttp)%this->cgra->get_t_max();
		else
			dest_toTP = (dest_t - ttp)%this->cgra->get_t_max();

	}else
	{
		tt = prev_t;
		ttp = prev_tp;

		if(tt < dest_t)
			dest_toT = (tt + this->cgra->get_t_max() - dest_t)%this->cgra->get_t_max();
		else
			dest_toT = (tt - dest_t )%this->cgra->get_t_max();

		if(ttp < dest_t)
			dest_toTP = (ttp + this->cgra->get_t_max() - dest_t)%this->cgra->get_t_max();
		else
			dest_toTP = (ttp - dest_t)%this->cgra->get_t_max();
	}

	if((((latencyNP > latencyN) && isParent) or ((latencyNP < latencyN) && !isParent)) && (dest_toTP < dest_toT) )
	{
		res2 = candDestIn;
	}
	else
	{
		if(((latNode->rootDP->getPE()->X==candDestIn->getPE()->X) && (latNode->rootDP->getPE()->Y==candDestIn->getPE()->Y)))
		{
			res2 = candDestIn;
		}else
		{
			if((((abs(dest_t - next_t)) != 0) && isParent))
			{

				res2 = candDestIn;

			}

			if((((abs(dest_t - prev_t)) != 0) && !isParent))
			{
				res2 = candDestIn;

			}
		}
	}

		return res2;

}
void CGRAXMLCompile::PathFinderMapper::sortBackEdgePriorityASAPALP()
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
	};

	std::set<BEDist> backedges;

	for (DFGNode &node : dfg->nodeList)
	{

		if (node.idx == 97)
		{
			std::cout << "node_idx:97,node_ASAP:" << node.ASAP << "\n";
		}
		for (DFGNode *child : node.children)
		{

			if (node.idx == 97)
			{
				std::cout << "child_idx:" << child->idx << "child_ASAP:" << child->ASAP << "\n";
			}

			if (child->ASAP <= node.ASAP)
			{

				backedges.insert(BEDist(&node, child, node.ASAP - child->ASAP));
			}
		}
	}

	//populate reccycles
	std::cout << "Populate Rec Cycles!\n";
	RecCycles.clear();

	for (BEDist be : backedges)
	{
		std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be.parent);

		//std::cout << "REC_CYCLE :: BE_Parent = " << be.parent->idx << "\n";
		//std::cout << "REC_CYCLE :: BE_Child = " << be.child->idx << "\n";
		//std::cout << "REC_CYCLE :: BE_Parent's ancesotry : \n";
		for (DFGNode *n : backedgePathVec)
		{
			if (RecCycles[BackEdge(be.parent, be.child)].find(n) == RecCycles[BackEdge(be.parent, be.child)].end())
			{
				//std::cout << n->idx << ",";
			}
			RecCycles[BackEdge(be.parent, be.child)].insert(n);
		}
		//std::cout << "REC_CYCLE :: Done!\n";

	}

	RecCyclesLS.clear();
	for (DFGNode &node : dfg->nodeList)
	{
		for (DFGNode *recParent : node.recParents)
		{
			BEDist be_temp(&node, recParent, node.ASAP - recParent->ASAP);

			std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be_temp.parent);

			std::cout << "REC_CYCLELS :: BE_Parent = " << be_temp.parent->idx << "\n";
			std::cout << "REC_CYCLELS :: BE_Child = " << be_temp.child->idx << "\n";
			std::cout << "REC_CYCLELS :: BE_Parent's ancesotry : \n";
			for (DFGNode *n : backedgePathVec)
			{
				if (n == be_temp.parent)
					continue;
				if (RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].find(n) == RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].end())
				{
					std::cout << n->idx << ",";
				}
				RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].insert(n);
			}
			std::cout << "REC_CYCLELS :: Done!\n";

			backedges.insert(be_temp);
		}
	}

	std::map<DFGNode *, std::vector<DFGNode *>> beparentAncestors;
	std::map<DFGNode *, std::vector<DFGNode *>> bechildAncestors;
	std::map<std::pair<DFGNode *, DFGNode *>, bool> trueBackedges;

	for (BEDist be : backedges)
	{
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		std::cout << "BE CHILD = " << be.child->idx << "\n";

		std::cout << "Ancestory : "
				  << "\n";
		beparentAncestors[be.parent] = dfg->getAncestoryASAP(be.parent);
		bechildAncestors[be.child] = dfg->getAncestoryASAP(be.child);
		std::cout << "\n";

		if (std::find(beparentAncestors[be.parent].begin(),
					  beparentAncestors[be.parent].end(),
					  be.child) == beparentAncestors[be.parent].end())
		{
			std::cout << "BE CHILD does not belong BE Parent's Ancestory\n";

			//Hack to force all backedges to be true backedges
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
		}
		else
		{
			//change this be.parent if PHI nodes are not removed
			std::cout << "RecPHI inserted : " << be.child->idx << "\n";
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
			//			RecPHIs.insert(be.child);
		}
	}


	std::map<DFGNode *, std::set<DFGNode *>> superiorChildren;

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
			//			if(trueBackedges[std::make_pair(be.parent,be.child)] == false) continue;
			if (std::find(mergedAncestories[key].begin(), mergedAncestories[key].end(), be.child) != mergedAncestories[key].end())
			{

				if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
				{
					superiorChildren[key].insert(be.child);
				}

				std::cout << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedAncestories[key] = dfg->mergeAncestoryASAP(mergedAncestories[key], beparentAncestors[be.parent], RecCycles);
				merged = true;
				std::cout << "Merging Done :: " << key->idx << ", " << be.parent->idx << "\n";
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

	for (BEDist be : backedges)
	{
		std::vector<DFGNode *> mergedSuperiorChildren;
		for (DFGNode *sChild : superiorChildren[mergedKeys[be.parent]])
		{
			mergedSuperiorChildren = dfg->mergeAncestoryASAP(mergedSuperiorChildren, bechildAncestors[sChild], RecCycles);
		}

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

		ancestoryNodes = dfg->getAncestoryASAP(be.child);
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

	std::map<double, std::vector<DFGNode *>> asapalapLevelNodeList;
	for (DFGNode &node : dfg->nodeList)
	{
		if(node.parents.empty())
			asapalapLevelNodeList[node.ALAP].push_back(&node);
		else if(node.children.empty())
			asapalapLevelNodeList[node.ASAP].push_back(&node);
		else
			asapalapLevelNodeList[(node.ASAP+node.ALAP)/2].push_back(&node);
	}

	int maxASAPlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : asapalapLevelNodeList)
	{
		if (pair.first > maxASAPlevel)
		{
			maxASAPlevel = pair.first;
		}
	}
	DFGNode * blacklist;
	for (int i = 0; i <= maxASAPlevel; ++i)
	{
		for (DFGNode *node : asapalapLevelNodeList[i])
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), node) == sortedNodeList.end())
			{

				//if(node->idx != 20037)
					sortedNodeList.push_back(node);
				//else
				//	blacklist = node;
			}
		}
	}


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

void CGRAXMLCompile::PathFinderMapper::printHyCUBEConfig(const std::vector<std::vector<std::vector<InsFormat> > >& insFArr,
		std::string fName, CGRA* cgra) {
	std::vector<std::vector<std::vector<InsFormatToggle>>> InsFArrToggle (cgra->get_t_max()+1,
			std::vector<std::vector<InsFormatToggle>>(cgra->get_y_max(),
					std::vector<InsFormatToggle>(cgra->get_x_max())));

	std::vector<std::vector<std::vector<InsFormatRouterHot>>> InsFArrRouter (cgra->get_t_max()+1,
				std::vector<std::vector<InsFormatRouterHot>>(cgra->get_y_max(),
						std::vector<InsFormatRouterHot>(cgra->get_x_max())));

	std::vector<std::vector<std::vector<InsFormatEnable>>> InsFArrEnable (cgra->get_t_max()+1,
					std::vector<std::vector<InsFormatEnable>>(cgra->get_y_max(),
							std::vector<InsFormatEnable>(cgra->get_x_max())));

	std::vector<std::vector<std::vector<string>>> InsFArrConst (cgra->get_t_max()+1,
						std::vector<std::vector<string>>(cgra->get_y_max(),
								std::vector<string>(cgra->get_x_max())));

	std::vector<std::vector<std::vector<string>>> InsFArrOpcode (cgra->get_t_max()+1,
						std::vector<std::vector<string>>(cgra->get_y_max(),
								std::vector<string>(cgra->get_x_max())));

	std::ofstream binFile(fName.c_str());
	int t_max = cgra->get_t_max() + 1;
	int y_max = cgra->get_y_max();
	int x_max = cgra->get_x_max();
	binFile << " -----------------------------------------------------------------------------\n";

	std::vector<int> activeClass(t_max);

	for(int y = 0 ; y < y_max ; y++){
		for(int x = 0 ; x < x_max ; x++){

			binFile << "Y=" << y << " X=" << x << "\n";



			binFile << "\nCONST BUFFER\n";
			if(insFArr[0][y][x].constant_valid == "1")
				binFile << insFArr[0][y][x].constant << "\n";
			for(int t = 1 ; t < t_max ; t++){
				if((insFArr[t-1][y][x].constant != insFArr[t][y][x].constant) && (insFArr[t][y][x].constant_valid == "1"))
					binFile << insFArr[t][y][x].constant << "\n";
			}
			binFile << "\nOPCODE\n";
			if(insFArr[0][y][x].opcode != "00000")
				binFile << insFArr[0][y][x].opcode << "\n";
			for(int t = 1 ; t < t_max ; t++){
				if((insFArr[t-1][y][x].opcode != insFArr[t][y][x].opcode) && (insFArr[t][y][x].opcode != "00000"))
					binFile << insFArr[t][y][x].opcode << "\n";
			}
			binFile << " -----------------------------------------------------------------------------\n";
		}
	}

}

std::vector<std::vector<CGRAXMLCompile::Port*>> CGRAXMLCompile::PathFinderMapper::getClasses(PE* pe ,int num)
{
	std::vector<std::vector<Port *>> classes;
	std::vector<Port *> class1;
	std::vector<Port *> class2;
	std::vector<Port *> class3;
	std::vector<Port *> class4;

	int y_max = this->cgra->get_y_max();
	int x_max = this->cgra->get_x_max();


	FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
	DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

	Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
	Port* easto = pe->getOutPort("EAST_O"); assert(easto);
	Port* westo = pe->getOutPort("WEST_O"); assert(westo);
	Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

	Port* i1_lsb_ip = fu->getInPort("DP0_I1_LSB"); assert(i1_lsb_ip);
	Port* i1_msb_ip = fu->getInPort("DP0_I1_MSB"); assert(i1_msb_ip);
	Port* i2_lsb_ip = fu->getInPort("DP0_I2_LSB"); assert(i2_lsb_ip);
	Port* i2_msb_ip = fu->getInPort("DP0_I2_MSB"); assert(i2_msb_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);


#ifndef TORUS
	if((pe->X == 0) && (pe->Y == 0))
	{
		class1.push_back(p_ip);
		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(easto);
		class2.push_back(southo);

		if(num ==4)
		{
			class3.push_back(easto);
			class3.push_back(southo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
		}

	} else if((pe->X == 0) && (pe->Y == y_max-1))
	{
		class1.push_back(p_ip);
		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(easto);
		class2.push_back(northo);

		if(num ==4)
		{
			class3.push_back(easto);
			class3.push_back(northo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
		}


	}else if((pe->X == x_max-1) && (pe->Y == 0))
	{
		class1.push_back(p_ip);
		class1.push_back(westo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(westo);
		class2.push_back(southo);

		if(num ==4)
		{
			class3.push_back(westo);
			class3.push_back(southo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
		}


	}else if((pe->X == x_max-1) && (pe->Y == y_max-1))
	{
		class1.push_back(p_ip);
		class1.push_back(westo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(westo);
		class2.push_back(northo);

		if(num ==4)
		{
			class3.push_back(westo);
			class3.push_back(northo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
		}

	}else if(pe->X == 0)
	{
		class1.push_back(p_ip);
		class1.push_back(easto);
		class1.push_back(southo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(easto);
		class2.push_back(southo);
		class2.push_back(northo);

		if(num ==4)
		{
			class3.push_back(easto);
			class3.push_back(southo);
			class3.push_back(northo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
			class4.push_back(easto);

		}
	}else if(pe->X == x_max-1)
	{
		class1.push_back(p_ip);
		class1.push_back(westo);
		class1.push_back(southo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(westo);
		class2.push_back(southo);
		class2.push_back(northo);

		if(num ==4)
		{
			class3.push_back(westo);
			class3.push_back(southo);
			class3.push_back(northo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
			class4.push_back(westo);

		}


	}else if(pe->Y == 0)
	{
		class1.push_back(p_ip);
		class1.push_back(easto);
		class1.push_back(southo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(westo);
		class2.push_back(southo);
		class2.push_back(easto);

		if(num ==4)
		{
			class3.push_back(westo);
			class3.push_back(southo);
			class3.push_back(easto);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
			class4.push_back(southo);

		}
	}else if(pe->Y == y_max-1)
	{
		class1.push_back(p_ip);
		class1.push_back(easto);
		class1.push_back(northo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);

		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);
		class2.push_back(westo);
		class2.push_back(northo);
		class2.push_back(easto);

		if(num ==4)
		{
			class3.push_back(westo);
			class3.push_back(northo);
			class3.push_back(easto);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);

			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);
			class4.push_back(northo);

		}

	}else
	{
#endif

		Port * dir1;
		Port * dir2;
		Port * dir3;
		Port * dir4;

		if(pe->X < x_max/2)
		{
			dir1 = easto;
			dir3 = westo;
		}
		else{
			dir1 = westo;
			dir3 = easto;
		}

		if(pe->Y < y_max/2)
		{
			dir2 = southo;
			dir4 = northo;
		}
		else{
			dir2 = northo;
			dir4 = southo;
		}

		if(pe->is_hot)
		{
			class1.push_back(p_ip);
			class1.push_back(dir1);
			class1.push_back(dir2);
			class1.push_back(dir3);
			class1.push_back(i1_lsb_ip);
			class1.push_back(i1_msb_ip);

			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);
			class2.push_back(dir1);
			class2.push_back(dir2);
			class2.push_back(dir3);
			class2.push_back(dir4);
			if(num ==4)
			{
				class3.push_back(dir1);
				class3.push_back(dir2);
				class3.push_back(dir3);
				class3.push_back(dir4);
				class3.push_back(i1_lsb_ip);
				class3.push_back(i1_msb_ip);

				class4.push_back(i1_lsb_ip);
				class4.push_back(i1_msb_ip);
				class4.push_back(i2_lsb_ip);
				class4.push_back(i2_msb_ip);
				class4.push_back(dir1);
				class4.push_back(p_ip);

			}
		}else{
			if(num==2){
				class1.push_back(p_ip);
				class1.push_back(dir1);
				class1.push_back(dir2);
				class1.push_back(dir3);
				class1.push_back(i1_lsb_ip);
				class1.push_back(i1_msb_ip);

				class2.push_back(i2_lsb_ip);
				class2.push_back(i2_msb_ip);
				class2.push_back(dir1);
				class2.push_back(dir2);
				class2.push_back(dir3);
				class2.push_back(dir4);
			}else if(num ==4)
			{

				class1.push_back(p_ip);
				class1.push_back(dir1);
				class1.push_back(dir2);
				class1.push_back(i1_lsb_ip);
				class1.push_back(i1_msb_ip);

				class2.push_back(i2_lsb_ip);
				class2.push_back(i2_msb_ip);
				class2.push_back(dir1);
				class2.push_back(dir2);
				class2.push_back(p_ip);

				class3.push_back(dir2);
				class3.push_back(dir3);
				class3.push_back(dir4);
				class3.push_back(i1_lsb_ip);
				class3.push_back(i1_msb_ip);

				class4.push_back(i1_lsb_ip);
				class4.push_back(i1_msb_ip);
				class4.push_back(i2_lsb_ip);
				class4.push_back(i2_msb_ip);
				class4.push_back(dir1);

			}
		}
#ifndef TORUS
	}
#endif

	classes.push_back(class1);
	classes.push_back(class2);
if(num==4){
	classes.push_back(class3);
	classes.push_back(class4);
}

	return classes;
}


int CGRAXMLCompile::PathFinderMapper::getCrossbarSimilarityCostDualclass4(LatPort src,
													LatPort next_to_src,bool multiCycle)
{


	int cost=ROUTER_COST;

	int x= next_to_src.second->getPE()->X;
	int y= next_to_src.second->getPE()->Y;


	vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);



	PE * pe = next_to_src.second->getPE();

	PE * prevPE = this->cgra->getPrevPE(pe);
	PE * prevPrevPE = this->cgra->getPrevPE(prevPE);
	PE * nextPE = this->cgra->getNextPE(pe);
	std::string currPString = next_to_src.second->getName();
	FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
	DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

	FU* prevfu = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevfu);
	DataPath* prevdp = static_cast<DataPath*>(prevfu->getSubMod("DP0")); assert(prevdp);

	Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
	Port* easto = pe->getOutPort("EAST_O"); assert(easto);
	Port* westo = pe->getOutPort("WEST_O"); assert(westo);
	Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);

	Port* i1_lsb_ip = fu->getInPort("DP0_I1_LSB"); assert(i1_lsb_ip);
	Port* i1_msb_ip = fu->getInPort("DP0_I1_MSB"); assert(i1_msb_ip);
	Port* i2_lsb_ip = fu->getInPort("DP0_I2_LSB"); assert(i2_lsb_ip);
	Port* i2_msb_ip = fu->getInPort("DP0_I2_MSB"); assert(i2_msb_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);

	std::map<std::string, std::string> currentPEmap = getConnectedPorts4(pe);
	std::map<std::string, std::string> prevPEmap = getConnectedPorts4(prevPE);
	std::map<std::string, std::string> prevPrevPEmap = getConnectedPorts4(prevPrevPE);
	std::map<std::string, std::string> nextPEmap = getConnectedPorts4(nextPE);


	PE * nextNextPE = this->cgra->getNextPE(nextPE);
	std::map<std::string, std::string> nextNextPEmap = getConnectedPorts4(nextNextPE);

	int x_max = 4;
	int y_max = 4;

	std::vector<std::vector<Port *>> classes;
	std::vector<Port *> class1;
	std::vector<Port *> class2;
	std::vector<Port *> class3;
	std::vector<Port *> class4;


	std::vector<Port *> classAP;
	std::vector<Port *> classBP;


#ifndef TORUS
	if((pe->X == 0) && (pe->Y == 0))
	{
		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(southo);

		class2.push_back(easto);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(easto);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(southo);

		class4.push_back(southo);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);


	} else if((pe->X == 0) && (pe->Y == y_max-1))
	{
		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(northo);


		class2.push_back(easto);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(easto);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(northo);

		class4.push_back(northo);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);


	}else if((pe->X == x_max-1) && (pe->Y == 0))
	{
		class1.push_back(westo);
		class1.push_back(southo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);


		class2.push_back(westo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(westo);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(southo);

		class4.push_back(southo);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);



	}else if((pe->X == x_max-1) && (pe->Y == y_max-1))
	{
		class1.push_back(westo);
		class1.push_back(northo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);

		class2.push_back(westo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(westo);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(northo);

		class4.push_back(northo);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);


	}else if(pe->X == 0)
	{


		class1.push_back(northo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(easto);
		class1.push_back(southo);

		class2.push_back(easto);
		class2.push_back(southo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(northo);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(easto);
		class3.push_back(southo);

		class4.push_back(southo);
		class4.push_back(northo);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);



	}else if(pe->X == x_max-1)
	{

		class1.push_back(northo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(westo);
		class1.push_back(southo);


		class2.push_back(westo);
		class2.push_back(southo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(northo);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(westo);
		class3.push_back(southo);

		class4.push_back(southo);
		class4.push_back(northo);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);


	}else if(pe->Y == 0)
	{

		class1.push_back(westo);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(southo);
		class1.push_back(easto);

		class2.push_back(southo);
		class2.push_back(easto);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(westo);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(southo);
		class3.push_back(easto);

		class4.push_back(easto);
		class4.push_back(westo);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);


	}else if(pe->Y == y_max-1)
	{
		class1.push_back(easto);
		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);
		class1.push_back(northo);
		class1.push_back(westo);

		class2.push_back(northo);
		class2.push_back(westo);
		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);
		class2.push_back(i2_lsb_ip);
		class2.push_back(i2_msb_ip);

		class3.push_back(westo);
		class3.push_back(i2_lsb_ip);
		class3.push_back(i2_msb_ip);
		class3.push_back(p_ip);
		class3.push_back(northo);
		class3.push_back(easto);

		class4.push_back(westo);
		class4.push_back(easto);
		class4.push_back(i1_lsb_ip);
		class4.push_back(i1_msb_ip);
		class4.push_back(i2_lsb_ip);
		class4.push_back(i2_msb_ip);

	}else
	{
#endif

		class1.push_back(i1_lsb_ip);
		class1.push_back(i1_msb_ip);
		class1.push_back(p_ip);


		class2.push_back(i1_lsb_ip);
		class2.push_back(i1_msb_ip);

		if(((pe->X == 2) && (pe->Y == 1))){
			class1.push_back(easto);
			class1.push_back(northo);
			class1.push_back(southo);

			class2.push_back(westo);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);

			class3.push_back(westo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);
			class3.push_back(southo);
			class3.push_back(northo);
			class3.push_back(easto);

			class4.push_back(northo);
			class4.push_back(easto);
			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);


		}else if(((pe->X == 1) && (pe->Y == 2))){
			class1.push_back(easto);
			class1.push_back(northo);
			class1.push_back(southo);

			class2.push_back(westo);
			class2.push_back(northo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);

			class3.push_back(westo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);
			class3.push_back(southo);
			class3.push_back(northo);
			class3.push_back(easto);

			class4.push_back(southo);
			class4.push_back(easto);
			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);

		}else if(((pe->X == 1) && (pe->Y == 1))){
			class1.push_back(easto);
			class1.push_back(westo);
			class1.push_back(northo);

			class2.push_back(easto);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);

			class3.push_back(westo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);
			class3.push_back(southo);
			class3.push_back(northo);
			class3.push_back(easto);

			class4.push_back(northo);
			class4.push_back(westo);
			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);

		}else if(((pe->X == 2) && (pe->Y == 2))){
			class1.push_back(easto);
			class1.push_back(westo);
			class1.push_back(northo);

			class2.push_back(westo);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);

			class3.push_back(westo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);
			class3.push_back(southo);
			class3.push_back(northo);
			class3.push_back(easto);

			class4.push_back(northo);
			class4.push_back(easto);
			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);

		}else{
			class1.push_back(easto);
			class1.push_back(westo);
			class1.push_back(northo);

			class2.push_back(westo);
			class2.push_back(southo);
			class2.push_back(i2_lsb_ip);
			class2.push_back(i2_msb_ip);

			class3.push_back(westo);
			class3.push_back(i1_lsb_ip);
			class3.push_back(i1_msb_ip);
			class3.push_back(southo);
			class3.push_back(northo);
			class3.push_back(easto);

			class4.push_back(northo);
			class4.push_back(easto);
			class4.push_back(i1_lsb_ip);
			class4.push_back(i1_msb_ip);
			class4.push_back(i2_lsb_ip);
			class4.push_back(i2_msb_ip);

		}
#ifndef TORUS
	}
#endif

	classes.push_back(class1);
	classes.push_back(class2);
	classes.push_back(class3);
	classes.push_back(class4);


	int size = classes.size();
	std::vector<bool> active_class_prev(size,false);
	std::vector<bool> active_class_next(size,false);
	std::vector<bool> active_class(size,false);
	std::vector<bool> curr_class(size,false);

	std::vector<std::vector<Port *>> prev_active_list;
	std::vector<std::vector<Port *>> next_active_list;

	std::vector<Port *> commonPorts;
	std::vector <Port* > class_vec0 = classes[0];
	for (Port * element : class_vec0) {
		bool is_common = true;
		for(int i=1;i<size;i++){
			if(std::find(classes[i].begin(),classes[i].end(),element) == classes[i].end())
				is_common = false;
		}
		if(is_common)
			commonPorts.push_back(element);
	}
	std::map<int,bool> allPINV;
	std::map<int,bool> allCINV;
	std::map<int,bool> allNINV;

	for(int i=0;i<size;i++){
		bool prevAllINV = true;
		bool nextAllINV = true;
		bool currAllINV = true;
		std::vector <Port* > class_vec = classes[i];
		for (Port * element : class_vec) {
			std::string portName = element->getName();
			if(currentPEmap[portName] != "INV")
				currAllINV = false;
			if(prevPEmap[portName] != "INV")
				prevAllINV = false;
			if(nextPEmap[portName] != "INV")
				nextAllINV = false;
		}
		allPINV[i]=prevAllINV;
		allNINV[i]=nextAllINV;
		allCINV[i]=currAllINV;
	}


	std::vector<int> active_index;
	for(int i=0;i<size;i++){

		std::vector <Port* > class_vec = classes[i];


		for (Port * element : class_vec) {
			std::string portName = element->getName();
			if((std::find(commonPorts.begin(),commonPorts.end(),element) != commonPorts.end()) or (std::find(commonPorts.begin(),commonPorts.end(),next_to_src.second) != commonPorts.end()))
				continue;
			if((!(currentPEmap[portName]==prevPEmap[portName]) && !(next_to_src.second == element)))
			{

				if(!(!allPINV[i] && allCINV[i]))
					active_class_prev[i] = true;
			}

			if((!(currentPEmap[portName]==nextPEmap[portName]) && !(next_to_src.second == element)))
			{

				if(!(allNINV[i] && !allCINV[i]))
					active_class_next[i] = true;
			}

			if(next_to_src.second == element)
			{
				curr_class[i] = true;
			}


		}
		active_class[i] = active_class_prev[i] or active_class_next[i];

		if(active_class[i])
			active_index.push_back(i);
	}


	if(active_index.size() == 1)
	{
		if(curr_class[active_index.at(0)])
			cost =0;
		else if (std::find(commonPorts.begin(),commonPorts.end(),next_to_src.second) != commonPorts.end()){
			cost =0;
		}else {
			cost = ROUTER_COST;
		}
	}else if(active_index.size() > 1)
	{
		bool belong_to_active=false;
		for(int j:active_index)
		{
			if(curr_class[j])
				belong_to_active=true;
		}

		if(belong_to_active)
			cost =0;
		else {
			cost = ROUTER_COST;
		}
	}else if(active_index.size() == 0){
		cost =0;
	}

	return cost;
}


int CGRAXMLCompile::PathFinderMapper::getCrossbarSimilarityCostDualclass4n(LatPort src,
													LatPort next_to_src,bool multiCycle)
{


	int cost=ROUTER_COST;

	int x= next_to_src.second->getPE()->X;
	int y= next_to_src.second->getPE()->Y;


	vector<PE *> temporalPEList = this->cgra->getTemporalPEList(x,y);



	PE * pe = next_to_src.second->getPE();

	PE * prevPE = this->cgra->getPrevPE(pe);
	PE * prevPrevPE = this->cgra->getPrevPE(prevPE);
	PE * nextPE = this->cgra->getNextPE(pe);
	std::string currPString = next_to_src.second->getName();
	FU* fu = static_cast<FU*>(pe->getSubMod("FU0")); assert(fu);
	DataPath* dp = static_cast<DataPath*>(fu->getSubMod("DP0")); assert(dp);

	FU* prevfu = static_cast<FU*>(prevPE->getSubMod("FU0")); assert(prevfu);
	DataPath* prevdp = static_cast<DataPath*>(prevfu->getSubMod("DP0")); assert(prevdp);

	Port* northo = pe->getOutPort("NORTH_O"); assert(northo);
	Port* easto = pe->getOutPort("EAST_O"); assert(easto);
	Port* westo = pe->getOutPort("WEST_O"); assert(westo);
	Port* southo = pe->getOutPort("SOUTH_O"); assert(southo);
#ifdef DUAL_STREAMING
	Port* i1_lsb_ip = fu->getInPort("DP0_I1_LSB"); assert(i1_lsb_ip);
	Port* i1_msb_ip = fu->getInPort("DP0_I1_MSB"); assert(i1_msb_ip);
	Port* i2_lsb_ip = fu->getInPort("DP0_I2_LSB"); assert(i2_lsb_ip);
	Port* i2_msb_ip = fu->getInPort("DP0_I2_MSB"); assert(i2_msb_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);
#else
	Port* i1_ip = fu->getInPort("DP0_I1"); assert(i1_ip);
	//Port* i1_msb_ip = fu->getInPort("DP0_I1_MSB"); assert(i1_msb_ip);
	Port* i2_ip = fu->getInPort("DP0_I2"); assert(i2_ip);
	//Port* i2_msb_ip = fu->getInPort("DP0_I2_MSB"); assert(i2_msb_ip);
	Port* p_ip = fu->getInPort("DP0_P"); assert(p_ip);
#endif
	std::map<std::string, std::string> currentPEmap = getConnectedPorts4(pe);
	std::map<std::string, std::string> prevPEmap = getConnectedPorts4(prevPE);
	std::map<std::string, std::string> prevPrevPEmap = getConnectedPorts4(prevPrevPE);
	std::map<std::string, std::string> nextPEmap = getConnectedPorts4(nextPE);


	PE * nextNextPE = this->cgra->getNextPE(nextPE);
	std::map<std::string, std::string> nextNextPEmap = getConnectedPorts4(nextNextPE);

	int x_max = 4;
	int y_max = 4;

	std::vector<std::vector<Port *>> classes;

	std::vector<Port *> classAP;
	std::vector<Port *> classBP;


	classes = getClasses(pe,4);



	int size = classes.size();
	std::vector<bool> active_class_prev(size,false);
	std::vector<bool> active_class_next(size,false);
	std::vector<bool> active_class(size,false);
	std::vector<bool> curr_class(size,false);

	std::vector<std::vector<Port *>> prev_active_list;
	std::vector<std::vector<Port *>> next_active_list;

	std::vector<Port *> commonPorts;
	std::vector <Port* > class_vec0 = classes[0];
	for (Port * element : class_vec0) {
		bool is_common = true;
		for(int i=1;i<size;i++){
			if(std::find(classes[i].begin(),classes[i].end(),element) == classes[i].end())
				is_common = false;
		}
		if(is_common)
			commonPorts.push_back(element);
	}
	std::map<int,bool> allPINV;
	std::map<int,bool> allCINV;
	std::map<int,bool> allNINV;

	for(int i=0;i<size;i++){
		bool prevAllINV = true;
		bool nextAllINV = true;
		bool currAllINV = true;
		std::vector <Port* > class_vec = classes[i];
		for (Port * element : class_vec) {
			std::string portName = element->getName();
			if(currentPEmap[portName] != "INV")
				currAllINV = false;
			if(prevPEmap[portName] != "INV")
				prevAllINV = false;
			if(nextPEmap[portName] != "INV")
				nextAllINV = false;
		}
		allPINV[i]=prevAllINV;
		allNINV[i]=nextAllINV;
		allCINV[i]=currAllINV;
	}


	std::vector<int> active_index;
	for(int i=0;i<size;i++){

		std::vector <Port* > class_vec = classes[i];


		for (Port * element : class_vec) {
			std::string portName = element->getName();
			if((std::find(commonPorts.begin(),commonPorts.end(),element) != commonPorts.end()) or (std::find(commonPorts.begin(),commonPorts.end(),next_to_src.second) != commonPorts.end()))
				continue;
			if((!(currentPEmap[portName]==prevPEmap[portName]) && !(next_to_src.second == element)))
			{

				if(!(!allPINV[i] && allCINV[i]))
					active_class_prev[i] = true;
			}

			if((!(currentPEmap[portName]==nextPEmap[portName]) && !(next_to_src.second == element)))
			{

				if(!(allNINV[i] && !allCINV[i]))
					active_class_next[i] = true;
			}

			if(next_to_src.second == element)
			{
				curr_class[i] = true;
			}


		}
		active_class[i] = active_class_prev[i] or active_class_next[i];

		if(active_class[i])
			active_index.push_back(i);
	}


	if(active_index.size() == 1)
	{
		if(curr_class[active_index.at(0)])
			cost =0;
		else if (std::find(commonPorts.begin(),commonPorts.end(),next_to_src.second) != commonPorts.end()){
			cost =0;
		}else {

			cost = ROUTER_COST;
		}
	}else if(active_index.size() > 1)
	{
		bool belong_to_active=false;
		for(int j:active_index)
		{
			if(curr_class[j])
				belong_to_active=true;
		}

		if(belong_to_active)
			cost =0;
		else {
			cost = ROUTER_COST;
		}
	}else if(active_index.size() == 0){
		cost =0;
	}
	return cost;
}
void CGRAXMLCompile::PathFinderMapper::printInterconnectData(CGRA* cgra) {
	std::cout << "Number of interconnects -> " << cgra->UsedLinks.size() << "\n";

	for(std::pair<Port *, Port *> port_pair:cgra->UsedLinks)
	{
		std::cout << "From -> " << port_pair.first->getFullName() << "  TO -> " << port_pair.second->getFullName() << "\n";
	}


	int invalid_const =0;
	int invalid_route =0;
	int invalid_opcode =0;


}

bool CGRAXMLCompile::PathFinderMapper::LeastCostPathAstar(LatPort start,
														  LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node,
														  std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode)
{


	std::unordered_map<LatPort, int, hash_LatPort> cost_to_port;
	std::unordered_map<LatPort, LatPort, hash_LatPort> cameFrom;
	std::unordered_map<LatPort, int, hash_LatPort> curr_hops_to_port;

	path.clear();
	mutexPaths.clear();

	bool detailedDebug = false;

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

			return dist_dest;
		}



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

	while (!q.empty())
	{
		port_heuristic curr = q.top();
		currPort = curr.p;
		q.pop();
		std::unordered_set<Port *> *currPath;
		std::vector<LatPort> *currPathVec;

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

		assert(curr_hops_to_port.find(currPort) != curr_hops_to_port.end());
		if(curr_hops_to_port[currPort] > cgra->max_hops){
			continue;
		}

		if (currPort == end)
		{
			if(cost_to_port[currPort] < curr_least_cost_to_end){
				curr_least_cost_to_end = cost_to_port[currPort];
			}
			continue;
		}

		if(cost_to_port[currPort] > curr_least_cost_to_end){
			continue;
		}


		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);


		int q_len = q.size();

		for (LatPort nextLatPort : nextPorts)
		{
			Port *nextPort = nextLatPort.second;

			if (nextLatPort.first > end.first)
				continue; //continue if the next port has higher latency
			assert(nextLatPort.first - currPort.first <= 1);



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

			if (newNodeDPOutCP.find(nextPort) != newNodeDPOutCP.end())
			{
				continue;
			}

			if (endPortCP.find(nextPort) != endPortCP.end())
			{
				continue;
			}



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
				// std::cout << "src=" << currPort.second->getFullName() << ",dest=" << nextLatPort.second->getFullName() << "\n";
				if (nextLatPort.first == currPort.first)
				{
					nextLatPort.first = nextLatPort.first + 1;
				}
			}

			if (true)
			{ // unmapped port
				if (detailedDebug)
					std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
				if (detailedDebug)
					std::cout << "latency = " << nextLatPort.first << ",";
				int nextPortCost = cost_to_port[currPort] + calculateCost(currPort, nextLatPort, end);


				if (nextPort->getNode() == node)
				{
					nextPortCost = cost_to_port[currPort];
				}

				if (checkRecParentViolation(currNode, nextLatPort))
				{
					std::cout << "Port is not inserted, since it violated recurrence parent..\n";
					continue;
				}
				if (detailedDebug)
					std::cout << "cost=" << nextPortCost << "\n";
				//					if(isNextPortMutex){
				//						//no cost is added in using mutually exclusive routes
				//						nextPortCost = cost_to_port[currPort];
				//					}

				if (nextPortCost < cost_to_port[currPort])
				{
					std::cout << "nextPortCost = " << nextPortCost << "\n";
					std::cout << "cost_to_port[currPort] = " << cost_to_port[currPort] << "\n";
				}
				assert(nextPortCost >= cost_to_port[currPort]);

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

						//							paths[nextLatPort]=paths[currPort];
						//							paths[nextLatPort].insert(nextLatPort.second);
						//							currPath.insert(currPort.second);

						//							pathsLatPort[nextLatPort]=pathsLatPort[currPort];
						//							pathsLatPort[nextLatPort].push_back(currPort);
						if (!lessthanII)
						{
							std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
							newPath->insert(currPort.second);
							port_heuristic ph(nextLatPort, end, nextPortCost, newPath);
							ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
							ph.pathVec->push_back(currPort);
							q.push(ph);
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


					if (!lessthanII)
					{
						std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
						newPath->insert(currPort.second);
						port_heuristic ph(nextLatPort, end, nextPortCost, newPath);
						ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
						ph.pathVec->push_back(currPort);
						q.push(ph);
					}
					else
					{
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

	//		if(detailedDebug) assert(false);

	//		if(currPort!=end){
	if (cameFrom.find(end) == cameFrom.end())
	{
		path.clear();
		for (LatPort p : deadEnds)
		{
			std::vector<LatPort> tmpPath;
			while (p != start)
			{
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



		return false; //routing failure
	}

	path.clear();
	//		assert(currPort==end);
	//		assert(currPort==end);
	currPort = end;
	while (currPort != start)
	{
		path.push_back(currPort);
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
		if (paths[end]->size() != path.size())
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
		}
		//			assert(paths[end]->size() == path.size());
		path.clear();
		path = finalPath;
	}


	return true;
}

int CGRAXMLCompile::PathFinderMapper::calculateCost(LatPort src,
													LatPort next_to_src, LatPort dest)
{

	std::string srcName = src.second->getName();
	//	std::cout << src->getName() << ",";

	std::string next_to_srcName = next_to_src.second->getName();
	//	std::cout << next_to_srcName << "\n";

	assert(src.second);
	assert(next_to_src.second);
	assert(dest.second);

	PE *srcPE = src.second->findParentPE();
	assert(srcPE);
	PE *nextPE = next_to_src.second->findParentPE();
	assert(nextPE);

	// int distance = abs(nextPE->Y - srcPE->Y) + abs(nextPE->X - srcPE->X) + regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());
	int distance = regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());

	distance = distance * PETransitionCostFactor + next_to_src.second->getCongCost() + PortTransitionCost;
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


		distance = distance + (nextPE->outputPorts.size() * 2 - (freePorts)) * UOPCostFactor;

		if (nextPE->outputPorts.size() * 2 < freePorts)
		{
			std::cout << "outportsize = " << nextPE->outputPorts.size() << "\n";
			std::cout << "freePorts = " << freePorts << "\n";
		}
	}


	assert(distance > 0);

	if ((next_to_src.second->getName().compare("P") == 0) || (next_to_src.second->getName().compare("I1") == 0) || (next_to_src.second->getName().compare("I2") == 0))
	{

		FU *fu = next_to_src.second->getMod()->getFU();
		if ((fu->supportedOPs.find("LOAD") != fu->supportedOPs.end()) && (dest == next_to_src))
		{
			double memrescost_dbl = (double)this->dfg->unmappedMemOps / (double)cgra->freeMemNodes;
			memrescost_dbl = memrescost_dbl * (double)MEMResourceCost;
			distance = distance + (int)memrescost_dbl;
			if (this->dfg->unmappedMemOps == cgra->freeMemNodes)
			{
				distance = distance + MRC * 10;
			}
		}
	}

	assert(distance > 0);
	return distance;
}

