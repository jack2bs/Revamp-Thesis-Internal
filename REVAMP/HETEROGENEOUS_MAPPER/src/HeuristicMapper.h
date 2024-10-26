/*
 * HeuristicMapper.h
 *
 *  Created on: 28 Feb 2018
 *      Author: manupa
 *      Edited By: Thilini
 */

#ifndef HEURISTICMAPPER_H_
#define HEURISTICMAPPER_H_

#include "DFG.h"
#include "CGRA.h"
#include "DataPath.h"
#include <queue>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <math.h>

#define MRC 100000
#define UOP 1000
//#define DUAL_STREAMING
namespace CGRAXMLCompile
{

//struct definitions
class HeuristicMapper;

struct dest_child_with_cost
{
	DFGNode *child;
	DataPath* childDP;
	LatPort childDest;
	LatPort startPort;
	int cost;

	dest_child_with_cost(DFGNode *child, DataPath* dp, LatPort childDest, LatPort startPort, int cost) : child(child), childDP(dp), childDest(childDest), startPort(startPort), cost(cost) {}
	bool operator<(const dest_child_with_cost &rhs) const
	{
		return cost > rhs.cost;
	}
};

struct cand_src_with_cost
{
	LatPort src;
	LatPort dest;
	int cost;
	cand_src_with_cost(LatPort src, LatPort dest, int cost) : src(src), dest(dest), cost(cost) {}

	bool operator<(const cand_src_with_cost &rhs) const
	{
		return cost > rhs.cost;
	}
};

struct parent_cand_src_with_cost
{
	DFGNode *parent;
	std::priority_queue<cand_src_with_cost> cswc;
	int cost;
	parent_cand_src_with_cost(DFGNode *parent, std::priority_queue<cand_src_with_cost> cswc) : parent(parent), cswc(cswc)
	{
		cost = cswc.top().cost;
	}

	bool operator<(const parent_cand_src_with_cost &rhs) const
	{
		return this->cost > rhs.cost;
	}
};

struct dest_with_cost
{
	//	std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
	std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
	std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
	int bestCost=0;
	DataPath *dest;
	int destLat;
	DFGNode *node;
	dest_with_cost() {}
	dest_with_cost(std::priority_queue<parent_cand_src_with_cost> parentStartLocs,
				   std::priority_queue<dest_child_with_cost> alreadyMappedChilds,
				   DataPath *dest, int destLat, DFGNode *node, int cost,
				   int unmappedMemNodeCount, HeuristicMapper *hm) : parentStartLocs(parentStartLocs), alreadyMappedChilds(alreadyMappedChilds), dest(dest), destLat(destLat), node(node)
	{
		if(bestCost==0)
			bestCost = sumBestCosts(unmappedMemNodeCount, hm);
	}

	int sumBestCosts(int unmappedMemNodeCount, HeuristicMapper *hm)
	{
		//std::cout << "HERE_X0\n";
		int cost = 0;
		//		for(std::pair<DFGNode*,std::priority_queue<cand_src_with_cost>> pair : parentStartLocs){
		//			assert(!pair.second.empty());
		//			cost+=pair.second.top().cost;
		//		}
		std::priority_queue<parent_cand_src_with_cost> parentStartLocsCopy = parentStartLocs;
		while (!parentStartLocsCopy.empty())
		{
			parent_cand_src_with_cost pcswc = parentStartLocsCopy.top();
			parentStartLocsCopy.pop();
			cost += pcswc.cswc.top().cost;
		}

		//		std::cout << "sumBestCosts :: alreadyMappedChilds size=" << alreadyMappedChilds.size() << "\n";
		std::priority_queue<dest_child_with_cost> alreadyMappedChildCopy = alreadyMappedChilds;
		while (!alreadyMappedChildCopy.empty())
		{
			dest_child_with_cost dcwc = alreadyMappedChildCopy.top();
			alreadyMappedChildCopy.pop();
			cost += dcwc.cost;
		}

		//		std::cout << "sumBestCosts :: alreadyMappedChilds size=" << alreadyMappedChilds.size() << "\n";
		//std::cout << "HERE_X2\n";
		if (cost == 0)
		{
			int freePorts = 0;
			//for (Port *p : dest->getPE()->outputPorts)
			for (Port *p : dest->getPE()->outputPorts)
			{
				Module *parent = dest->getPE()->getParent();
				if (parent->getNextPorts(std::make_pair(dest->getPE()->T, p), hm).empty())
					continue;
				if (p->getNode() == NULL)
				{
					freePorts++;
				}
			}
			//			cost = cost + (dest->getPE()->outputPorts.size() - freePorts)*UOP;
			//cost = cost + (15 - freePorts) * UOP;
			PE *destPE = dest->getPE();
			CGRA *cgra = destPE->getCGRA();
#ifndef TORUS
			//change this to 2x for cov2d
			//PE *destPE = dest->getPE();
			//CGRA *cgra = destPE->getCGRA();
			if((destPE->X==0 && destPE->Y==0) or (destPE->X==0 && destPE->Y==(cgra->get_y_max()-1)) or (destPE->X==(cgra->get_x_max()-1) && destPE->Y==0) or (destPE->X==(cgra->get_x_max()-1) && destPE->Y==(cgra->get_y_max()-1)))
			{
				cost = cost + (2 - freePorts) * UOP;
			}else if(destPE->X==0 or destPE->Y==0 or destPE->Y==(cgra->get_y_max()-1) or destPE->X==(cgra->get_x_max()-1))
			{
				cost = cost + (3 - freePorts) * UOP;
			}else{
				cost = cost + ((dest->getPE()->outputPorts.size()) - freePorts) * UOP;
			}
#else
			cost = cost + (dest->getPE()->outputPorts.size()*2 - freePorts) * UOP;
#endif
			//if(node->idx ==42)
			// std::cout << "PAMU:: dest ::" << dest->getPE()->getName() << " free ports = " << cost << "\n";

			int primaryCost = 0;
			for (DFGNode *child : node->children)
			{
				for (DFGNode *parentil : child->parents)
				{
					if (parentil->rootDP != NULL && parentil != node)
					{
						PE *parentilPE = parentil->rootDP->getPE();
						PE *destPE = dest->getPE();
						CGRA *cgra = destPE->getCGRA();

						// int dx = std::abs(parentilPE->X - destPE->X);
						// int dy = std::abs(parentilPE->Y - destPE->Y);
						int dt = (destPE->T - parentilPE->T + cgra->get_t_max()) % cgra->get_t_max();
						primaryCost = primaryCost + dt;
					}
				}
			}
			//parse json branch temporally not using dx dy costs due to templates -- removed again

			cost = cost + primaryCost * 100;
			//if(node->idx ==42)
			//		 std::cout << "PAMU:: dest ::" << dest->getPE()->getName() << " primary = " << primaryCost << " total =" << cost<< "\n";
			//std::cout << "HERE_X3\n";
			int secondaryCost = 0;
			for (DFGNode *child : node->children)
			{
				for (DFGNode *childp : child->parents)
				{
					if ((childp != child) && (childp->rootDP != NULL))
					{
						PE *childpPE = childp->rootDP->getPE();
						PE *destPE = dest->getPE();


						int dx = std::abs(childpPE->X - destPE->X);
						int dy = std::abs(childpPE->Y - destPE->Y);
						secondaryCost = secondaryCost + dx + dy;

					}
				}
				for (DFGNode *childchild : node->children)
				{
					for (DFGNode *childparent : childchild->parents)
					{
						if (childparent != childchild)
						{
							for (DFGNode *parent : childparent->parents)
							{
								if (parent != node && parent->rootDP != NULL)
								{
									PE *parentilPE = parent->rootDP->getPE();
									PE *destPE = dest->getPE();
									CGRA *cgra = destPE->getCGRA();

									 int dx = std::abs(parentilPE->X - destPE->X);
									 int dy = std::abs(parentilPE->Y - destPE->Y);
									 int dt = (destPE->T - parentilPE->T + cgra->get_t_max()) % cgra->get_t_max();
									// secondaryCost = secondaryCost + dx + dy + dt;
								}
							}
						}
					}
				}
			}


			//near congestion cost
			//			std::cout << "HERE1\n";
			if(node->op == "OLOAD" or node->op == "LOAD" or node->op == "LOADB")
			{
				/*PE* neigh;

				if(dest->getPE()->X == cgra->get_x_max()-1){
					PE* neigh1 = dest->getPE()->getOutPort("WEST_O")->getMod()->;
					std::cout << neigh1->getFullName() << "\n";
					neigh = neigh1->getParent()->getNextTimeIns()->getNextTimeIns()->getPE();
					//neigh =cgra1->getPE((dest->getPE()->T+2)%cgra1->get_t_max(), dest->getPE()->Y, dest->getPE()->X -1);
				}else if(dest->getPE()->X == 0)
				{
					PE* neigh1 = dest->getPE()->getOutPort("EAST_O")->getPE();
					std::cout << neigh1->getFullName() << "\n";
					neigh = neigh1->getParent()->getNextTimeIns()->getNextTimeIns()->getPE();
					//neigh =cgra1->getPE((dest->getPE()->T+2)%cgra1->get_t_max(), dest->getPE()->Y, dest->getPE()->X +1);
				}

				int fp=0;

				if(neigh != NULL){
					std::cout << neigh->getFullName() << "\n";
				//Module *parent = neigh->getParent();
				for (Port *p : neigh->outputPorts)
				{
					std::cout << p->getFullName() << "\n";
					if (p->getNode() == NULL)
					{
						fp++;
					}
				}
			//	std::cout << "HERE\n";
				for (Module *submod : neigh->subModules)
				{
					if (FU *fu = dynamic_cast<FU *>(submod))
					{
						for (Module *submodFU : fu->subModules)
						{
							if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
							{
								if(dp->getMappedNode() != NULL)
								{
									cost =cost +1000;
								}
							}
						}
					}

				}

				cost = cost + (4-fp)*10;
				}
				std::cout << "HERE2\n";*/
				cost = cost + secondaryCost*10;
				//FFT specific
				if(node->idx ==26)
				{
					int delta =  destPE->getCGRA()->get_x_max() - destPE->X;
					cost = cost + delta*1000;
				}

			}else{
				cost = cost + secondaryCost*1000;
			}
			//if(node->idx ==42)
			//	std::cout << "PAMU:: dest ::" << dest->getPE()->getName() << " secondary = " << secondaryCost<< " total =" << cost << "\n";


			//std::cout << "HERE3\n";
			FU *fu = dest->getFU();
			//CGRA *cgra = dest->getCGRA();
			int memcost = 0;
			if ((fu->supportedOPs.find("LOAD") != fu->supportedOPs.end()) or (fu->supportedOPs.find("OLOAD") != fu->supportedOPs.end()))
			{
				double memrescost_dbl = (double)unmappedMemNodeCount / (double)cgra->freeMemNodes;
				memrescost_dbl = memrescost_dbl * (double)MRC;
				memcost = (int)memrescost_dbl;
			}

			cost = cost + memcost;
			//if(node->idx ==42)
			//	std::cout << "PAMU:: dest ::" << dest->getPE()->getName() << " mem = " << memcost << " total =" << cost <<"\n";

			//std::cout << "HERE_X4\n";
			//std::cout << dest->getPE()->getName() << ",cost=" << cost << "\n";

			if((dest->getPE()->is_hot) && ((node->children.size() + node->parents.size()) < 3))
				cost = cost + 10000;
		}

		cost += getDestTportCost();

		//std::cout << "HERE_X1\n";
		//if(node->idx ==42)
		//		std::cout << "PAMU:: dest ::" << dest->getPE()->getName() << " destPortCost = " << getDestTportCost()<< " total =" << cost << "\n";
		return cost;
	}

	int getDestTportCost()
	{
		PE *destPE = dest->getPE();
		FU *destFU = dest->getFU();

		int latency = destFU->supportedOPs[node->op];
		Port *outputPort = dest->getOutputPort(latency);
		return outputPort->getCongCost();
	}

	bool operator<(const dest_with_cost &rhs) const
	{
		return bestCost > rhs.bestCost;
	}

};

class HeuristicMapper
{
public:
	//	HeuristicMapper(CGRA* cgra, DFG* dfg) : cgra(cgra), dfg(dfg){};
	HeuristicMapper(std::string fName)
	{
		fNameLog1 = fName;
		//		mappingLog.open(fName.c_str());
		//		mappingLog2.open(fName2.c_str());
	}
	virtual ~HeuristicMapper(){};
	CGRA *cgra;
	DFG *dfg;

	int getMinimumII(CGRA *cgra, DFG *dfg);
	void SortTopoGraphicalDFG();
	void SortSCCDFG();
	bool Map(CGRA *cgra, DFG *dfg);
	//	bool LeastCostPathAstar(Port* start, Port* end, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool LeastCostPathAstar(LatPort start, LatPort end, std::vector<LatPort> &path, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode);
	bool LeastCostPathDjk(Port *start, Port *end, std::vector<Port *> &path, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths);
	//	int calculateCost(Port* src, Port* next_to_src, Port* dest);
	int calculateCost(LatPort src, LatPort next_to_src, LatPort dest);

	bool estimateRouting(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	bool Route(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);

	void assignPath(DFGNode *src, DFGNode *dest, std::vector<LatPort> path);
	bool dataPathCheck(DataPath *dp, DFGNode *node);

	bool sanityCheck();

	bool enableBackTracking = false;
	bool enableMutexPaths = false;
	int backTrackLimit = 4;

	void printMappingLog();
	void printMappingLog2();

	bool checkRecParentViolation(DFGNode *node, LatPort nextPort);

	int upperboundII = 1000000;
	int upperboundIter = -1;
	int upperboundFoundBy = -1;

protected:
	int regDiscourageFactor = 1000;
	int PETransitionCostFactor = 100;
	int PortTransitionCost = 1;
	int UOPCostFactor = UOP;
	int MEMResourceCost = MRC;

	std::string fNameLog1;
	std::string fNameLog2;

	std::ofstream usedPLog;

	std::ofstream mappingLog;
	std::ofstream mappingLog2;
	std::vector<DFGNode *> sortedNodeList;

	void removeFailedNode(std::stack<DFGNode *> &mappedNodes, std::stack<DFGNode *> &unmappedNodes, DFGNode *failedNode);

	int getlatMinStarts(const std::map<DFGNode *, std::vector<Port *>> &possibleStarts);
	std::map<DataPath *, int> getLatCandDests(const std::vector<DataPath *> &candidateDests, int minlat);
};

} /* namespace CGRAXMLCompile */

#endif /* HEURISTICMAPPER_H_ */
