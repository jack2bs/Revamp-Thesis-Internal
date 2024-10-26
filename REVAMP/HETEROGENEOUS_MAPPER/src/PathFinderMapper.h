/*
 * PathFinderMapper.h
 *
 *  Created on: 31 Mar 2018
 *      Author: manupa
 *      Edited By: Thilini
 */

#include "HeuristicMapper.h"
#include <string>

#ifndef PATHFINDERMAPPER_H_
#define PATHFINDERMAPPER_H_

namespace CGRAXMLCompile
{

#define LARGE_VALUE 100000000
#define ROUTER_COST 10000000
#define SAME_PE_COST 10000
//#define DUAL_STREAMING
//Enable the optimizations individually
//#define OPCODE_CONST_CONFIG
//#define ROUTER_CONFIG
//#define STREAMING

struct beParentInfo
{
	DFGNode *beParent;
	int lat;
	int downStreamOps;

	bool dsMEMfound = false;
	int uptoMEMops = -1;
	bool isLDST = false;

	bool operator<(const beParentInfo &other) const
	{
		return this->beParent < other.beParent;
	}
};

struct InsFormat{
	std::string treg_we_msb;
	std::string alu_i1_msb;
	std::string alu_i2_msb;
	std::string easto;
	std::string southo;
	std::string westo;
	std::string northo;
	std::string alu_i1;
	std::string alu_i2;
	std::string alu_p;

	std::string treg_we;

	std::string east_reg_we;
	std::string south_reg_we;
	std::string west_reg_we;
	std::string north_reg_we;

	std::string east_reg_bypass;
	std::string south_reg_bypass;
	std::string west_reg_bypass;
	std::string north_reg_bypass;

	std::string opcode;
	std::string constant;
	std::string constant_valid;
	std::string is_predicate;
	std::string negated_predicate;
};

struct InsFormatToggle{
	std::string treg_we_msb;
	std::string classRout;
	std::string isChanged;

	std::string treg_we;

	std::string east_reg_we;
	std::string south_reg_we;
	std::string west_reg_we;
	std::string north_reg_we;

	std::string east_reg_bypass;
	std::string south_reg_bypass;
	std::string west_reg_bypass;
	std::string north_reg_bypass;

	std::string isChangeOp;
	std::string isChangconst;
	std::string constant_valid;
	std::string negated_predicate;
};

struct InsFormatEnable{
	std::string treg_we_msb;
	std::string treg_we;

	std::string east_reg_we;
	std::string south_reg_we;
	std::string west_reg_we;
	std::string north_reg_we;

	std::string east_reg_bypass;
	std::string south_reg_bypass;
	std::string west_reg_bypass;
	std::string north_reg_bypass;
};


struct InsFormatOpcode{
	std::string treg_we_msb;
	std::string alu_i1_msb;
	std::string alu_i2_msb;
	std::string easto;
	std::string southo;
	std::string westo;
	std::string northo;
	std::string alu_i1;
	std::string alu_i2;
	std::string alu_p;

	std::string treg_we;

	std::string east_reg_we;
	std::string south_reg_we;
	std::string west_reg_we;
	std::string north_reg_we;

	std::string east_reg_bypass;
	std::string south_reg_bypass;
	std::string west_reg_bypass;
	std::string north_reg_bypass;

	std::string opcode;
	std::string constant;
	std::string constant_valid;
	std::string is_predicate;
	std::string negated_predicate;
};

struct InsFormatRouterNoHot{
	std::string out0;
	std::string out1;
	std::string out2;
	std::string out3;
	std::string out4;
};

struct InsFormatRouterHot{
	std::string out0;
	std::string out1;
	std::string out2;
	std::string out3;
	std::string out4;
	std::string out5;
};

class PathFinderMapper : public HeuristicMapper
{
public:
	std::vector<DataPath *> usedDPs;
	std::vector<std::pair<DataPath *,bool>> candidateDestsModi;
	PathFinderMapper(std::string fName) : HeuristicMapper(fName){

										  };

	bool Map(CGRA *cgra, DFG *dfg);
	//	bool LeastCostPathAstar(Port* start, Port* end, DataPath* endDP, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool LeastCostPathAstar(LatPort start, LatPort end, DataPath *endDP, std::vector<LatPort> &path,std::vector<LatPort> pathMSB, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode,bool isStreaming,bool isPtoC);
	bool LeastCostPathAstar(LatPort start, LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode);

	bool estimateRouting(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	int predictiveRoute(DFGNode *node,
						DataPath *dest,
						const std::map<DFGNode *, Port *> routingSourcesIn,
						const std::map<DFGNode *, DataPath *> mappedNodesIn,
						std::map<DFGNode *, Port *> &routingSourcesOut,
						std::map<DFGNode *, DataPath *> &mappedNodesOut);

	bool Route(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	int calculateCost(LatPort src, LatPort next_to_src, LatPort dest,bool isStreaming,bool isPtoC, std::vector<LatPort> regs);
	int calculateCost(LatPort src, LatPort next_to_src, LatPort dest);
	void assignPath(DFGNode *src, DFGNode *dest, std::vector<LatPort> path);

	bool updateCongestionCosts(int iter);
	bool clearCurrMapping();
	std::map<Port *, std::set<DFGNode *>> *getcongestedPortsPtr() { return &congestedPorts; }
	std::map<Port *, std::set<DFGNode *>> *getconflictedPortsPtr() { return &conflictedPorts; }

	bool checkConflictedPortCompatibility();
	bool checkRegALUConflicts();
	bool checkDPFree(DataPath *dp, DFGNode *node, int &penalty);

	bool updateConflictedTimeSteps(int timeStep, int conflicts);
	int getTimeStepConflicts(int timeStep);

	void sortBackEdgePriorityASAP();
	void sortBackEdgePriorityALAP();
	void sortBackEdgePriorityFaninFanoutMax();
	void sortBackEdgePriorityASAPALP();
	std::ofstream congestionInfoFile;

	void addPseudoEdgesOrphans(DFG *dfg);

	std::vector<DFGNode *> getLongestDFGPath(DFGNode *src, DFGNode *dest);
	int getFreeMEMPeDist(PE *currPE);

	bool canExitCurrPE(LatPort p);
	static bool checkMEMOp(string op);
	void setMaxIter(int m){maxIter = m;}

	bool Check_DFG_CGRA_Compatibility();

	void UpdateVariableBaseAddr();

	void printHyCUBEBinary(CGRA* cgra);
	void printHyCUBEBinary4(CGRA* cgra);
	void printHyCUBEConfig(const std::vector<std::vector<std::vector<InsFormat>>>& insFArr, std::string fName, CGRA* cgra);
	void printBinFile(const std::vector<std::vector<std::vector<InsFormat>>>& insFArr, std::string fName, CGRA* cgra);
	void printInterconnectData(CGRA* cgra);

private:
	std::map<Port *, std::set<DFGNode *>> congestedPorts;
	std::map<Port *, std::set<DFGNode *>> conflictedPorts;
	int maxIter = 10;
	int const_cost = 5000;
	int opcode_cost = 5000;

	std::map<int, int> conflictedTimeStepMap;

	std::set<DFGNode *> RecPHIs;
	int getlatMinStartsPHI(const DFGNode *currNode, const std::map<DFGNode *, std::vector<Port *>> &possibleStarts);
	std::set<DFGNode *> getElders(DFGNode *node);

	std::map<BackEdge, std::set<DFGNode *>> RecCycles;
	std::map<BackEdge, std::set<DFGNode *>> RecCyclesLS;
	int getMaxLatencyBE(DFGNode *node, std::map<DataPath *, beParentInfo> &beParentDests, int &downStreamOps);
	std::vector<DataPath *> modifyMaxLatCandDest(std::map<DataPath *, int> candDestIn, DFGNode *node, bool &changed);
	std::map<DataPath *, int> checkConstSimilarity(std::vector<DataPath *> candDestIn,DFGNode * node);
	std::map<DataPath *, int> checkOpcodeSimilarity(std::vector<DataPath *> candDestIn,DFGNode * node);
	int getCrossbarSimilarityCost(LatPort src,LatPort next_to_src);
	int getCrossbarSimilarityCostwithSim(LatPort src,LatPort next_to_src);
	int getCrossbarSimilarityCostHetero(LatPort src,LatPort next_to_src,bool multiCycle);
	int getCrossbarSimilarityCostDual(LatPort src,LatPort next_to_src,bool multiCycle);
	int getCrossbarSimilarityCostDualclass4(LatPort src,LatPort next_to_src,bool multiCycle);
	int getCrossbarSimilarityCostDualclass4n(LatPort src,LatPort next_to_src,bool multiCycle);
	//std::map<Port *, Port *> getConnectedPorts(PE * pe);
	std::map<std::string, std::string> getConnectedPorts(PE * pe);
	std::map<std::string, std::string> getConnectedPorts4(PE * pe);
	std::vector<DataPath *> removeConstFilledCandidates(std::vector<DataPath *> candDestIn, std::map<DataPath *,int> constDest, std::map<DataPath *,int> opcodeDests);
	void GetAllSupportedOPs(Module* currmod, unordered_set<string>& supp_ops, unordered_set<string>& supp_pointers);
	std::vector<DataPath *> modifyUsedConnCandDest(std::vector<DataPath *> candDestIn, std::vector<Port *> availablePorts, bool &changed1);
	std::vector<CGRAXMLCompile::DataPath *> modifyLatencyDest(std::vector<DataPath *> candDestIn, DFGNode *node,int diff);
	std::vector<std::pair<CGRAXMLCompile::DataPath *,bool>> prevTimeDest(std::vector<DataPath *> candDestIn,DFGNode * node,int diff);
	std::vector<CGRAXMLCompile::DataPath *> modifyUsedDest(std::vector<DataPath *> candDestIn, DFGNode *node);
	std::vector<std::pair<CGRAXMLCompile::DataPath *,std::pair<std::vector<CGRAXMLCompile::DFGNode *>,std::vector<CGRAXMLCompile::DFGNode *>>>> modifyLatencyDestUnordered(std::vector<DataPath *> candDestIn, DFGNode *node);
	CGRAXMLCompile::DataPath * checkDP(DataPath * candDestIn, std::vector<std::pair <DFGNode *,bool>> mappedList, DFGNode *node,bool isParent);
	CGRAXMLCompile::DataPath * findNextDP(DataPath * currDP);
	std::vector<LatPort> getNextPath(std::vector<LatPort> path);
	std::vector<LatPort> getPrevPath(std::vector<LatPort> path);
	LatPort getNextLatPort(LatPort port);
	LatPort getPrevLatPort(LatPort port);
	std::vector<std::vector<Port *>> getClasses(PE* pe,int num);
};

} /* namespace CGRAXMLCompile */

#endif /* PATHFINDERMAPPER_H_ */
