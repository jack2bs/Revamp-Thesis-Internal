/*
 * DataPath.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 *      Edited By: Thilini
 */

#ifndef DATAPATH_H_
#define DATAPATH_H_
#include "Module.h"
#include "DFGNode.h"
#include <assert.h>
//#define DUAL_STREAMING
namespace CGRAXMLCompile
{

class FU;
class PE;
class CGRA;

class DataPath : public Module
{
public:
	DataPath(const Module *Parent, std::string name, int t) : Module(Parent, name, "DP", t)
	{
		//create inputPorts
#ifdef DUAL_STREAMING
		inputPorts.push_back(new Port("P", IN, this));
		inputPorts.push_back(new Port("I1_LSB", IN, this));
		inputPorts.push_back(new Port("I1_MSB", IN, this));
		inputPorts.push_back(new Port("I2_LSB", IN, this));
		inputPorts.push_back(new Port("I2_MSB", IN, this));
#else
		inputPorts.push_back(new Port("P", IN, this));
		inputPorts.push_back(new Port("I1", IN, this));
		inputPorts.push_back(new Port("I2", IN, this));
#endif
		//create outputPorts
#ifdef DUAL_STREAMING
		outputPorts.push_back(new Port("T_LSB", OUT, this));
		outputPorts.push_back(new Port("T_MSB", OUT, this));
#else
		outputPorts.push_back(new Port("T", OUT, this));
#endif
		//create output to input connection
		//		connectedTo[getOutPort("T")].push_back(getInPort("P"));
		//		connectedTo[getOutPort("T")].push_back(getInPort("I1"));
		//		connectedTo[getOutPort("T")].push_back(getInPort("I2"));
#ifdef DUAL_STREAMING
		insertConnection(getOutPort("T_LSB"), getInPort("P"));
		insertConnection(getOutPort("T_LSB"), getInPort("I1_LSB"));
		insertConnection(getOutPort("T_MSB"), getInPort("I1_MSB"));
		insertConnection(getOutPort("T_LSB"), getInPort("I2_LSB"));
		insertConnection(getOutPort("T_MSB"), getInPort("I2_MSB"));
#else
		insertConnection(getOutPort("T"), getInPort("P"));
		insertConnection(getOutPort("T"), getInPort("I1"));
		insertConnection(getOutPort("T"), getInPort("I2"));
#endif

		mappedNode = NULL;
		outputDP = NULL;
	}

	FU *getFU();
	PE *getPE();
	CGRA *getCGRA();
	Port *getOutputPort(int latency);
	Port *getOutputPort(int latency, bool isMSB);

	void assignNode(DFGNode *node, int lat, DFG *dfg);
	DFGNode *getMappedNode() { return mappedNode; }
	DataPath *getOutputDP() { return outputDP; }
	void clear();

	Port *getPotOutputPort(DFGNode *node);
	int getLat() { return latency; }

	unordered_set<string> accesible_memvars;

private:
	DFGNode *mappedNode;
	DataPath *outputDP;
	int latency = -1;
};

} /* namespace CGRAXMLCompile */

#endif /* DATAPATH_H_ */
