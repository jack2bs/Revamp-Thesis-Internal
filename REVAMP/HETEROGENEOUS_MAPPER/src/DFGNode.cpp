/*
 * dfgnode.cpp
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 *      Edited By: Thilini
 */

#include "DFGNode.h"
#include "Port.h"
#include <algorithm>
#include <assert.h>
#include <iostream>
#include "DataPath.h"
#include "CGRA.h"
#include "DFG.h"
#include <bitset>

namespace CGRAXMLCompile
{

DFGNode::DFGNode()
{
	// TODO Auto-generated constructor stub
}

} /* namespace CGRAXMLCompile */

std::string CGRAXMLCompile::DFGNode::get27bitConstantBinaryString() {

       assert(hasConst);
       std::string s = std::bitset<27>(constant).to_string();
       return s;
}

std::string CGRAXMLCompile::DFGNode::getBinaryString() {

       if(op == "NOP"){
               return "00000";
       }
       else if(op == "ADD"){
               return "00001";
       }
       else if(op == "SUB"){
               return "00010";
       }
       else if(op == "MUL"){
               return "00011";
       }
       else if(op == "SEXT"){
               return "00100";
       }
       else if(op == "DIV"){
               return "00101";
       }
       else if(op == "INVDIV"){
               return "00111";
       }
       else if(op == "LS"){
               return "01000";
       }
       else if(op == "RS"){
               return "01001";
       }
       else if(op == "ARS"){
               return "01010";
       }
       else if(op == "AND"){
               return "01011";
       }
       else if(op == "OR"){
               return "01100";
       }
       else if(op == "XOR"){
               return "01101";
       }
       else if(op == "SELECT"){
               return "10000";
       }
       else if(op == "CMERGE"){
               return "10001";
       }
       else if(op == "CMP"){
               return "10010";
       }
       else if(op == "CLT"){
               return "10011";
       }
       else if(op == "BR"){
               return "10100";
       }
       else if(op == "BR"){
               return "10100";
       }
       else if(op == "CGT"){
               return "10101";
       }
       else if(op == "LOADCL"){
               return "10110";
       }
       else if(op == "MOVCL"){
               return "10111";
       }
       else if(op == "LOAD" || op == "OLOAD"){
               return "11000";
       }
       else if(op == "LOADH" || op == "OLOADH"){
               return "11001";
       }
       else if(op == "LOADB" || op == "OLOADB"){
               return "11010";
       }
       else if(op == "STORE" || op == "OSTORE"){
               return "11011";
       }
       else if(op == "STOREH" || op == "OSTOREH"){
               return "11100";
       }
       else if(op == "STOREB" || op == "OSTOREB"){
               return "11101";
       }
       else if(op == "ENDL"){
               return "11110";
       }
       else if(op == "END"){
               return "11110";
       }
       else if(op == "MOVC"){
               return "11111";
       }
       else{
               std::cout << "Op not compatible : " << op << "\n";
               assert(false);
       }

}


void CGRAXMLCompile::DFGNode::clear(DFG *dfg)
{

	if (rootDP != NULL)
	{
#ifdef DUAL_STREAMING

		rootDP->getOutputDP()->getOutPort("T_LSB")->clear();

#else
		//std::cout << "XXXXX\n"
		rootDP->getOutputDP()->getOutPort("T")->clear();
#endif

		rootDP->clear();

		CGRA *cgra = rootDP->getCGRA();
		FU *fu = rootDP->getFU();
		if (fu->supportedOPs.find("LOAD") != fu->supportedOPs.end())
		{
			cgra->freeMemNodes++;
			cgra->freeMemNodeSet.insert(rootDP);
		}

		if (this->isMemOp())
		{
			dfg->unmappedMemOps++;
			dfg->unmappedMemOpSet.insert(this);
		}

		rootDP = NULL;
	}

	for (std::pair<Port *, int> pair : routingPorts)
	{
		Port *p = pair.first;
		p->clear();
	}

	this->routingPorts.clear();
//	std::cout << "HERE\n";
	for (DFGNode *parent : parents)
	{
		std::set<std::pair<Port *, int>> delPorts;
		for (std::pair<Port *, int> pair : parent->routingPorts)
		{
			Port *p = pair.first;
			int destIdx = pair.second;
			//			assert(parent->routingPortDestMap.find(p)!=parent->routingPortDestMap.end());
			//			int dest_idx = routingPortDestMap[p];
			//			assert(dest!=NULL);
#ifdef DUAL_STREAMING
			if (parent->rootDP->getOutputDP()->getOutPort("T_LSB") == p)
				continue;
#else
			if (parent->rootDP->getOutputDP()->getOutPort("T") == p)
							continue;
#endif
			if (destIdx == this->idx)
			{
				p->clear();
				delPorts.insert(pair);
			}
		}
		std::cout << "delPorts.size = " << delPorts.size() << "\n";
		std::cout << "parentRoutingPort size(before) = " << parent->routingPorts.size() << "\n";
		for (std::pair<Port *, int> pair : delPorts)
		{
			parent->routingPorts.erase(std::find(parent->routingPorts.begin(), parent->routingPorts.end(), pair));
		}
		std::cout << "parentRoutingPort size(after) = " << parent->routingPorts.size() << "\n";
	}
	//std::cout << "HERE\n";
}

std::string CGRAXMLCompile::DFGNode::getOPtype(DFGNode *child)
{
	assert(std::find(children.begin(), children.end(), child) != children.end());
	assert(childrenOPType.find(child) != childrenOPType.end());
	return childrenOPType[child];
}

bool CGRAXMLCompile::DFGNode::isMemOp()
{
	if (this->op.compare("LOAD") == 0)
	{
		return true;
	}
	else if (this->op.compare("LOADH") == 0)
	{
		return true;
	}
	else if (this->op.compare("LOADB") == 0)
	{
		return true;
	}
	else if (this->op.compare("STORE") == 0)
	{
		return true;
	}
	else if (this->op.compare("STOREH") == 0)
	{
		return true;
	}
	else if (this->op.compare("STOREB") == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
