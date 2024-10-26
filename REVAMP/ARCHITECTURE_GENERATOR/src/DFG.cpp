/*
 * DFG.cpp
 *
 *      Author: thilini
 */

#include "DFG.h"
#include <assert.h>
#include <iostream>
#include <queue>
#include <algorithm>

#include "tinyxml2.h"
using namespace tinyxml2;

namespace revamp
{
DFG::DFG()
{
	// TODO Auto-generated constructor stub
}

bool DFG::parseXML(std::string fileName)
{

	XMLDocument doc;
	doc.LoadFile(fileName.c_str());
	XMLElement *DFG = doc.FirstChildElement("DFG");

	int node_count;

	DFG->QueryIntAttribute("count", &node_count);

	XMLElement *node = NULL;
	int initial_nodelist = this->nodeOpList.size();

	for (int i = 0; i < node_count; ++i)
	{
		if (node == NULL)
		{
			node = DFG->FirstChildElement("Node");
		}
		else
		{
			node = node->NextSiblingElement("Node");
		}
		assert(node);

		XMLElement *op = node->FirstChildElement("OP");
		const char *opName = op->GetText();

		this->nodeOpList.push_back(opName);
	}

	std::cout << fileName << " = " <<this->nodeOpList.size()-initial_nodelist << "\n";
	return 1;
}

opCount DFG::analyzeOps()
{
	for (std::string &nodeOp : nodeOpList)
	{
		if(std::find(this->adder.begin(), this->adder.end(), nodeOp) != this->adder.end()){
			this->opcount.add_count +=1;
		}else if(std::find(this->logic.begin(), this->logic.end(), nodeOp) != this->logic.end()){
			this->opcount.logic_count +=1;
		}else if(std::find(this->comparator.begin(), this->comparator.end(), nodeOp) != this->comparator.end()){
			this->opcount.cmp_count +=1;
		}else if(std::find(this->multiplier.begin(), this->multiplier.end(), nodeOp) != this->multiplier.end()){
			this->opcount.mul_count +=1;
		}else if(std::find(this->common.begin(), this->common.end(), nodeOp) != this->common.end()){
			this->opcount.common_count +=1;
		}else if(std::find(this->memory.begin(), this->memory.end(), nodeOp) != this->memory.end()){
			this->opcount.mem_count +=1;
		}else
		{
			std::cout << "Operand not in list OP: "<< nodeOp << "\n";
		}

	}
	int total_hetero_count = this->opcount.add_count+this->opcount.logic_count+this->opcount.cmp_count+this->opcount.mul_count;
	std::cout << "\nOperation Distribution:\n";
	std::cout << "ADDER:" << this->opcount.add_count << "\nMULTIPLIER:" << this->opcount.mul_count <<"\nLOGIC:"<< this->opcount.logic_count <<"\nCMP:" << this->opcount.cmp_count <<"\n";
	std::cout << "ADDER:" << this->opcount.add_count*100/total_hetero_count << "%\nMULTIPLIER:" << this->opcount.mul_count*100/total_hetero_count <<"%\nLOGIC:"<< this->opcount.logic_count*100/total_hetero_count <<"%\nCMP:" << this->opcount.cmp_count*100/total_hetero_count <<"%\n";
	return this->opcount;
}

void DFG::printDFG()
{
	std::cout << "Printing DFG Ops...\n";
	for (std::string &nodeOp : nodeOpList)
	{
		std::cout << "OP: "<< nodeOp << "\n";
	}
	std::cout << "Printing DFG Ops Done!\n";
}

void DFG::initialize()
{
	this->opcount.add_count=0;
	this->opcount.logic_count =0;
	this->opcount.cmp_count =0;
	this->opcount.mul_count =0;
	this->opcount.mem_count =0;
	this->opcount.common_count =0;

}
}
