/*
 * DataPath.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef DATAPATH_H_
#define DATAPATH_H_
#include <morpher/arch/Module.h>
#include <morpher/dfg/DFGNode.h>
#include <assert.h>

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
		inputPorts.push_back(new Port("P", IN, this));
		inputPorts.push_back(new Port("I1", IN, this));
		inputPorts.push_back(new Port("I2", IN, this));
		inputPorts.push_back(new Port("I3", IN, this));
		inputPorts.push_back(new Port("I4", IN, this));


		//create outputPorts
		outputPorts.push_back(new Port("T", OUT, this));

		//create output to input connection
		//		connectedTo[getOutPort("T")].push_back(getInPort("P"));
		//		connectedTo[getOutPort("T")].push_back(getInPort("I1"));
		//		connectedTo[getOutPort("T")].push_back(getInPort("I2"));
		insertConnection(getOutPort("T"), getInPort("P"));
		insertConnection(getOutPort("T"), getInPort("I1"));
		insertConnection(getOutPort("T"), getInPort("I2"));
		insertConnection(getOutPort("T"), getInPort("I3"));
		insertConnection(getOutPort("T"), getInPort("I4"));

		mappedNode = NULL;
		outputDP = NULL;
	}

	FU *getFU();
	PE *getPE();
	CGRA *getCGRA();
	Port *getOutputPort(int latency);

	void assignNode(DFGNode *node, int lat, DFG *dfg);
	DFGNode *getMappedNode() { return mappedNode; }
	DFGNode *getReservedNode() { return reservedNode; }
	DFGNode *setReservedNode(DFGNode *node) { reservedNode = node; }

	DataPath *getOutputDP() { return outputDP; }
	void clear();

	Port *getPotOutputPort(DFGNode *node);
	int getLat() { return latency; }

	unordered_set<string> accesible_memvars;

#ifdef FLEX
	bool isReserved = false;
#endif
	

private:
	DFGNode *mappedNode;
	DFGNode *reservedNode;
	DataPath *outputDP;
	int latency = -1;
};

} /* namespace CGRAXMLCompile */

#endif /* DATAPATH_H_ */
