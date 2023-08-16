/*
 * PathFinderMapper.h
 *
 *  Created on: 31 Mar 2018
 *      Author: manupa
 */

#include <morpher/mapper/HeuristicMapper.h>
#include <morpher/util/debug.h>
#include <string>

#ifndef PATHFINDERMAPPER_H_
#define PATHFINDERMAPPER_H_

namespace CGRAXMLCompile
{

#define LARGE_VALUE 100000000
#define FLEX
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
	std::string negated_predicate;
	std::string rem_zeros;
};

struct InsFormatMem{
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
	std::string shiftVal;
	std::string offsetval;

	std::string negated_predicate;
};

struct InsFormatIter{
	std::string stride;
	std::string maxCount;
};





class PathFinderMapper : public HeuristicMapper
{
public:
	PathFinderMapper(std::string fName) : HeuristicMapper(fName){
		mapping_method_name  = "PathFinder";
										  };
struct hash_LatPort { 
    size_t operator()(const pair<int, CGRAXMLCompile::Port*>& p) const
    { 
        auto hash1 = hash<int>{}(p.first); 
        auto hash2 = hash<CGRAXMLCompile::Port*>{}(p.second); 
        return hash1 ^ hash2; 
    } 
}; 
	bool Map(CGRA *cgra, DFG *dfg);
	//	bool LeastCostPathAstar(Port* start, Port* end, DataPath* endDP, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool LeastCostPathAstar(LatPort start, LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode);

	bool estimateRouting(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	bool estimateRoutingVec(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);

	int predictiveRoute(DFGNode *node,
						DataPath *dest,
						const std::map<DFGNode *, Port *> routingSourcesIn,
						const std::map<DFGNode *, DataPath *> mappedNodesIn,
						std::map<DFGNode *, Port *> &routingSourcesOut,
						std::map<DFGNode *, DataPath *> &mappedNodesOut);

	bool Route(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
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
	void sortGivenOrder(std::vector<int> nodeOrder);
	bool checkInCameFrom(std::unordered_map<LatPort, LatPort, hash_LatPort> cameFrom, Port * currPort);
	std::ofstream congestionInfoFile;

	void addPseudoEdgesOrphans(DFG *dfg);

	std::vector<DFGNode *> getLongestDFGPath(DFGNode *src, DFGNode *dest);
	int getFreeMEMPeDist(PE *currPE);

	bool canExitCurrPE(LatPort p);
	static bool checkMEMOp(string op);
	void setMaxIter(int m){maxIter = m;}

	bool Check_DFG_CGRA_Compatibility();
	bool checkVectorDP(DataPath* currDP);
	bool checkVectorPort(Port* currPort);
	std::vector<CGRAXMLCompile::DataPath *> selectLeastLat(vector<pair<DataPath *, int> > candDestIn);
        
	void UpdateVariableBaseAddr();
	//void printBinFileIter(const InsFormatIter insFIter,std::map<int,std::map<int,bool>> peEnable, std::map<int,std::map<int,bool>> configOPEnable, std::map<int,std::map<int,std::map<int,bool>>> configRouteEnable,std::map<int,std::map<int,std::map<int,bool>>> configOPEnableCycle,std::string fName);
	void printBinFileIter(const InsFormatIter insFIter,std::map<int,std::map<int,bool>> peEnable, std::map<int,std::map<int,bool>> configOPEnable, std::string fName);
	void printHyCUBEBinary(CGRA* cgra);
	void printBinFile(const std::vector<InsFormat>& insFArr, std::string fName, CGRA* cgra);

	int getIndexOfBin(int t, int y, int x){
		return t * cgra->get_y_max() * cgra->get_x_max() + y * cgra->get_x_max() + x;
	}
	std::tuple<int, int, int> getIndexOfBin(int index);

	void setIterDP(DFGNode* node, DataPath* currDP, int lat);
	std::map<DFGNode *, std::vector<Port *>> selectClosestIterationCounter(std::map<DFGNode *, std::vector<Port *>> possibleStarts, int minLatDestVal,DataPath* dest);
#ifdef FLEX
	void reserveVectorNodes(DataPath* currDP,DFGNode* node,int lat);
	void reserveVectorPorts(Port* currPort,DFGNode* src,int lat,int idx);
	void checkTempReservedVectorPort(Port* currPort);
	void releaseTempReservedVectorPort(Port* currPort);
	void clearReservedPort();
	void restrictVectorPorts(Port* currPort);	
#endif

protected:
	int getlatMinStartsPHI(const DFGNode *currNode, const std::map<DFGNode *, std::vector<Port *>> &possibleStarts);

	std::map<Port *, std::set<DFGNode *>> congestedPorts;
	std::map<Port *, std::set<DFGNode *>> conflictedPorts;
	bool cmp(pair<DataPath *, int>& a,pair<DataPath *, int>& b);


// private:
	
	
	
	int maxIter = 3;

	std::map<int, int> conflictedTimeStepMap;

	std::set<DFGNode *> RecPHIs;
	
	std::set<DFGNode *> getElders(DFGNode *node);

	std::map<BackEdge, std::set<DFGNode *>> RecCycles; // real backedge
	std::map<BackEdge, std::set<DFGNode *>> RecCyclesLS; // the recurrent edge for load store.
	int getMaxLatencyBE(DFGNode *node, std::map<DataPath *, beParentInfo> &beParentDests, int &downStreamOps);
	std::vector<DataPath *> modifyMaxLatCandDest(std::map<DataPath *, int> candDestIn, DFGNode *node, bool &changed);

	void GetAllSupportedOPs(Module* currmod, unordered_set<string>& supp_ops, unordered_set<string>& supp_pointers);
};

} /* namespace CGRAXMLCompile */

#endif /* PATHFINDERMAPPER_H_ */
