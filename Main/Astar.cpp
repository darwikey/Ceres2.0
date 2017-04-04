#include <WProgram.h>
#include "Astar.h"
#include "TrajectoryManager.h"
#include "PositionManager.h"
#include <cmath>

#define OBSTACLE_MARGIN 80.f//120.f
#define TERRAIN_WIDTH	3000.f
#define TERRAIN_HEIGHT	2000.f

class DummyCout {};
class DummyEndl {};

template<typename T> DummyCout& operator<<(DummyCout& cout, T val) {
	Serial.print(val);
	return cout;
}

DummyCout& operator<<(DummyCout& cout, const char* str) {
	Serial.print(str);
	return cout;
}

DummyCout& operator<<(DummyCout& cout, const DummyEndl&) {
	Serial.println();
	return cout;
}

static DummyCout cout;
static DummyEndl endl;

void astar_test(AStarCoord _c)
{
	Graph &g = Graph::Instance;
	g.Init();

	Serial.printf("Test A*: (0,0) to (%d, %d)\r\n", _c.x, _c.y);
	AStar::Path path;

	AStarCoord Start;
	Start.FromWordPosition(PositionManager::Instance.GetXMm(), PositionManager::Instance.GetYMm());
	AStar::Node source(Start);
	AStar::Node dest(_c);

	auto ret = AStar::Instance.FindPath(source, dest, path);

	if (ret == AStar::SUCCESS) cout << "SUCCESS" << endl;
	else if (ret == AStar::ERROR_NOT_FOUND) cout << "ERROR_NOT_FOUND" << endl;
	else if (ret == AStar::ERROR_OUT_OF_MEMORY) cout << "ERROR_OUT_OF_MEMORY" << endl;

	for (const auto& it : path)
		g.PutElement(it._pos, Graph::Value::PATH);

	cout << "Path size: " << path.size() << endl;
	g.Print(false);

	for (const auto& it : path) {
		float x, y;
		it._pos.ToWordPosition(x, y);
		Serial.printf("GotoXY %f,%f\r\n", x, y);
		TrajectoryManager::Instance.GotoXY(x, y);
	}
}

void std::__throw_length_error(char const*) {
	while(1)
		cout << "ERROR : __throw_length_error" << endl;
}

Graph Graph::Instance;
AStar AStar::Instance(Graph::Instance);

void AStarCoord::ToWordPosition(float & _x, float & _y) const
{
	_x = (x + 0.5f) * (TERRAIN_WIDTH / Graph::WIDTH);
	_y = (y + 0.5f) * (TERRAIN_HEIGHT / Graph::HEIGHT);
}

void AStarCoord::FromWordPosition(float _x, float _y)
{
	x = _x * (Graph::WIDTH / TERRAIN_WIDTH);
	y = _y * (Graph::HEIGHT / TERRAIN_HEIGHT);
}

void AStar::Node::SetParent(const Node & parent)
{
	_parent = parent._pos;
	const float xx = _pos.x - _parent.x;
	const float yy = _pos.y - _parent.y;
	const float dist = sqrt(xx*xx + yy*yy);
	/*float angle = 0;
	if (parent._parent.x != -1 && parent._parent.y != -1) {
		const float pxx = parent._pos.x - parent._parent.x;
		const float pyy = parent._pos.y - parent._parent.y;
		const float pdist = sqrt(pxx*pxx + pyy*pyy);
		const float scal = xx*pxx + yy*pyy;
		const float cosa = scal / (dist*pdist);
		angle = fmod(acos(cosa), 3.15 / 2);
	}*/
	_cost = parent._cost + dist;// +(100 * angle);
}

Graph::Graph()
{
	Init();
}

void Graph::Init()
{
	for (int y = 0; y < HEIGHT; y++) {
		for (int x = 0; x < WIDTH; x++) {
			m_Data[x][y].v = Value::EMPTY;
			m_Data[x][y].parent = AStarCoord();
		}
	}
	
	float margin = OBSTACLE_MARGIN;
	// Start zones
	PutObstacleBox(0.f, 0.f, 710.f + margin, 382.f + margin);
	PutObstacleBox(2290.f - margin, 0.f, 3000.f, 382.f + margin);
	// craters close to start zones
	PutObstacleCircle(650.f, 540.f, 100.f + margin);
	PutObstacleCircle(2350.f, 540.f, 100.f + margin);
	// craters in corner
	PutObstacleCircle(0, 2000, 520.f + margin);
	PutObstacleCircle(3000, 2000, 520.f + margin);
	//things on the side
	PutObstacleBox(0.f, 700.f - margin, 80.f + margin, 1150.f + margin);
	PutObstacleBox(2920.f - margin, 700.f - margin, 3000.f, 1150.f + margin);
	// central construction zone
	PutObstacleBox(1500 - 68 - margin, 1200.f - margin, 1500.f + 68 + margin, 2000.f);

	{
		int l = 800.f / (TERRAIN_WIDTH / Graph::WIDTH);
		AStarCoord c0;
		c0.FromWordPosition(1500.f, 2000.f);
		AStarCoord c1=c0, c2=c0;
		for (int  i = 0; i < l; i++)
		{
			c1.x--; c1.y--;
			c2.x++; c2.y--;

			int l2 = (68.f + margin) / (TERRAIN_WIDTH / Graph::WIDTH);
			for (int j = -l2; j <= l2; j++)
			{
				AStarCoord c3 = c1, c4 = c2;
				c3.x -= j;
				c3.y += j;
				c4.x += j;
				c4.y += j;
				PutElement(c3, Value::OBSTACLE);
				PutElement(c3.x, c3.y+1, Value::OBSTACLE);
				PutElement(c4.x-1, c4.y, Value::OBSTACLE);
				PutElement(c4, Value::OBSTACLE);
			}
		}
	}
}

Graph::InternalNode & Graph::operator[](const AStarCoord & c)
{
	if ((unsigned)c.x >= WIDTH || (unsigned)c.y >= HEIGHT) {
		while (1)
			cout << "ERROR : Graph::operator []" << endl;
	}
	return m_Data[c.x][c.y];
}

const Graph::InternalNode & Graph::operator[](const AStarCoord & c) const
{
	if ((unsigned)c.x >= WIDTH || (unsigned)c.y >= HEIGHT) {
		while (1)
			cout << "ERROR : Graph::operator [] const" << endl;
	}
	return m_Data[c.x][c.y];
}

void Graph::Print(bool _debug) const
{
	Serial.printf("size %d\r\n", sizeof(m_Data));
	for (int y = 0; y < HEIGHT; y++) {
		for (int x = 0; x < WIDTH; x++) {
			auto v = m_Data[x][y].v;
			if (v == Value::OBSTACLE)
				cout << "#";
			else if (v == Value::PATH)
				cout << "X";
			else if(_debug)
			{
				if (v == Value::CLOSED)
					cout << "c";
				else if (v == Value::OPEN)
					cout << "o";
				else
					cout << ".";
			}
			else
				cout << ".";
		}
		cout << endl;
	}
}

bool Graph::IsNode(const AStarCoord & c) const
{
	if ((unsigned)c.x >= WIDTH) return false;
	if ((unsigned)c.y >= HEIGHT) return false;
	return m_Data[c.x][c.y].v != Value::OBSTACLE;
}

std::vector<AStar::Node> Graph::GetNeighbors(const AStar::Node & node) const
{
	std::vector<AStar::Node> ret;
	AStarCoord c;
	for (c.y = node._pos.y - 1; c.y <= node._pos.y + 1; c.y++) {
		for (c.x = node._pos.x - 1; c.x <= node._pos.x + 1; c.x++) {
			if (IsNode(c)) {
				AStar::Node n(c);
				if (!(node == n)) {
					n.SetParent(node);
					ret.push_back(n);
				}
			}
		}
	}
	//cout << endl;
	//print();
	return ret;
}

std::vector<AStar::Node> Graph::GetParents(const AStar::Node & node) const
{
	std::vector<AStar::Node> ret;
	AStarCoord cur = node._pos;
	while (cur != AStarCoord()) {
		ret.push_back(AStar::Node(cur));
		cur = (*this)[cur].parent;
	}
	return ret;
}

void Graph::PutObstacleBox(int x0, int y0, int x1, int y1)
{
	for (int x = x0; x <= x1; x++)
		for (int y = y0; y <= y1; y++)
			PutElement(x, y, Value::OBSTACLE);
}

void Graph::PutObstacleBox(float x0, float y0, float x1, float y1)
{
	AStarCoord c0, c1;
	c0.FromWordPosition(x0, y0);
	c1.FromWordPosition(x1, y1);
	PutObstacleBox(c0.x, c0.y, c1.x, c1.y);
}

void Graph::PutObstacleCircle(float x, float y, float radius)
{
	AStarCoord c0, c1, it;
	c0.FromWordPosition(x - radius, y - radius);
	c1.FromWordPosition(x + radius, y + radius);
	radius *= radius;
	for (it.x = c0.x; it.x <= c1.x; it.x++)
	{
		for (it.y = c0.y; it.y <= c1.y; it.y++)
		{
			float x2, y2;
			it.ToWordPosition(x2, y2);
			if ((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y) <= radius)
				PutElement(it.x, it.y, Value::OBSTACLE);
		}
	}
}

struct ClosedList {
	std::vector<AStar::Node> m_Data;
	Graph& m_Graph;

	ClosedList(Graph& graph)
		: m_Graph(graph) {

	}

	void insert(AStar::Node e) {
		for (auto it = m_Data.begin(); it != m_Data.end(); ++it) {
			if (*it == e) {
				m_Graph.PutElement(e._pos, Graph::Value::CLOSED);
				*it = e;
				return;
			}
		}
		m_Graph.PutElement(e._pos, Graph::Value::CLOSED);
		m_Data.push_back(e);
	}

	std::vector<AStar::Node>::const_iterator begin(void) const {
		return m_Data.begin();
	}

	std::vector<AStar::Node>::const_iterator end(void) const {
		return m_Data.end();
	}

	bool isEmpty(void) const {
		return m_Data.size() == 0;
	}

	bool IsFull(void) const {
		return false;
	}

	void Flush(void) {
		m_Data.resize(0);
	}

};

struct OpenList {
	std::vector<AStar::Node> m_Data;
	Graph& m_Graph;
	AStar::Node _dest;

	OpenList(Graph& graph, AStar::Node dest)
		: m_Graph(graph), _dest(dest) {
	}

	void insert(AStar::Node e) {
		for (auto it = m_Data.begin(); it != m_Data.end(); ++it) {
			if (*it == e) {
				m_Graph.PutElement(e._pos, Graph::Value::OPEN);
				*it = e;
				return;
			}
		}
		m_Graph.PutElement(e._pos, Graph::Value::OPEN);
		m_Data.push_back(e);
	}

	void pop(void) {
		auto rm = m_Data.begin();
		for (auto it = m_Data.begin(); it != m_Data.end(); ++it) {
			if (heuristic(*it) < heuristic(*rm)) {
				rm = it;
			}
		}
		m_Graph.PutElement(rm->_pos, Graph::Value::EMPTY);
		m_Data.erase(rm);
	}

	int heuristic(const AStar::Node& n) const {
		return n.cost() + m_Graph.GetDistance(n, _dest);
	}

	AStar::Node head(void) const {
		auto min = m_Data.begin();
		for (auto it = m_Data.begin(); it != m_Data.end(); ++it) {
			if (heuristic(*it) < heuristic(*min)) {
				min = it;
			}
		}
		return *min;
	}

	std::vector<AStar::Node>::const_iterator begin(void) const {
		return m_Data.begin();
	}

	std::vector<AStar::Node>::const_iterator end(void) const {
		return m_Data.end();
	}

	bool isEmpty(void) const {
		return m_Data.size() == 0;
	}

	bool IsFull(void) const {
		return false;
	}

	void Flush(void) {
		m_Data.resize(0);
	}
};

template<class List>
bool  AStar::TryInsert(List& list, const Node& node)
{
	bool ret = !list.IsFull();
	if (ret) {
		list.insert(node);
	}
	return ret;
}

bool AStar::FindBetter(const ClosedList & closed, const OpenList & open, const Node & node)
{
	for (auto it = closed.begin(); it != closed.end(); ++it) {
		if ((*it) == node) {
			return m_Graph.GetCost(*it) <= m_Graph.GetCost(node);
		}
	}

	for (auto it = open.begin(); it != open.end(); ++it) {
		if ((*it) == node) {
			return m_Graph.GetCost(*it) <= m_Graph.GetCost(node);
		}
	}

	return false;
}

AStar::ReturnStatus AStar::BuildPath(const Node & last, Path & out_path)
{
	auto parents = m_Graph.GetParents(last);
	for (auto it = parents.begin(); it != parents.end(); ++it) {
		const Node& n = *it;

		if (out_path.IsFull()) {
			return ReturnStatus::ERROR_OUT_OF_MEMORY;
		}
		out_path.Push(n);
	}

	return ReturnStatus::SUCCESS;
}

AStar::ReturnStatus AStar::FindPath(const Node & source, const Node & destination, Path & out_path)
{
	OpenList open(m_Graph, destination);
	ClosedList closed(m_Graph);
	out_path.Flush();

	if (!TryInsert(open, source)) {
		return ReturnStatus::ERROR_OUT_OF_MEMORY;
	}

	while (!open.isEmpty()) {
		const Node& current = open.head();
		open.pop();

		if (!TryInsert(closed, current)) {
			return ReturnStatus::ERROR_OUT_OF_MEMORY;
		}

		if (current == destination) {
			return BuildPath(current, out_path);
		}

		auto neighbors = m_Graph.GetNeighbors(current);
		for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
			Node& neighbor = *it;

			if (!FindBetter(closed, open, neighbor)) {
				m_Graph.SetParent(neighbor, current);
				if (!TryInsert(open, neighbor)) {
					return ReturnStatus::ERROR_OUT_OF_MEMORY;
				}
			}
		}
	}

	return ReturnStatus::ERROR_NOT_FOUND;
}
