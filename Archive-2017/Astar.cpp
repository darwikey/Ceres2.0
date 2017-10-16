#ifdef ENABLE_ASTAR
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

	AStarCoord Start;
	Start.FromWordPosition(PositionManager::Instance.GetPosMm());
	Serial.printf("Test A*: (%d,%d) to (%d, %d)\r\n", Start.x, Start.y, _c.x, _c.y);
	AStar::Node source(Start);
	AStar::Node dest(_c);

	AStar::Path path;
	auto ret = AStar::Instance.FindPath(source, dest, path);

	if (ret == AStar::SUCCESS) cout << "SUCCESS" << endl;
	else if (ret == AStar::ERROR_NOT_FOUND) cout << "ERROR_NOT_FOUND" << endl;
	else if (ret == AStar::ERROR_OUT_OF_MEMORY) cout << "ERROR_OUT_OF_MEMORY" << endl;

	for (const auto& it : path)
	{
		g.PutElement(it._pos, Graph::Value::PATH);
	}

	cout << "Path size: " << path.Size() << endl;
	g.Print(false);

	for (const auto& it : path) {
		Float2 p;
		it._pos.ToWordPosition(p);
		Serial.printf("GotoXY %f,%f\r\n", p.x, p.y);
		TrajectoryManager::Instance.GotoXY(p);
	}
}

//void std::__throw_length_error(char const*) {
//	while(1)
//		cout << "ERROR : __throw_length_error" << endl;
//}

Graph Graph::Instance;
AStar AStar::Instance(Graph::Instance);

void AStarCoord::ToWordPosition(Float2 & _pos) const
{
	_pos.x = (x + 0.5f) * (TERRAIN_WIDTH / Graph::WIDTH);
	_pos.y = (y + 0.5f) * (TERRAIN_HEIGHT / Graph::HEIGHT);
}

void AStarCoord::FromWordPosition(const Float2 &_pos)
{
	x = _pos.x * (Graph::WIDTH / TERRAIN_WIDTH);
	y = _pos.y * (Graph::HEIGHT / TERRAIN_HEIGHT);
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
	PutObstacleBox(Float2(0.f, 0.f), Float2(710.f + margin, 382.f + margin));
	PutObstacleBox(Float2(2290.f - margin, 0.f), Float2(3000.f, 382.f + margin));
	// craters close to start zones
	PutObstacleCircle(Float2(650.f, 540.f), 100.f + margin);
	PutObstacleCircle(Float2(2350.f, 540.f), 100.f + margin);
	// craters in corner
	PutObstacleCircle(Float2(0, 2000), 520.f + margin);
	PutObstacleCircle(Float2(3000, 2000), 520.f + margin);
	//things on the side
	PutObstacleBox(Float2(0.f, 700.f - margin), Float2(80.f + margin, 1150.f + margin));
	PutObstacleBox(Float2(2920.f - margin, 700.f - margin), Float2(3000.f, 1150.f + margin));
	// central construction zone
	PutObstacleBox(Float2(1500 - 68 - margin, 1200.f - margin), Float2(1500.f + 68 + margin, 2000.f));

	{
		int l = 800.f / (TERRAIN_WIDTH / Graph::WIDTH);
		AStarCoord c0;
		c0.FromWordPosition(Float2(1500.f, 2000.f));
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

void Graph::GetNeighbors(const AStar::Node & node, Vector<AStar::Node> &neightbors) const
{
	AStarCoord c;
	for (c.y = node._pos.y - 1; c.y <= node._pos.y + 1; c.y++) {
		for (c.x = node._pos.x - 1; c.x <= node._pos.x + 1; c.x++) {
			if (IsNode(c)) {
				AStar::Node n(c);
				if (!(node == n)) {
					n.SetParent(node);
					neightbors.Push(n);
				}
			}
		}
	}
	//cout << endl;
	//print();
}

void Graph::GetParents(const AStar::Node & node, Vector<AStar::Node> &parents) const
{
	AStarCoord cur = node._pos;
	while (cur != AStarCoord()) {
		parents.Push(AStar::Node(cur));
		cur = (*this)[cur].parent;
	}
}

void Graph::PutObstacleBox(int x0, int y0, int x1, int y1)
{
	for (int x = x0; x <= x1; x++)
		for (int y = y0; y <= y1; y++)
			PutElement(x, y, Value::OBSTACLE);
}

void Graph::PutObstacleBox(const Float2 &p0, const Float2 &p1)
{
	AStarCoord c0, c1;
	c0.FromWordPosition(p0);
	c1.FromWordPosition(p1);
	PutObstacleBox(c0.x, c0.y, c1.x, c1.y);
}

void Graph::PutObstacleCircle(const Float2 &center, float radius)
{
	AStarCoord c0, c1, it;
	c0.FromWordPosition(Float2(center.x - radius, center.y - radius));
	c1.FromWordPosition(Float2(center.x + radius, center.y + radius));
	radius *= radius;
	for (it.x = c0.x; it.x <= c1.x; it.x++)
	{
		for (it.y = c0.y; it.y <= c1.y; it.y++)
		{
			Float2 p;
			it.ToWordPosition(p);
			if ((p - center).LengthSquared() <= radius)
				PutElement(it.x, it.y, Value::OBSTACLE);
		}
	}
}

struct ClosedList {
	Vector<AStar::Node> m_Data;
	Graph& m_Graph;

	ClosedList(Graph& graph)
		: m_Graph(graph) {

	}

	void insert(AStar::Node e) {
		for (auto &it : m_Data) {
			if (it == e) {
				m_Graph.PutElement(e._pos, Graph::Value::CLOSED);
				it = e;
				return;
			}
		}
		m_Graph.PutElement(e._pos, Graph::Value::CLOSED);
		//Serial.printf("c %d\r\n", m_Data.Size());
		m_Data.Push(e);
	}

	Vector<AStar::Node>::ConstIterator begin(void) const {
		return m_Data.Begin();
	}

	Vector<AStar::Node>::ConstIterator end(void) const {
		return m_Data.End();
	}

	bool isEmpty(void) const {
		return m_Data.Size() == 0;
	}

	bool IsFull(void) const {
		return false;
	}

	void Flush(void) {
		m_Data.Clear();
	}

};

struct OpenList {
	Vector<AStar::Node> m_Data;
	Graph& m_Graph;
	AStar::Node _dest;

	OpenList(Graph& graph, AStar::Node dest)
		: m_Graph(graph), _dest(dest) {
	}

	void insert(AStar::Node e) {
		for (auto &it : m_Data) {
			if (it == e) {
				m_Graph.PutElement(e._pos, Graph::Value::OPEN);
				it = e;
				return;
			}
		}
		m_Graph.PutElement(e._pos, Graph::Value::OPEN);
		//Serial.printf("o %d\r\n", m_Data.Size());
		m_Data.Push(e);
	}

	void pop(void) {
		auto rm = m_Data.Begin();
		for (auto it = m_Data.Begin(); it != m_Data.End(); ++it) {
			if (heuristic(*it) < heuristic(*rm)) {
				rm = it;
			}
		}
		m_Graph.PutElement(rm->_pos, Graph::Value::EMPTY);
		m_Data.Erase(rm);
	}

	int heuristic(const AStar::Node& n) const {
		return n.cost() + m_Graph.GetDistance(n, _dest);
	}

	AStar::Node head(void) const {
		auto min = m_Data.Begin();
		for (auto it = m_Data.Begin(); it != m_Data.End(); ++it) {
			if (heuristic(*it) < heuristic(*min)) {
				min = it;
			}
		}
		return *min;
	}

	Vector<AStar::Node>::ConstIterator begin(void) const {
		return m_Data.Begin();
	}

	Vector<AStar::Node>::ConstIterator end(void) const {
		return m_Data.End();
	}

	bool isEmpty(void) const {
		return m_Data.Size() == 0;
	}

	bool IsFull(void) const {
		return false;
	}

	void Flush(void) {
		m_Data.Clear();
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
	Vector<AStar::Node> parents;
	m_Graph.GetParents(last, parents);
	for (auto it = parents.Begin(); it != parents.End(); ++it) {
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
	//Serial.println("open");
	ClosedList closed(m_Graph);
	out_path.Flush();

	if (!TryInsert(open, source)) {
		return ReturnStatus::ERROR_OUT_OF_MEMORY;
	}

	while (!open.isEmpty()) {
		const Node current = open.head();
		open.pop();

		if (!TryInsert(closed, current)) {
			return ReturnStatus::ERROR_OUT_OF_MEMORY;
		}

		if (current == destination) {
			Serial.println("ok");
			return BuildPath(current, out_path);
		}

		Vector<AStar::Node> neighbors;
		m_Graph.GetNeighbors(current, neighbors);
		//Serial.printf("get neightboor %d\r\n", neighbors.Size());
		for (Node& neighbor : neighbors) {
			//Serial.println("find better");
			if (!FindBetter(closed, open, neighbor)) {
				//Serial.println("set parent");
				m_Graph.SetParent(neighbor, current);
				if (!TryInsert(open, neighbor)) {
					return ReturnStatus::ERROR_OUT_OF_MEMORY;
				}
			}
		}
	}

	return ReturnStatus::ERROR_NOT_FOUND;
}
#endif