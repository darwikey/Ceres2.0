#include <WProgram.h>
#include "Astar.h"
#include <cmath>

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

void std::__throw_length_error(char const*) {
	while(1)
		cout << "ERROR : __throw_length_error" << endl;
}

Graph Graph::Instance;
AStar AStar::Instance(Graph::Instance);

void AStar::Node::SetParent(const Node & parent)
{
	_parent = parent._pos;
	const double xx = _pos.x - _parent.x;
	const double yy = _pos.y - _parent.y;
	const double dist = sqrt(xx*xx + yy*yy);
	double angle = 0;
	if (parent._parent.x != -1 && parent._parent.y != -1) {
		const double pxx = parent._pos.x - parent._parent.x;
		const double pyy = parent._pos.y - parent._parent.y;
		const double pdist = sqrt(pxx*pxx + pyy*pyy);
		const double scal = xx*pxx + yy*pyy;
		const double cosa = scal / (dist*pdist);
		angle = fmod(acos(cosa), 3.15 / 2);
	}
	_cost = parent._cost + dist + (100 * angle);
}

void astar_test(int x, int y)
{
	Serial.printf("Test A*: (0,0) to (%d, %d)\r\n",x,y);
	AStar::Path path;

	AStar::Node source(AStarCoord(0, 0));
	AStar::Node dest(AStarCoord(x, y));

	Graph &g = Graph::Instance;
	g.PutObstacle(5, 5);
	g.PutObstacle(4, 4);
	g.PutObstacle(5, 4);
	g.PutObstacle(4, 5);

	g.PutObstacle(5, 3);
	g.PutObstacle(4, 3);

	g.PutObstacle(3, 4);
	g.PutObstacle(3, 5);
	g.PutObstacle(2, 4);
	g.PutObstacle(2, 5);

	auto ret = AStar::Instance.FindPath(source, dest, path);

	if (ret == AStar::SUCCESS) cout << "SUCCESS" << endl;
	else if (ret == AStar::ERROR_NOT_FOUND) cout << "ERROR_NOT_FOUND" << endl;
	else if (ret == AStar::ERROR_OUT_OF_MEMORY) cout << "ERROR_OUT_OF_MEMORY" << endl;

	cout << path.size() << endl;

	for (auto it = path.begin(); it != path.end(); ++it) {
		g.PutElement(it->_pos, Graph::Value::PATH);
	}

	cout << endl;
	g.Print(false);
}

Graph::Graph()
{
	for (int y = 0; y < _height; y++) {
		for (int x = 0; x < _width; x++) {
			m_Data[x][y].v = Value::EMPTY;
		}
	}
}

Graph::InternalNode & Graph::operator[](const AStarCoord & c)
{
	if ((unsigned)c.x >= _width || (unsigned)c.y >= _height) {
		while (1)
			cout << "ERROR : Graph::operator []" << endl;
	}
	return m_Data[c.x][c.y];
}

const Graph::InternalNode & Graph::operator[](const AStarCoord & c) const
{
	if ((unsigned)c.x >= _width || (unsigned)c.y >= _height) {
		while (1)
			cout << "ERROR : Graph::operator [] const" << endl;
	}
	return m_Data[c.x][c.y];
}

void Graph::Print(bool _debug) const
{
	for (int y = 0; y < _height; y++) {
		for (int x = 0; x < _width; x++) {
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
	if ((unsigned)c.x >= _width) return false;
	if ((unsigned)c.y >= _height) return false;
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

void Graph::PutObstacle(unsigned x0, unsigned y0, unsigned x1, unsigned y1)
{
	for (unsigned x = x0; x <= x1; x++) 
		for (unsigned y = y0; y <= y1; y++)
			PutElement(x, y, Value::OBSTACLE);
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
