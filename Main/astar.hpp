#ifndef _ASTAR_H_
#define _ASTAR_H_

void astar_test(int x, int y);

struct AStarCoord
{
	AStarCoord() : x(-1), y(-1) {}
	AStarCoord(uint16_t _x, uint16_t _y) : x(_x), y(_y) {}
	bool operator==(const AStarCoord& other) const {
		return x == other.x && y == other.y;
	}
	bool operator!=(const AStarCoord& other) const {
		return x != other.x || y != other.y;
	}
	int16_t x;
	int16_t y;
};

template<class Graph, class Node, class ClosedList, class OpenList>
struct AStar {
private:
	bool findBetter(const ClosedList& closed, const OpenList& open, const Node& node) {
		for (auto it = closed.begin(); it != closed.end(); ++it) {
			if ((*it) == node) {
				return _graph.getCost(*it) <= _graph.getCost(node);
			}
		}

		for (auto it = open.begin(); it != open.end(); ++it) {
			if ((*it) == node) {
				return _graph.getCost(*it) <= _graph.getCost(node);
			}
		}

		return false;
	}

	template<class List>
	bool tryInsert(List& list, const Node& node) {
		bool ret = !list.isFull();
		if (ret) {
			list.insert(node);
		}
		return ret;
	}

private:
	Graph& _graph;

public:
	AStar(Graph& g)
		: _graph(g) {
	}

	enum ReturnStatus {
		SUCCESS,
		ERROR_NOT_FOUND,
		ERROR_OUT_OF_MEMORY
	};

	template<typename Path>
	ReturnStatus buildPath(const Node& last, Path& out_path) {
		auto parents = _graph.getParents(last);
		for (auto it = parents.begin(); it != parents.end(); ++it) {
			const Node& n = *it;

			if (out_path.isFull()) {
				return ReturnStatus::ERROR_OUT_OF_MEMORY;
			}
			out_path.push(n);
		}

		return ReturnStatus::SUCCESS;
	}

	template<typename Path>
	ReturnStatus findPath(const Node& source, const Node& destination, Path& out_path) {
		OpenList open(_graph, destination);
		ClosedList closed(_graph);
		out_path.flush();

		if (!tryInsert(open, source)) {
			return ReturnStatus::ERROR_OUT_OF_MEMORY;
		}

		while (!open.isEmpty()) {
			const Node& current = open.head();
			open.pop();

			if (!tryInsert(closed, current)) {
				return ReturnStatus::ERROR_OUT_OF_MEMORY;
			}

			if (current == destination) {
				return buildPath(current, out_path);
			}

			auto neighbors = _graph.getNeighbors(current);
			for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
				Node& neighbor = *it;

				if (!findBetter(closed, open, neighbor)) {
					_graph.setParent(neighbor, current);
					if (!tryInsert(open, neighbor)) {
						return ReturnStatus::ERROR_OUT_OF_MEMORY;
					}
				}
			}
		}

		return ReturnStatus::ERROR_NOT_FOUND;
	}
};

#endif