"""
A* Pathfinding Algorithm Implementation
Author: [Your Name]
Course: CSE 511 Advanced Algorithms
Date: December 11, 2025

This implementation finds the shortest path in a weighted graph using the A* algorithm.
"""

import heapq
from typing import Dict, List, Tuple, Optional, Set
import math


class AStarPathfinder:
    """
    A* Pathfinding Algorithm implementation for weighted graphs.
    
    Attributes:
        graph: Dictionary representation of the graph with edge weights
        heuristic: Dictionary of heuristic values (estimated cost to goal)
    """
    
    def __init__(self, graph: Dict[str, Dict[str, float]], heuristic: Dict[str, float]):
        """
        Initialize the A* pathfinder.
        
        Args:
            graph: Graph as adjacency list with weights
                   Format: {node: {neighbor: weight, ...}, ...}
            heuristic: Heuristic function values for each node
                      Format: {node: h_value, ...}
        """
        self.graph = graph
        self.heuristic = heuristic
        self.nodes_explored = 0
        self.path_cost = 0
    
    def find_path(self, start: str, goal: str) -> Tuple[Optional[List[str]], float, int]:
        """
        Find the shortest path from start to goal using A* algorithm.
        
        Args:
            start: Starting node
            goal: Goal/destination node
            
        Returns:
            Tuple of (path, total_cost, nodes_explored)
            - path: List of nodes in the shortest path, or None if no path exists
            - total_cost: Total cost of the path
            - nodes_explored: Number of nodes explored during search
        """
        # Reset counters
        self.nodes_explored = 0
        self.path_cost = 0
        
        # Priority queue: (f_score, node)
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic.get(start, 0), start))
        
        # Track path
        came_from = {}
        
        # Cost from start to node
        g_score = {start: 0}
        
        # Estimated total cost (f = g + h)
        f_score = {start: self.heuristic.get(start, 0)}
        
        # Track visited nodes
        visited = set()
        
        while open_set:
            # Get node with lowest f_score
            current_f, current = heapq.heappop(open_set)
            
            # Skip if already visited
            if current in visited:
                continue
            
            visited.add(current)
            self.nodes_explored += 1
            
            # Goal reached
            if current == goal:
                path = self._reconstruct_path(came_from, current)
                self.path_cost = g_score[current]
                return path, self.path_cost, self.nodes_explored
            
            # Explore neighbors
            if current in self.graph:
                for neighbor, weight in self.graph[current].items():
                    if neighbor in visited:
                        continue
                    
                    # Calculate tentative g_score
                    tentative_g = g_score[current] + weight
                    
                    # Update if better path found
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic.get(neighbor, 0)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None, float('inf'), self.nodes_explored
    
    def _reconstruct_path(self, came_from: Dict[str, str], current: str) -> List[str]:
        """
        Reconstruct the path from start to goal.
        
        Args:
            came_from: Dictionary mapping each node to its parent
            current: Current (goal) node
            
        Returns:
            List of nodes representing the path from start to goal
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.insert(0, current)
        return path
    
    def print_path_info(self, path: Optional[List[str]], cost: float, nodes_explored: int):
        """
        Print formatted information about the found path.
        
        Args:
            path: The path found (or None)
            cost: Total cost of the path
            nodes_explored: Number of nodes explored
        """
        print("\n" + "="*60)
        print("A* PATHFINDING RESULTS")
        print("="*60)
        
        if path is None:
            print("❌ No path found!")
        else:
            print(f"✓ Path found: {' → '.join(path)}")
            print(f"✓ Path length: {len(path)} nodes")
            print(f"✓ Total cost: {cost:.2f}")
        
        print(f"✓ Nodes explored: {nodes_explored}")
        
        if path and len(self.graph) > 0:
            efficiency = (len(path) / nodes_explored) * 100
            print(f"✓ Search efficiency: {efficiency:.2f}%")
        
        print("="*60 + "\n")


def create_sample_graph_1() -> Tuple[Dict, Dict]:
    """
    Create a sample graph for testing (simple network).
    
    Returns:
        Tuple of (graph, heuristic)
    """
    graph = {
        'A': {'B': 2, 'C': 3},
        'B': {'A': 2, 'D': 5, 'E': 1},
        'C': {'A': 3, 'F': 8},
        'D': {'B': 5, 'G': 2},
        'E': {'B': 1, 'G': 4},
        'F': {'C': 8, 'G': 3},
        'G': {'D': 2, 'E': 4, 'F': 3}
    }
    
    # Heuristic: estimated distance to goal (G)
    heuristic = {
        'A': 7,
        'B': 6,
        'C': 5,
        'D': 2,
        'E': 4,
        'F': 3,
        'G': 0
    }
    
    return graph, heuristic


def create_sample_graph_2() -> Tuple[Dict, Dict]:
    """
    Create a more complex sample graph (city navigation).
    
    Returns:
        Tuple of (graph, heuristic)
    """
    graph = {
        'Start': {'A': 1, 'B': 4},
        'A': {'Start': 1, 'C': 2, 'B': 2},
        'B': {'Start': 4, 'A': 2, 'D': 5},
        'C': {'A': 2, 'E': 3, 'F': 6},
        'D': {'B': 5, 'F': 2, 'Goal': 8},
        'E': {'C': 3, 'Goal': 4},
        'F': {'C': 6, 'D': 2, 'Goal': 3},
        'Goal': {'D': 8, 'E': 4, 'F': 3}
    }
    
    # Heuristic: straight-line distance to Goal
    heuristic = {
        'Start': 10,
        'A': 9,
        'B': 8,
        'C': 7,
        'D': 6,
        'E': 4,
        'F': 3,
        'Goal': 0
    }
    
    return graph, heuristic


def create_sample_graph_3() -> Tuple[Dict, Dict]:
    """
    Create a grid-based graph (representing a map).
    
    Returns:
        Tuple of (graph, heuristic)
    """
    graph = {
        '0,0': {'0,1': 1, '1,0': 1},
        '0,1': {'0,0': 1, '0,2': 1, '1,1': 1},
        '0,2': {'0,1': 1, '1,2': 1},
        '1,0': {'0,0': 1, '1,1': 1, '2,0': 1},
        '1,1': {'0,1': 1, '1,0': 1, '1,2': 1, '2,1': 1},
        '1,2': {'0,2': 1, '1,1': 1, '2,2': 1},
        '2,0': {'1,0': 1, '2,1': 1},
        '2,1': {'2,0': 1, '1,1': 1, '2,2': 1},
        '2,2': {'2,1': 1, '1,2': 1}
    }
    
    # Heuristic: Manhattan distance to (2,2)
    heuristic = {
        '0,0': 4, '0,1': 3, '0,2': 2,
        '1,0': 3, '1,1': 2, '1,2': 1,
        '2,0': 2, '2,1': 1, '2,2': 0
    }
    
    return graph, heuristic


def main():
    """
    Main function to demonstrate A* pathfinding algorithm.
    """
    print("\n" + "="*60)
    print("A* PATHFINDING ALGORITHM DEMONSTRATION")
    print("="*60)
    
    # Example 1: Simple Network
    print("\n📍 EXAMPLE 1: Simple Network Graph")
    print("-" * 60)
    graph1, heuristic1 = create_sample_graph_1()
    pathfinder1 = AStarPathfinder(graph1, heuristic1)
    
    start1, goal1 = 'A', 'G'
    print(f"Finding path from '{start1}' to '{goal1}'...")
    path1, cost1, explored1 = pathfinder1.find_path(start1, goal1)
    pathfinder1.print_path_info(path1, cost1, explored1)
    
    # Example 2: City Navigation
    print("\n📍 EXAMPLE 2: City Navigation Graph")
    print("-" * 60)
    graph2, heuristic2 = create_sample_graph_2()
    pathfinder2 = AStarPathfinder(graph2, heuristic2)
    
    start2, goal2 = 'Start', 'Goal'
    print(f"Finding path from '{start2}' to '{goal2}'...")
    path2, cost2, explored2 = pathfinder2.find_path(start2, goal2)
    pathfinder2.print_path_info(path2, cost2, explored2)
    
    # Example 3: Grid-based Map
    print("\n📍 EXAMPLE 3: Grid-Based Map (3x3)")
    print("-" * 60)
    graph3, heuristic3 = create_sample_graph_3()
    pathfinder3 = AStarPathfinder(graph3, heuristic3)
    
    start3, goal3 = '0,0', '2,2'
    print(f"Finding path from '{start3}' to '{goal3}'...")
    path3, cost3, explored3 = pathfinder3.find_path(start3, goal3)
    pathfinder3.print_path_info(path3, cost3, explored3)
    
    # Custom graph example
    print("\n📍 EXAMPLE 4: Custom Graph (User Input)")
    print("-" * 60)
    print("Define your own graph:")
    print("Format: {node: {neighbor: weight, ...}, ...}")
    print("\nExample custom graph:")
    
    custom_graph = {
        'S': {'A': 3, 'B': 5},
        'A': {'S': 3, 'C': 2, 'D': 4},
        'B': {'S': 5, 'D': 2},
        'C': {'A': 2, 'T': 6},
        'D': {'A': 4, 'B': 2, 'T': 3},
        'T': {'C': 6, 'D': 3}
    }
    
    custom_heuristic = {
        'S': 7,
        'A': 5,
        'B': 4,
        'C': 3,
        'D': 2,
        'T': 0
    }
    
    print(f"Graph: {custom_graph}")
    print(f"Heuristic: {custom_heuristic}")
    
    pathfinder4 = AStarPathfinder(custom_graph, custom_heuristic)
    start4, goal4 = 'S', 'T'
    print(f"\nFinding path from '{start4}' to '{goal4}'...")
    path4, cost4, explored4 = pathfinder4.find_path(start4, goal4)
    pathfinder4.print_path_info(path4, cost4, explored4)
    
    print("\n" + "="*60)
    print("ALGORITHM COMPLEXITY ANALYSIS")
    print("="*60)
    print("Time Complexity:  O(b^d) where b=branching factor, d=depth")
    print("Space Complexity: O(b^d) - stores all nodes in memory")
    print("Optimality:       Guaranteed with admissible heuristic")
    print("Completeness:     Yes, if solution exists")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()


"""
USAGE INSTRUCTIONS:
==================

1. Basic Usage:
   - Run the script to see example demonstrations
   - Modify the graph dictionaries to test your own graphs

2. Graph Format:
   graph = {
       'node1': {'neighbor1': weight, 'neighbor2': weight},
       'node2': {'neighbor1': weight},
       ...
   }

3. Heuristic Format:
   heuristic = {
       'node1': estimated_cost_to_goal,
       'node2': estimated_cost_to_goal,
       ...
   }

4. Creating Custom Graphs:
   # Define your graph
   my_graph = {
       'A': {'B': 5, 'C': 3},
       'B': {'A': 5, 'D': 2},
       'C': {'A': 3, 'D': 4},
       'D': {'B': 2, 'C': 4}
   }
   
   # Define heuristic (must be admissible for optimal results)
   my_heuristic = {
       'A': 6,
       'B': 3,
       'C': 4,
       'D': 0  # Goal node has heuristic of 0
   }
   
   # Create pathfinder and find path
   pathfinder = AStarPathfinder(my_graph, my_heuristic)
   path, cost, explored = pathfinder.find_path('A', 'D')
   pathfinder.print_path_info(path, cost, explored)

5. Requirements:
   - Python 3.6 or higher
   - No external libraries required (uses only standard library)

6. Output Information:
   - Path: Sequence of nodes from start to goal
   - Path length: Number of nodes in the path
   - Total cost: Sum of edge weights in the path
   - Nodes explored: Number of nodes visited during search
   - Search efficiency: Percentage showing how direct the search was
"""