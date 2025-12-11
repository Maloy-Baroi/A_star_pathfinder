"""
A* Pathfinding Algorithm - Visual Implementation with Tkinter
Author: [Your Name]
Course: CSE 511 Advanced Algorithms
Date: December 11, 2025

This implementation provides a visual interface for A* pathfinding on graphs.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import heapq
import math
import time
from typing import Dict, List, Tuple, Optional, Set


class AStarVisualizer:
    """
    Visual A* Pathfinding implementation using Tkinter.
    """
    
    def __init__(self, root):
        self.root = root
        self.root.title("A* Pathfinding Algorithm Visualizer")
        self.root.geometry("1400x900")
        self.root.configure(bg='#f0f0f0')
        
        # Graph data
        self.graph = {}
        self.heuristic = {}
        self.node_positions = {}
        
        # Visual elements
        self.canvas_items = {}
        self.node_circles = {}
        self.edge_lines = {}
        
        # Algorithm state
        self.start_node = None
        self.goal_node = None
        self.current_path = []
        self.visited_nodes = set()
        self.is_running = False
        
        # Statistics
        self.nodes_explored = 0
        self.path_cost = 0
        self.execution_time = 0
        
        # Colors
        self.colors = {
            'node': '#e3f2fd',
            'start': '#4caf50',
            'goal': '#f44336',
            'visited': '#90caf9',
            'path': '#ffd54f',
            'edge': '#757575',
            'text': '#212121'
        }
        
        self.setup_ui()
        self.load_sample_graph()
    
    def setup_ui(self):
        """Create the user interface."""
        # Title
        title_frame = tk.Frame(self.root, bg='#1976d2', pady=15)
        title_frame.pack(fill=tk.X)
        
        title_label = tk.Label(
            title_frame,
            text="🔍 A* Pathfinding Algorithm Visualizer",
            font=('Arial', 24, 'bold'),
            bg='#1976d2',
            fg='white'
        )
        title_label.pack()
        
        # Main container
        main_frame = tk.Frame(self.root, bg='#f0f0f0')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Left panel - Controls
        left_panel = tk.Frame(main_frame, bg='white', relief=tk.RIDGE, bd=2)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        self.setup_control_panel(left_panel)
        
        # Right panel - Canvas
        right_panel = tk.Frame(main_frame, bg='white', relief=tk.RIDGE, bd=2)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.setup_canvas(right_panel)
    
    def setup_control_panel(self, parent):
        """Setup control panel with buttons and inputs."""
        # Graph Selection
        graph_frame = tk.LabelFrame(
            parent, text="Select Graph", font=('Arial', 12, 'bold'),
            bg='white', padx=10, pady=10
        )
        graph_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.graph_var = tk.StringVar(value="Graph 1")
        graphs = [
            "Graph 1 - Simple",
            "Graph 2 - City",
            "Graph 3 - Grid",
            "Graph 4 - Complex",
            "Graph 5 - Corridor",
            "Graph 6 - Bridges"
        ]
        
        for graph_name in graphs:
            rb = tk.Radiobutton(
                graph_frame, text=graph_name, variable=self.graph_var,
                value=graph_name.split()[1], font=('Arial', 10),
                bg='white', command=self.change_graph
            )
            rb.pack(anchor=tk.W, pady=2)
        
        # Node Selection
        node_frame = tk.LabelFrame(
            parent, text="Select Nodes", font=('Arial', 12, 'bold'),
            bg='white', padx=10, pady=10
        )
        node_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Label(node_frame, text="Start Node:", font=('Arial', 10), bg='white').grid(
            row=0, column=0, sticky=tk.W, pady=5
        )
        self.start_entry = tk.Entry(node_frame, font=('Arial', 10), width=10)
        self.start_entry.grid(row=0, column=1, pady=5, padx=5)
        self.start_entry.insert(0, "A")
        
        tk.Label(node_frame, text="Goal Node:", font=('Arial', 10), bg='white').grid(
            row=1, column=0, sticky=tk.W, pady=5
        )
        self.goal_entry = tk.Entry(node_frame, font=('Arial', 10), width=10)
        self.goal_entry.grid(row=1, column=1, pady=5, padx=5)
        self.goal_entry.insert(0, "G")
        
        # Control Buttons
        button_frame = tk.Frame(parent, bg='white', pady=10)
        button_frame.pack(fill=tk.X, padx=10)
        
        self.run_button = tk.Button(
            button_frame, text="▶ Run A*", font=('Arial', 12, 'bold'),
            bg='#2196f3', fg='white', command=self.run_astar,
            relief=tk.RAISED, bd=3, padx=20, pady=10
        )
        self.run_button.pack(fill=tk.X, pady=5)
        
        self.reset_button = tk.Button(
            button_frame, text="↻ Reset", font=('Arial', 12, 'bold'),
            bg='#757575', fg='white', command=self.reset_visualization,
            relief=tk.RAISED, bd=3, padx=20, pady=10
        )
        self.reset_button.pack(fill=tk.X, pady=5)
        
        # Statistics
        stats_frame = tk.LabelFrame(
            parent, text="Statistics", font=('Arial', 12, 'bold'),
            bg='white', padx=10, pady=10
        )
        stats_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.stats_labels = {}
        stats = [
            ("Nodes Explored:", "0"),
            ("Path Length:", "0"),
            ("Total Cost:", "0.00"),
            ("Time (ms):", "0.00"),
            ("Efficiency:", "0%")
        ]
        
        for i, (label, value) in enumerate(stats):
            tk.Label(
                stats_frame, text=label, font=('Arial', 10, 'bold'),
                bg='white', anchor=tk.W
            ).grid(row=i, column=0, sticky=tk.W, pady=3)
            
            value_label = tk.Label(
                stats_frame, text=value, font=('Arial', 10),
                bg='#e3f2fd', anchor=tk.E, relief=tk.SUNKEN, padx=5
            )
            value_label.grid(row=i, column=1, sticky=tk.EW, pady=3, padx=(5, 0))
            self.stats_labels[label] = value_label
        
        stats_frame.columnconfigure(1, weight=1)
        
        # Legend
        legend_frame = tk.LabelFrame(
            parent, text="Legend", font=('Arial', 12, 'bold'),
            bg='white', padx=10, pady=10
        )
        legend_frame.pack(fill=tk.X, padx=10, pady=10)
        
        legends = [
            ("Start Node", self.colors['start']),
            ("Goal Node", self.colors['goal']),
            ("Visited Node", self.colors['visited']),
            ("Path Node", self.colors['path']),
            ("Unvisited Node", self.colors['node'])
        ]
        
        for i, (text, color) in enumerate(legends):
            color_box = tk.Label(
                legend_frame, bg=color, width=3, height=1,
                relief=tk.RAISED, bd=2
            )
            color_box.grid(row=i, column=0, pady=3, padx=(0, 10))
            
            tk.Label(
                legend_frame, text=text, font=('Arial', 9),
                bg='white', anchor=tk.W
            ).grid(row=i, column=1, sticky=tk.W, pady=3)
    
    def setup_canvas(self, parent):
        """Setup canvas for graph visualization."""
        canvas_frame = tk.Frame(parent, bg='white')
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        tk.Label(
            canvas_frame, text="Graph Visualization",
            font=('Arial', 14, 'bold'), bg='white'
        ).pack(pady=(0, 10))
        
        self.canvas = tk.Canvas(
            canvas_frame, bg='#fafafa', highlightthickness=2,
            highlightbackground='#1976d2'
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
    
    def load_sample_graph(self):
        """Load the initial sample graph."""
        self.change_graph()
    
    def change_graph(self):
        """Change the displayed graph based on selection."""
        graph_num = self.graph_var.get()
        
        if graph_num == "1":
            self.load_graph_1()
        elif graph_num == "2":
            self.load_graph_2()
        elif graph_num == "3":
            self.load_graph_3()
        elif graph_num == "4":
            self.load_graph_4()
        elif graph_num == "5":
            self.load_graph_5()
        else:
            self.load_graph_6()
        
        self.reset_visualization()
        self.draw_graph()
    
    def load_graph_1(self):
        """Load simple network graph."""
        self.graph = {
            'A': {'B': 2, 'C': 3},
            'B': {'A': 2, 'D': 5, 'E': 1},
            'C': {'A': 3, 'F': 8},
            'D': {'B': 5, 'G': 2},
            'E': {'B': 1, 'G': 4},
            'F': {'C': 8, 'G': 3},
            'G': {'D': 2, 'E': 4, 'F': 3}
        }
        
        self.heuristic = {
            'A': 7, 'B': 6, 'C': 5, 'D': 2,
            'E': 4, 'F': 3, 'G': 0
        }
        
        self.node_positions = {
            'A': (100, 150), 'B': (250, 100), 'C': (250, 200),
            'D': (400, 80), 'E': (400, 150), 'F': (400, 220),
            'G': (550, 150)
        }
        
        self.start_entry.delete(0, tk.END)
        self.start_entry.insert(0, "A")
        self.goal_entry.delete(0, tk.END)
        self.goal_entry.insert(0, "G")
    
    def load_graph_2(self):
        """Load city navigation graph."""
        self.graph = {
            'Start': {'A': 1, 'B': 4},
            'A': {'Start': 1, 'C': 2, 'B': 2},
            'B': {'Start': 4, 'A': 2, 'D': 5},
            'C': {'A': 2, 'E': 3, 'F': 6},
            'D': {'B': 5, 'F': 2, 'Goal': 8},
            'E': {'C': 3, 'Goal': 4},
            'F': {'C': 6, 'D': 2, 'Goal': 3},
            'Goal': {'D': 8, 'E': 4, 'F': 3}
        }
        
        self.heuristic = {
            'Start': 10, 'A': 9, 'B': 8, 'C': 7,
            'D': 6, 'E': 4, 'F': 3, 'Goal': 0
        }
        
        self.node_positions = {
            'Start': (80, 150), 'A': (200, 100), 'B': (200, 200),
            'C': (320, 80), 'D': (320, 220), 'E': (440, 100),
            'F': (440, 180), 'Goal': (560, 150)
        }
        
        self.start_entry.delete(0, tk.END)
        self.start_entry.insert(0, "Start")
        self.goal_entry.delete(0, tk.END)
        self.goal_entry.insert(0, "Goal")
    
    def load_graph_3(self):
        """Load grid-based graph."""
        self.graph = {}
        self.heuristic = {}
        self.node_positions = {}
        
        # Create 4x4 grid
        size = 4
        spacing_x = 120
        spacing_y = 120
        offset_x = 100
        offset_y = 80
        
        for i in range(size):
            for j in range(size):
                node = f"{i},{j}"
                self.graph[node] = {}
                
                # Add connections to adjacent nodes
                if i > 0:
                    self.graph[node][f"{i-1},{j}"] = 1
                if i < size - 1:
                    self.graph[node][f"{i+1},{j}"] = 1
                if j > 0:
                    self.graph[node][f"{i},{j-1}"] = 1
                if j < size - 1:
                    self.graph[node][f"{i},{j+1}"] = 1
                
                # Manhattan distance to goal (3,3)
                self.heuristic[node] = abs(i - (size-1)) + abs(j - (size-1))
                
                # Position
                self.node_positions[node] = (
                    offset_x + j * spacing_x,
                    offset_y + i * spacing_y
                )
        
        self.start_entry.delete(0, tk.END)
        self.start_entry.insert(0, "0,0")
        self.goal_entry.delete(0, tk.END)
        self.goal_entry.insert(0, "3,3")
    
    def load_graph_4(self):
        """Load complex graph."""
        self.graph = {
            'S': {'A': 3, 'B': 5, 'C': 2},
            'A': {'S': 3, 'D': 2, 'E': 4},
            'B': {'S': 5, 'E': 2, 'F': 6},
            'C': {'S': 2, 'F': 3},
            'D': {'A': 2, 'G': 3},
            'E': {'A': 4, 'B': 2, 'G': 2, 'H': 5},
            'F': {'B': 6, 'C': 3, 'H': 4},
            'G': {'D': 3, 'E': 2, 'T': 4},
            'H': {'E': 5, 'F': 4, 'T': 3},
            'T': {'G': 4, 'H': 3}
        }
        
        self.heuristic = {
            'S': 9, 'A': 7, 'B': 6, 'C': 8,
            'D': 5, 'E': 4, 'F': 5, 'G': 3,
            'H': 3, 'T': 0
        }
        
        self.node_positions = {
            'S': (100, 200), 'A': (220, 120), 'B': (220, 200),
            'C': (220, 280), 'D': (340, 80), 'E': (340, 160),
            'F': (340, 240), 'G': (460, 120), 'H': (460, 220),
            'T': (580, 170)
        }
        
        self.start_entry.delete(0, tk.END)
        self.start_entry.insert(0, "S")
        self.goal_entry.delete(0, tk.END)
        self.goal_entry.insert(0, "T")
    
    def load_graph_5(self):
        """Load corridor-style weighted graph."""
        self.graph = {
            'H': {'I': 2, 'K': 3},
            'I': {'H': 2, 'J': 2, 'L': 2},
            'J': {'I': 2, 'M': 3, 'Goal': 4},
            'K': {'H': 3, 'L': 2},
            'L': {'K': 2, 'I': 2, 'M': 2},
            'M': {'L': 2, 'J': 3, 'Goal': 2},
            'Goal': {'J': 4, 'M': 2}
        }

        self.heuristic = {
            'H': 7, 'I': 5, 'J': 3,
            'K': 7, 'L': 5, 'M': 3,
            'Goal': 0
        }

        self.node_positions = {
            'H': (120, 120), 'I': (260, 120), 'J': (400, 120),
            'K': (120, 240), 'L': (260, 240), 'M': (400, 240),
            'Goal': (520, 180)
        }

        self.start_entry.delete(0, tk.END)
        self.start_entry.insert(0, "H")
        self.goal_entry.delete(0, tk.END)
        self.goal_entry.insert(0, "Goal")

    def load_graph_6(self):
        """Load bridge-and-bottleneck graph."""
        self.graph = {
            'P': {'Q': 3, 'R': 4},
            'Q': {'P': 3, 'S': 2, 'U': 6},
            'R': {'P': 4, 'S': 3, 'T': 2},
            'S': {'Q': 2, 'R': 3, 'T': 2, 'U': 3},
            'T': {'R': 2, 'S': 2, 'V': 3},
            'U': {'Q': 6, 'S': 3, 'Goal': 3, 'V': 2},
            'V': {'T': 3, 'U': 2, 'Goal': 2},
            'Goal': {'U': 3, 'V': 2}
        }

        self.heuristic = {
            'P': 8, 'Q': 7, 'R': 7,
            'S': 4, 'T': 4, 'U': 2,
            'V': 2, 'Goal': 0
        }

        self.node_positions = {
            'P': (80, 180), 'Q': (200, 100), 'R': (200, 260),
            'S': (340, 160), 'T': (340, 260),
            'U': (480, 120), 'V': (480, 220),
            'Goal': (600, 180)
        }

        self.start_entry.delete(0, tk.END)
        self.start_entry.insert(0, "P")
        self.goal_entry.delete(0, tk.END)
        self.goal_entry.insert(0, "Goal")

    def draw_graph(self):
        """Draw the graph on canvas."""
        self.canvas.delete("all")
        self.node_circles.clear()
        self.edge_lines.clear()
        
        # Draw edges first (so they appear behind nodes)
        drawn_edges = set()
        for node, neighbors in self.graph.items():
            x1, y1 = self.node_positions[node]
            for neighbor, weight in neighbors.items():
                edge_key = tuple(sorted([node, neighbor]))
                if edge_key not in drawn_edges:
                    x2, y2 = self.node_positions[neighbor]
                    
                    line = self.canvas.create_line(
                        x1, y1, x2, y2, fill=self.colors['edge'],
                        width=2, tags="edge"
                    )
                    self.edge_lines[edge_key] = line
                    
                    # Draw weight
                    mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
                    self.canvas.create_text(
                        mid_x, mid_y, text=str(weight),
                        font=('Arial', 9, 'bold'), fill='#d32f2f',
                        tags="weight"
                    )
                    
                    drawn_edges.add(edge_key)
        
        # Draw nodes
        for node, (x, y) in self.node_positions.items():
            circle = self.canvas.create_oval(
                x-20, y-20, x+20, y+20,
                fill=self.colors['node'], outline='#1976d2',
                width=2, tags="node"
            )
            self.node_circles[node] = circle
            
            # Node label
            self.canvas.create_text(
                x, y-5, text=node, font=('Arial', 11, 'bold'),
                fill=self.colors['text'], tags="label"
            )
            
            # Heuristic value
            h_val = self.heuristic.get(node, 0)
            self.canvas.create_text(
                x, y+8, text=f"h={h_val}", font=('Arial', 8),
                fill='#666', tags="heuristic"
            )
    
    def reset_visualization(self):
        """Reset the visualization to initial state."""
        self.visited_nodes.clear()
        self.current_path.clear()
        self.nodes_explored = 0
        self.path_cost = 0
        self.execution_time = 0
        
        # Reset statistics display
        self.stats_labels["Nodes Explored:"].config(text="0")
        self.stats_labels["Path Length:"].config(text="0")
        self.stats_labels["Total Cost:"].config(text="0.00")
        self.stats_labels["Time (ms):"].config(text="0.00")
        self.stats_labels["Efficiency:"].config(text="0%")
        
        # Redraw graph
        self.draw_graph()
    
    def run_astar(self):
        """Run A* algorithm with visualization."""
        if self.is_running:
            return
        
        start = self.start_entry.get().strip()
        goal = self.goal_entry.get().strip()
        
        if start not in self.graph:
            messagebox.showerror("Error", f"Start node '{start}' not found in graph!")
            return
        
        if goal not in self.graph:
            messagebox.showerror("Error", f"Goal node '{goal}' not found in graph!")
            return
        
        self.reset_visualization()
        self.is_running = True
        self.run_button.config(state=tk.DISABLED)
        
        # Start algorithm
        self.root.after(100, lambda: self.astar_algorithm(start, goal))
    
    def astar_algorithm(self, start, goal):
        """A* pathfinding algorithm with visualization."""
        start_time = time.time()
        
        # Priority queue: (f_score, node)
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic.get(start, 0), start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic.get(start, 0)}
        visited = set()
        
        # For step-by-step visualization
        self.animate_search(
            open_set, came_from, g_score, f_score, visited, goal, start_time
        )
    
    def animate_search(self, open_set, came_from, g_score, f_score, visited, goal, start_time):
        """Animate one step of A* search."""
        if not open_set:
            # No path found
            messagebox.showinfo("Result", "No path found!")
            self.is_running = False
            self.run_button.config(state=tk.NORMAL)
            return
        
        current_f, current = heapq.heappop(open_set)
        
        if current in visited:
            self.root.after(50, lambda: self.animate_search(
                open_set, came_from, g_score, f_score, visited, goal, start_time
            ))
            return
        
        visited.add(current)
        self.nodes_explored += 1
        
        # Update visualization
        self.color_node(current, 'visited')
        self.stats_labels["Nodes Explored:"].config(text=str(self.nodes_explored))
        
        # Goal reached
        if current == goal:
            path = self.reconstruct_path(came_from, current)
            self.path_cost = g_score[current]
            self.execution_time = (time.time() - start_time) * 1000
            
            # Highlight path
            self.root.after(200, lambda: self.highlight_path(path))
            return
        
        # Explore neighbors
        if current in self.graph:
            for neighbor, weight in self.graph[current].items():
                if neighbor in visited:
                    continue
                
                tentative_g = g_score[current] + weight
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic.get(neighbor, 0)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # Continue animation
        self.root.after(300, lambda: self.animate_search(
            open_set, came_from, g_score, f_score, visited, goal, start_time
        ))
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.insert(0, current)
        return path
    
    def highlight_path(self, path):
        """Highlight the found path."""
        self.current_path = path
        
        # Color start and goal
        if path:
            self.color_node(path[0], 'start')
            self.color_node(path[-1], 'goal')
            
            # Color path nodes
            for node in path[1:-1]:
                self.color_node(node, 'path')
        
        # Update statistics
        self.stats_labels["Path Length:"].config(text=str(len(path)))
        self.stats_labels["Total Cost:"].config(text=f"{self.path_cost:.2f}")
        self.stats_labels["Time (ms):"].config(text=f"{self.execution_time:.2f}")
        
        if self.nodes_explored > 0:
            efficiency = (len(path) / self.nodes_explored) * 100
            self.stats_labels["Efficiency:"].config(text=f"{efficiency:.1f}%")
        
        # Show result message
        path_str = " → ".join(path)
        messagebox.showinfo(
            "Path Found!",
            f"Path: {path_str}\n\n"
            f"Length: {len(path)} nodes\n"
            f"Cost: {self.path_cost:.2f}\n"
            f"Nodes Explored: {self.nodes_explored}\n"
            f"Time: {self.execution_time:.2f} ms"
        )
        
        self.is_running = False
        self.run_button.config(state=tk.NORMAL)
    
    def color_node(self, node, node_type):
        """Color a node based on its type."""
        if node in self.node_circles:
            color = self.colors[node_type]
            self.canvas.itemconfig(self.node_circles[node], fill=color)
            self.root.update()


def main():
    """Main function to run the visualizer."""
    root = tk.Tk()
    app = AStarVisualizer(root)
    root.mainloop()


if __name__ == "__main__":
    main()