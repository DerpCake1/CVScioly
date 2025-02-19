import math

# Define types for readability
Coordinate = tuple[int, int]

class Track:
    def __init__(self, 
                 start_point: Coordinate, 
                 target_point: str, 
                 obstacles: list[tuple[str, str]], 
                 gate_zones: list[str], 
                 target_time: int):
        """Initialize the track setup with given start, target, obstacles, gate zones, and target time."""
        self.track_size = 200  # Track is 200mm x 200mm
        self.grid_size = 50    # Each grid cell is 50mm x 50mm

        # Validate start point
        self.start_point = start_point
        if not self.is_valid_start_point(self.start_point):
            raise ValueError(f"Invalid start point: {start_point}. Start point must be on the perimeter at a grid edge center.")
        
        # Convert target point to 0-200mm coordinates
        self.target_point = self.convert_to_coordinates(target_point)
        if not self.is_valid_grid_center(self.target_point):
            raise ValueError(f"Invalid target point: {target_point}. Target point must be at a valid grid center within track boundaries.")
        
        # Convert obstacles and gate zones to 0-200mm coordinates
        self.obstacles = [self.convert_to_coordinates(obs1) + self.convert_to_coordinates(obs2) for obs1, obs2 in obstacles]
        self.gate_zones = [self.convert_to_coordinates(gate) for gate in gate_zones]

        # Validate obstacles
        for obstacle in self.obstacles:
            if not self.is_valid_obstacle_position(obstacle[0], obstacle[1]):
                raise ValueError(f"Invalid obstacle between {obstacle[0]} and {obstacle[1]}.")

        self.target_time = target_time

    def convert_to_coordinates(self, grid: str) -> tuple[int, int]:
        """Convert grid identifier (e.g., 'A1') to the center of the grid in 0-200mm coordinates."""
        col, row = grid[0], grid[1]
        
        # Calculate the x-coordinate (horizontal position)
        x = (ord(col.upper()) - ord('A')) * self.grid_size + self.grid_size // 2
        
        # Calculate the y-coordinate (vertical position)
        y = (int(row) - 1) * self.grid_size + self.grid_size // 2
        
        return (x, y)

    def is_valid_start_point(self, point: Coordinate) -> bool:
        """Check if the start point is on the track perimeter at a grid edge center."""
        x, y = point
        # Check if point is on the left or right edge at a grid center
        if (x == 0 or x == self.track_size) and y in {25, 75, 125, 175}:
            return True
        # Check if point is on the top or bottom edge at a grid center
        if (y == 0 or y == self.track_size) and x in {25, 75, 125, 175}:
            return True
        return False

    def is_valid_grid_center(self, point: Coordinate) -> bool:
        """Check if a point is a valid grid center within track boundaries."""
        x, y = point
        return 0 <= x < self.track_size and 0 <= y < self.track_size

    def is_valid_obstacle_position(self, point1: Coordinate, point2: Coordinate) -> bool:
        """Check if the obstacle is between two adjacent grid centers."""
        x1, y1 = point1
        x2, y2 = point2
        
        # Check if the two points are adjacent horizontally or vertically
        if abs(x1 - x2) == self.grid_size and y1 == y2:  # Horizontal adjacency
            return True
        if abs(y1 - y2) == self.grid_size and x1 == x2:  # Vertical adjacency
            return True
        return False

    def get_adjacent_grids_without_obstacle(self, grid: str) -> list[str]:
        """Returns a list of adjacent grids without an obstacle between them."""
        # Convert the input grid to its 0-200mm coordinates
        x, y = self.convert_to_coordinates(grid)
        
        # Define the adjacent grid coordinates (top, bottom, left, right)
        adjacent_grids = [
            (x, y - self.grid_size),  # Top
            (x, y + self.grid_size),  # Bottom
            (x - self.grid_size, y),  # Left
            (x + self.grid_size, y)   # Right
        ]
        
        # List to store valid adjacent grids without obstacles
        valid_adjacent_grids = []

        for adj_x, adj_y in adjacent_grids:
            # Check if the adjacent grid is within the track boundaries
            if 0 <= adj_x < self.track_size and 0 <= adj_y < self.track_size:
                # Convert the adjacent coordinates back to grid labels
                adj_grid = self.convert_to_grid_label(adj_x, adj_y)
                # Check if there is an obstacle between the original grid and the adjacent grid
                if not self.is_obstacle_between((x, y), (adj_x, adj_y)):
                    valid_adjacent_grids.append(adj_grid)
        
        return valid_adjacent_grids

    def convert_to_grid_label(self, x: int, y: int) -> str:
        """Convert 0-200mm coordinates back to grid label (e.g., 'A1')."""
        col = chr((x // self.grid_size) + ord('A'))  # Determine the column (A, B, C, D)
        row = (y // self.grid_size) + 1  # Determine the row (1, 2, 3, 4)
        return f"{col}{row}"

    def is_obstacle_between(self, point1: Coordinate, point2: Coordinate) -> bool:
        """Check if there is an obstacle between two points."""
        for (x1, y1, x2, y2) in self.obstacles:
            if (point1 == (x1, y1) and point2 == (x2, y2)) or (point1 == (x2, y2) and point2 == (x1, y1)):
                return True
        return False

    def __repr__(self):
        return (f"Track(start_point={self.start_point}, target_point={self.target_point}, "
                f"obstacles={self.obstacles}, gate_zones={self.gate_zones}, target_time={self.target_time})")

# Example usage
try:
    track = Track(
        start_point=(0, 25),          # Start point at center of vertical side of first grid (A1)
        target_point='D4',            # Target point at grid D4
        obstacles=[('A1', 'A2'), ('B2', 'B3'), ('C3', 'C4')],  # Obstacles between adjacent grids
        gate_zones=['B1', 'C2'],      # Gate zones
        target_time=60                # Target time to complete the track
    )

    # Test: Get adjacent grids to 'B2' without an obstacle
    print(track.get_adjacent_grids_without_obstacle('B2'))
    
except ValueError as e:
    print(e)
