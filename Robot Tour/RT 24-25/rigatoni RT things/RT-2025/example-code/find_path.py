import math

# Define types for readability
Coordinate = tuple[int, int]

class Track:
    def __init__(self, 
                 start_point: Coordinate, 
                 target_point: Coordinate, 
                 obstacles: list[Coordinate], 
                 gate_zones: list[Coordinate], 
                 target_time: int):
        """Initialize the track setup with given start, target, obstacles, gate zones, and target time."""
        self.track_size = 200  # Track is 200mm x 200mm
        self.grid_size = 50    # Each grid cell is 50mm x 50mm

        # Validate start point
        if not self.is_valid_start_point(start_point):
            raise ValueError(f"Invalid start point: {start_point}. Start point must be on the perimeter at a grid edge center.")
        
        # Validate target point
        if not self.is_valid_grid_center(target_point):
            raise ValueError(f"Invalid target point: {target_point}. Target point must be at a valid grid center within track boundaries.")
        
        # Validate obstacles
        for obstacle in obstacles:
            if not self.is_valid_obstacle_position(obstacle):
                raise ValueError(f"Invalid obstacle position: {obstacle}. Obstacles must be at grid edge centers only.")

        # Assign values after validation
        self.start_point = start_point
        self.target_point = target_point
        self.obstacles = obstacles
        self.gate_zones = gate_zones
        self.target_time = target_time

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
        return 0 <= x <= self.track_size and 0 <= y <= self.track_size and x % self.grid_size == 0 and y % self.grid_size == 0

    def is_valid_obstacle_position(self, point: Coordinate) -> bool:
        """Check if a point is valid as a grid edge center within track boundaries."""
        return self.is_valid_edge_center(point)

    def is_valid_edge_center(self, point: Coordinate) -> bool:
        """Check if a point is a valid edge center on a grid edge."""
        x, y = point
        return (
            (x % self.grid_size == 0 and y % self.grid_size == self.grid_size // 2) or
            (y % self.grid_size == 0 and x % self.grid_size == self.grid_size // 2)
        ) and 0 <= x <= self.track_size and 0 <= y <= self.track_size

    def calculate_distance(self, p1: Coordinate, p2: Coordinate) -> float:
        """Calculate the Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def is_obstructed(self, point: Coordinate) -> bool:
        """Check if a point has an obstacle."""
        return point in self.obstacles

    def is_obstacle_between(self, grid1: Coordinate, grid2: Coordinate) -> bool:
        """Check if there is an obstacle directly between two adjacent grid centers."""
        # Validate adjacency
        dx = abs(grid1[0] - grid2[0])
        dy = abs(grid1[1] - grid2[1])
        if (dx == 0 and dy == self.grid_size) or (dx == self.grid_size and dy == 0):  # Adjacent if 50mm apart in one direction
            # Check for obstacle in between
            if grid1[0] == grid2[0]:  # Moving vertically
                x = grid1[0]
                y_between = (grid1[1] + grid2[1]) // 2
                return (x, y_between) in self.obstacles
            elif grid1[1] == grid2[1]:  # Moving horizontally
                y = grid1[1]
                x_between = (grid1[0] + grid2[0]) // 2
                return (x_between, y) in self.obstacles
        else:
            # Raise an exception if grid1 and grid2 are not adjacent
            raise ValueError(f"The grids {grid1} and {grid2} are not adjacent.")
        return False  # No obstacle in between if adjacent and clear

    def is_gate_zone(self, point: Coordinate) -> bool:
        """Check if a point is in a gate zone."""
        return point in self.gate_zones

    def adjacent_grids_without_obstacle(self, grid: Coordinate) -> list[Coordinate]:
        """Return a list of adjacent grids without obstacles between them and the given grid."""
        x, y = grid
        adjacent_grids = []

        # Check each of the four possible adjacent grids (up, down, left, right)
        potential_adjacent = [
            (x, y - self.grid_size),  # Above
            (x, y + self.grid_size),  # Below
            (x - self.grid_size, y),  # Left
            (x + self.grid_size, y)   # Right
        ]

        for adj in potential_adjacent:
            if 0 <= adj[0] < self.track_size and 0 <= adj[1] < self.track_size:  # Ensure within track boundaries
                if not self.is_obstacle_between(grid, adj):
                    adjacent_grids.append(adj)
        
        return adjacent_grids

    def __repr__(self):
        return (f"Track(start_point={self.start_point}, target_point={self.target_point}, "
                f"obstacles={self.obstacles}, gate_zones={self.gate_zones}, target_time={self.target_time})")

# Example usage
try:
    track = Track(
        start_point=(0, 125),            # Valid start point on the perimeter edge center
        target_point=(150, 150),         # Valid target point at grid center
        obstacles=[(50, 25), (100, 25), (150, 25),   # Top edge obstacles (valid edge center positions)
                   (50, 175), (100, 175), (150, 175)],  # Bottom edge obstacles (valid edge center positions)
        gate_zones=[(50, 150), (150, 50)],  # Gate zones in mm at grid centers
        target_time=60                   # Target time for the run
    )
    print(track.adjacent_grids_without_obstacle((100, 100)))  # Example grid center (100, 100)
except ValueError as e:
    print(e)