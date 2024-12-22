"""
Assignment 4: Agent-Based Model for Structural Tessellation Generation

Author: Martin Forná 

Description:
This script implements an agent-based model using Object-Oriented Programming (OOP) principles.
It simulates the behavior of agents to generate structural patterns, exploring how changes in
rules and parameters affect the resulting form.

The script explores attraction of agents to specified attractor points, as well as repulsing agents from each other
A small amount of randomness has been added to add a slight erraticness to their movement, so it is less detmerministic
The script has many parameters that can be adjusted to achive different results
Finally the script will create a Delaunay triangulated mesh from the final positions of the agents.

Note: This script is intended to be used within Grasshopper's Python scripting component.
Note: The script contains multiple codeblocks which are intended to run in their own Python scripting component. These have been marked accordingly.
"""
# Import necessary libraries
import Rhino.Geometry as rg
import random
import math
import scipy.spatial
import numpy as np

class Agent:
    """
    A class to represent an agent in the simulation.

    Attributes:
    - position: The current position of the agent (Point3d).
    - velocity: The current velocity of the agent (tuple).
    - surface: The surface on which the agent is confined. (NurbsSurface)

    Methods:
    - move(): Updates the agent's position and keeps it on the surface.
    - interact(): Defines how the agent interacts with attractors and other agents.
    - update(): Updates the agent's state.
    """
    def __init__(self, position, velocity, surface):
        self.position = position
        self.velocity = velocity
        self.surface = surface

    def move(self):
        """
        Update the agent's position based on its velocity and restrict it to the surface.
        """
        
        new_position = rg.Point3d(
            self.position.X + self.velocity[0],
            self.position.Y + self.velocity[1],
            self.position.Z + self.velocity[2]
        )

        # We use this to make sure the agents cannot leave the surface
        success, u, v = self.surface.ClosestPoint(new_position)
        if success:
            self.position = self.surface.PointAt(u, v)

    def interact(self, attractors, radius, max_attract_distance, agents, repulse_radius, repulse_force, attract_force):
        """
        Define interactions with attractor points and repulsion from nearby agents.
        Attributes:
        - attractors: Points to which the agents are attracted (Point3dList)
        - attract_force: The intensity of which the agents are attracted to the attractor points (float).
        - repulse_force: The intensity of which the agents are repulsed by each other (float).
        - surface: The surface on which the agent is confined. (NurbsSurface)
        - max_attract_distance: Radius of the area of effect for the attraction of agents. I.E. Agents outside of this distance are not affect by attraction (float).
        - repulse_radius: Radius of area of effect for the repulsion of agents. I.E. Agents outside of this distance are not affect by repulsion (float).
        """
        # Attraction logic
        closest_attractor = None
        closest_distance = float('inf')

        for attractor in attractors:
            distance = self.position.DistanceTo(attractor)
            if radius < distance <= max_attract_distance and distance < closest_distance:
                closest_attractor = attractor
                closest_distance = distance

        attractor_velocity = rg.Vector3d(0, 0, 0)
        if closest_attractor:
            direction = rg.Vector3d(
                closest_attractor.X - self.position.X,
                closest_attractor.Y - self.position.Y,
                closest_attractor.Z - self.position.Z
            )
            direction.Unitize()
            attractor_velocity = direction * (attract_force / (closest_distance + 0.01)) 

        # Repulsion logic
        repulsion_velocity = rg.Vector3d(0, 0, 0)
        for other_agent in agents:
            if other_agent is not self:
                distance = self.position.DistanceTo(other_agent.position)
                if distance < repulse_radius:
                    direction = rg.Vector3d(
                        self.position.X - other_agent.position.X,
                        self.position.Y - other_agent.position.Y,
                        self.position.Z - other_agent.position.Z
                    )
                    direction.Unitize()
                    repulsion_velocity += direction * (repulse_force / (distance + 0.01))  

        # Combine attractor and repulsion velocities to get a singular veloctiy for the agents
        self.velocity = (
            attractor_velocity.X + repulsion_velocity.X,
            attractor_velocity.Y + repulsion_velocity.Y,
            attractor_velocity.Z + repulsion_velocity.Z
        )

    def update(self):
        """
        Update the agent's state with random noise in movement to make it less detmerministic/stiff, and more erratic.
        """

        noise_x = random.uniform(-0.1, 0.1)
        noise_y = random.uniform(-0.1, 0.1)
        noise_z = random.uniform(-0.1, 0.1)

        
        self.velocity = (
            self.velocity[0] + noise_x,
            self.velocity[1] + noise_y,
            self.velocity[2] + noise_z
        )
        pass

# Simulation parameters
agents = []       
# num_agents: number of agents used in the simulation (int)
# num_steps: number of times the simulation is ran (int)

# Initialize agents with uniform distribution over the surface
u_domain = surface.Domain(0)  
v_domain = surface.Domain(1)  

grid_size = int(math.sqrt(num_agents)) 
u_values = [u_domain.Min + (u_domain.Length / (grid_size - 1)) * i for i in range(grid_size)]
v_values = [v_domain.Min + (v_domain.Length / (grid_size - 1)) * i for i in range(grid_size)]

for u in u_values:
    for v in v_values:
        if len(agents) < num_agents:
            position = surface.PointAt(u, v)

            # plots the agents intial postion, while setting the velocity to zero.
            velocity = (0, 0, 0)
            agent = Agent(position, velocity, surface)
            agents.append(agent)

# Simulation loop for all the agents over the specified number of steps
for step in range(num_steps):
    for agent in agents:
        agent.interact(attractors, attraction_radius, max_attract_distance, agents, repulse_radius, repulse_force, attract_force)
        agent.move()
        agent.update()

# After simulation, process results
agent_positions = [rg.Point3d(agent.position.X, agent.position.Y, agent.position.Z) for agent in agents]
agent_velocity = [rg.Point3d(agent.velocity[0], agent.velocity[1], agent.velocity[2]) for agent in agents]


# Generate a Delaunay mesh from the final agent positions
# Convert Rhino Point3d to numpy array for scipy
points_np = np.array([(pt.X, pt.Y, pt.Z) for pt in agent_positions])

# Compute Convex Hull using scipy.spatial.ConvexHull
hull = scipy.spatial.ConvexHull(points_np)

# Create a mesh from the convex hull vertices
mesh = rg.Mesh()
for pt in points_np:
    mesh.Vertices.Add(pt[0], pt[1], pt[2])

# Apply Delaunay triangulation from the hull points (for 2D projection of points)
hull_points_2d = np.array([(pt[0], pt[1]) for pt in points_np])
delaunay = scipy.spatial.Delaunay(hull_points_2d)

# Create faces from Delaunay triangulation
for simplex in delaunay.simplices:
    mesh.Faces.AddFace(int(simplex[0]), int(simplex[1]), int(simplex[2]))

# Output the mesh
Delaunay_mesh = mesh

# Now, trace the edges of the Delaunay triangulation
delaunay_lines = []
for simplex in delaunay.simplices:
    # Create lines between the points that form the edges of the triangle of each cell in the mesh
    pt1 = points_np[simplex[0]]
    pt2 = points_np[simplex[1]]
    pt3 = points_np[simplex[2]]

    line1 = rg.Line(rg.Point3d(pt1[0], pt1[1], pt1[2]), rg.Point3d(pt2[0], pt2[1], pt2[2]))
    line2 = rg.Line(rg.Point3d(pt2[0], pt2[1], pt2[2]), rg.Point3d(pt3[0], pt3[1], pt3[2]))
    line3 = rg.Line(rg.Point3d(pt3[0], pt3[1], pt3[2]), rg.Point3d(pt1[0], pt1[1], pt1[2]))

    delaunay_lines.append(line1)
    delaunay_lines.append(line2)
    delaunay_lines.append(line3)

# Output the lines
Delaunay_edges = delaunay_lines


"""
Codeblocks below this point are NOT intended to be used together with the above script.
They are inteded to be used in their own Python scripting component in Grasshopper
The following two codeblocks will describe how the attractor points have been generated.
The first codeblock will describe how the ranom attractor points are generated
The second codeblock will describe how the specified attractor points are generated. These points have been intended to simulte where the supports would be located.
"""
### Codeblock one ###

# Import necessary libraries
import rhinoscriptsyntax as rs
import random

# Required inputs:
# Surface: the surface the points will be generated one (Point3d).
# Num_points: the number of desired, randomly placed, attractor points (int).


def generate_attraction_points(surface, num_points):
    points = []
    
    # Get the domain of the surface in U and V directions
    u_domain = rs.SurfaceDomain(surface, 0)  
    v_domain = rs.SurfaceDomain(surface, 1) 
    
    for i in range(num_points):
        # Generate random UV coordinates within the surface's domain
        u = random.uniform(u_domain[0], u_domain[1]) 
        v = random.uniform(v_domain[0], v_domain[1])  
        
        # Evaluate the point on the surface
        point = rs.SurfaceEvaluate(surface, (u, v), 0)[0] 
        points.append(point)
    
    return points

attractors = generate_attraction_points(surface, num_points) 

### CODEBLOCK TWO ###

# Import necessary libraries
import rhinoscriptsyntax as rs

# Required input
# Surface: The surface that the points will be generated on

def generate_attraction_points(surface):
    points = []
    
    # Get the domain of the surface in U and V directions
    u_domain = rs.SurfaceDomain(surface, 0)  
    v_domain = rs.SurfaceDomain(surface, 1) 
    
    # Get the corners of the surface
    corner_1 = rs.SurfaceEvaluate(surface, (u_domain[0], v_domain[0]), 0)[0]
    corner_2 = rs.SurfaceEvaluate(surface, (u_domain[1], v_domain[0]), 0)[0]
    corner_3 = rs.SurfaceEvaluate(surface, (u_domain[1], v_domain[1]), 0)[0]
    corner_4 = rs.SurfaceEvaluate(surface, (u_domain[0], v_domain[1]), 0)[0]
    
    # Get the center of the surface 
    center_u = (u_domain[0] + u_domain[1]) / 2
    center_v = (v_domain[0] + v_domain[1]) / 2
    center = rs.SurfaceEvaluate(surface, (center_u, center_v), 0)[0]
    
    # Calculate the halfway points between the center and each corner
    point_1 = rs.SurfaceEvaluate(surface, ((u_domain[0] + center_u) / 2, (v_domain[0] + center_v) / 2), 0)[0]
    point_2 = rs.SurfaceEvaluate(surface, ((u_domain[1] + center_u) / 2, (v_domain[0] + center_v) / 2), 0)[0]
    point_3 = rs.SurfaceEvaluate(surface, ((u_domain[1] + center_u) / 2, (v_domain[1] + center_v) / 2), 0)[0]
    point_4 = rs.SurfaceEvaluate(surface, ((u_domain[0] + center_u) / 2, (v_domain[1] + center_v) / 2), 0)[0]
    
    points.extend([point_1, point_2, point_3, point_4])

    return points


attractors = generate_attraction_points(surface)



"""
This final codeblock describes how the distance between each agents and the attractor points is calculated.
The minimum distance to each attractor point is then used to color the point in a gradient.
"""

# Import necessary libraries
import Rhino.Geometry as rg

# Inputs:
# agents: A list of agent positions (list of Point3d)
# attractors: A list of attractor points (list of Point3d)

# Initialize a list to store the minimum distances for each agent
min_distances = []

# Loop through each agent
for agent in agents:
    # Initialize a variable to hold the minimum distance to an attractor
    min_distance = float('inf')
    
    # Loop through each attractor
    for attractor in attractors:
        # Calculate the distance between the agent and the attractor
        distance = agent.DistanceTo(attractor)
        
        # Update the minimum distance if this attractor is closer
        if distance < min_distance:
            min_distance = distance
    
    # Add the minimum distance for this agent to the list
    min_distances.append(min_distance)

# Output: List of minimum distances from agents to the closest attractor
a = min_distances

