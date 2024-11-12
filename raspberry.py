import os
import math
import heapq
import requests
import paho.mqtt.client as mqtt
import threading
from concurrent.futures import ThreadPoolExecutor

# MQTT broker details
broker = "broker.hivemq.com"  # MQTT broker address
port = 1883  # Port for MQTT communication
topic = "Home/FireAlert"  # Topic for subscribing to fire alert notifications

# Firebase setup
database_secret = os.getenv('FIREBASE_SECRET')  # Retrieve Firebase database secret from environment variable
database_url = "https://embedded-1system-default-rtdb.firebaseio.com"  # Firebase database URL

# Check if the Firebase secret is set
if not database_secret:
    print("Error: FIREBASE_SECRET environment variable is not set.")
    exit(1)

# Define points and roads representing locations and connections between them
points = { ... }  # Placeholder for a dictionary of points with coordinates
roads = [ ... ]   # Placeholder for a list of roads (connections between points)

# Function to calculate the Euclidean distance between two points
def calculate_distance(pointA, pointB):
    x1, y1 = pointA
    x2, y2 = pointB
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Create a graph using the list of roads and points
def create_graph(roads, points):
    graph = {}
    for road in roads:
        start, end = road['start'], road['end']
        distance = calculate_distance(points[start], points[end])  # Calculate distance between start and end points
        if start not in graph:
            graph[start] = []  # Initialize adjacency list for start point
        if end not in graph:
            graph[end] = []  # Initialize adjacency list for end point
        graph[start].append({'node': end, 'distance': distance})  # Add edge from start to end
        graph[end].append({'node': start, 'distance': distance})  # Add edge from end to start (bidirectional)
    return graph

# Implementation of Dijkstra's algorithm to find the shortest path
def dijkstra(graph, start_node, end_node):
    distances = {node: float('inf') for node in graph}  # Initialize distances to infinity
    previous_nodes = {node: None for node in graph}  # Track previous node in path
    distances[start_node] = 0  # Distance from start node to itself is zero
    priority_queue = [(0, start_node)]  # Priority queue with initial node

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)  # Get node with shortest distance
        if current_node == end_node:  # Stop if end node is reached
            break
        for neighbor in graph[current_node]:  # Iterate through neighboring nodes
            neighbor_node = neighbor['node']
            new_distance = current_distance + neighbor['distance']
            if new_distance < distances[neighbor_node]:  # Update distance if shorter path is found
                distances[neighbor_node] = new_distance
                previous_nodes[neighbor_node] = current_node  # Update path
                heapq.heappush(priority_queue, (new_distance, neighbor_node))

    # Reconstruct shortest path
    path = []
    current_node = end_node
    while current_node is not None:
        path.append(current_node)
        current_node = previous_nodes[current_node]

    return path[::-1], distances[end_node]  # Return reversed path and total distance

# Wrapper to find the shortest path between two points
def find_shortest_path(start_node, end_node):
    if start_node not in points or end_node not in points:
        return "", float('inf')
    graph = create_graph(roads, points)  # Build the graph from points and roads
    path, total_distance = dijkstra(graph, start_node, end_node)  # Run Dijkstra's algorithm
    path_str = ",".join(path)  # Convert path list to comma-separated string
    return path_str, total_distance  # Return path and distance

# Function to update Firebase with path data
def update_firebase_path(path_name, path_value):
    data = {path_name: path_value}
    # Send HTTP PATCH request to update Firebase with path data
    response = requests.patch(f"{database_url}/{path_name}.json?auth={database_secret}", json=data)
    if response.status_code == 200:
        print(f"Value of '{path_name}' updated to '{path_value}' successfully.")
    else:
        print(f"Failed to update value for '{path_name}':", response.text)

# Calculate shortest paths in parallel using threads
def calculate_and_update_paths():
    with ThreadPoolExecutor(max_workers=2) as executor:
        # Run shortest path calculation for two different start-end pairs in parallel
        future_a = executor.submit(find_shortest_path, 'p29', 'p47')
        future_b = executor.submit(find_shortest_path, 'p25', 'p47')

        # Retrieve results from futures
        path_a, _ = future_a.result()
        path_b, _ = future_b.result()

        # Update Firebase with the calculated paths if found
        if path_a:
            update_firebase_path("PathA", path_a)
        else:
            print("No path found for PathA from p29 to p47.")
        if path_b:
            update_firebase_path("PathB", path_b)
        else:
            print("No path found for PathB from p25 to p47.")

# MQTT on_connect callback function
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT Broker with result code {rc}")
    client.subscribe(topic)  # Subscribe to the specified MQTT topic

# MQTT on_message callback function to handle incoming messages
def on_message(client, userdata, msg):
    print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")
    # Start a new thread to handle path calculation and update to avoid blocking MQTT message handling
    threading.Thread(target=calculate_and_update_paths).start()

# MQTT setup
client = mqtt.Client()  # Create MQTT client
client.on_connect = on_connect  # Attach on_connect callback
client.on_message = on_message  # Attach on_message callback
client.connect(broker, port, 60)  # Connect to the broker

# Start the MQTT client loop to listen for messages indefinitely
client.loop_forever()
