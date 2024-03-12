import cTheia

graph = cTheia.TraversalGraph('src/isaac_sim_theia/graphs/test_world_1_graph.json')
print(graph.nodes[0].edges)